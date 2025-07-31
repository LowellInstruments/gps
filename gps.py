import time
import pynmea2
import serial
from pynmea2 import ChecksumError
from serial.tools import list_ports
from datetime import datetime, timezone



VID_PID_GPS_PUCK = "067B:23A3"
VID_PID_GPS_PUCK_2 = "067B:2303"
VID_PID_GPS_HAT = "2C7C:0125"
p_mod = 'GPS'
g_last_ymd = ''
using_puck = 0
using_hat = 0



def pm(s):
    # stands for print module
    print(f'{p_mod}: {s}', flush=True)



def gps_get_type_of_antenna():
    if using_puck:
        return 'external'
    return 'internal'



def gps_hat_activate_nmea_output(usb_port):

    # usb_port: '/dev/ttyUSB2'
    bb = bytes()
    ser = None
    rv = False

    try:
        ser = serial.Serial(usb_port, baudrate=115200, timeout=0)
        for i in range(3):
            # probably has echo activated so will receive back this
            ser.write(b'AT+QGPS=1\r')
            time.sleep(.1)
            bb = ser.read(ser.in_waiting)

            # basic GPS debug
            # print('bb', bb)

            if b'OK' in bb:
                rv = True
            if b'AT+QGPS=1' in bb:
                # this is the echo
                rv = True
            if b'CME ERROR: 504' in bb:
                # means was already on
                rv = True
            if rv:
                break

    except (Exception,) as ex:
        pm(f'error gps_activate_hat_output -> {ex}')

    finally:
        if ser:
            ser.close()
        return rv, bb



def gps_hat_get_firmware_version(port_ctrl):

    # usb_port: '/dev/ttyUSB2'
    ser = None
    ans_v = bytes()
    ans_m = bytes()
    pm(f'getting hat\'s firmware version from port {port_ctrl}')

    try:
        ser = serial.Serial(port_ctrl, baudrate=115200, timeout=0)
        ser.write(b"AT+CVERSION\r")
        time.sleep(.1)
        ans_v = ser.read(ser.in_waiting)
        ser.write(b"AT+QGMR\r")
        time.sleep(.1)
        ans_m = ser.read(ser.in_waiting)
        ans_v = ans_v.replace(b"OK", b"")
        ans_v = ans_v.replace(b"\r\n", b"")
        ans_m = ans_m.replace(b"OK", b"")
        ans_m = ans_m.replace(b"\r\n", b"")

    except (Exception,) as ex:
        pm(f'error gps_know_hat_firmware_version -> {ex}')

    finally:
        if ser:
            ser.close()
        return ans_v, ans_m




def gps_usb_find_one_port_for_puck():
    for port, _, vp in sorted(list(list_ports.comports())):
        if (VID_PID_GPS_PUCK in vp) or (VID_PID_GPS_PUCK_2 in vp):
            # port: /dev/ttyUSB0
            pm(f'found puck USB port on {port}')
            return port
    return None



def gps_usb_find_list_of_ports_for_hat():
    ls = []
    for port, _, vp in sorted(list(list_ports.comports())):
        if not VID_PID_GPS_HAT in vp:
            continue
        ls.append(port)
        # ls: ['/dev/ttyUSB0' ... '/dev/ttyUSB3']
    if ls:
        pm(f'found hat list of USB ports on\n\t{ls}')
    return ls



def gps_usb_ports_find_any():
    global using_puck
    global using_hat
    using_hat = 0
    using_puck = 0

    # preference for GPS puck
    p = gps_usb_find_one_port_for_puck()
    if p:
        using_puck = 1
        return p, None

    # when using GPS shield
    ls_p = gps_usb_find_list_of_ports_for_hat()
    if not ls_p:
        return None, None
    using_hat = 1
    port_nmea = ls_p[1]
    port_ctrl = ls_p[-2]
    return port_nmea, port_ctrl



def gps_read(usb_port) -> bytes:
    # usb_port: '/dev/ttyUSB0'
    bb = bytes()
    ser = None

    try:
        if using_puck:
            ser = serial.Serial(usb_port, 4800, timeout=0)
        else:
            ser = serial.Serial(usb_port, baudrate=115200, timeout=0)

        for _ in range(11):
            time.sleep(.1)
            bb += ser.read(ser.in_waiting)

            # basic debug
            # print('bb', bb)

            is_there_rmc = [x for x in bb.split(b'\r\n') if x.startswith(b'$GPRMC') and chr(x[-3]) == '*']
            is_there_gga = [x for x in bb.split(b'\r\n') if x.startswith(b'$GPGGA') and chr(x[-3]) == '*']
            if is_there_rmc or is_there_gga:
                break
    except (Exception,) as ex:
        pm(f'error gps_read -> {ex}')
    finally:
        if ser:
            ser.close()
        return bb



def gps_sentence_parse_time_field(m, b_type: bytes):
    global g_last_ymd
    if b_type == b'$GPRMC':
        g_last_ymd = m.datetime.strftime("%Y-%m-%d")
        return m.datetime.strftime("%Y-%m-%dT%H:%M:%SZ")

    # GPGGA only contains the HMS, not Ymd
    # ts: 14:24:17+00:00
    ts = m.timestamp.strftime("%H:%M:%S")
    if not g_last_ymd:
        g_last_ymd = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    return f'{g_last_ymd}T{ts}Z'



def gps_sentence_parse_whole(bb: bytes, b_type: bytes) -> dict:
    # bb: list of binary GPS sentences
    # b_type: b'$GPRMC' or b'$GPGGA'
    assert type(b_type) is bytes

    ll = bb.split(b'\r\n')
    ll = [i for i in ll if b_type in i]
    ok = False
    lat, lon, dt = '', '', ''
    sentence = ''
    speed = ''

    for line in ll:
        try:
            sentence = line.decode()
            m = pynmea2.parse(sentence, check=False)
            lat = m.latitude
            lon = m.longitude

            # time is slightly different depending on sentence
            dt = gps_sentence_parse_time_field(m, b_type)

            if lat and lon and dt:
                ok = True
                if b_type == b'$GPRMC':
                    speed = sentence.split(',')[7]
                break
        except ChecksumError:
            pm(f'error parse bad checksum on {sentence}')
        except (Exception,) as ex:
            pm(f'error parsing {sentence} -> {ex}')

    if type(lat) is float:
        lat = '{:.4f}'.format(float(lat))

    if type(lon) is float:
        lon = '{:.4f}'.format(float(lon))

    return {
        'ok': ok,
        'type': b_type.decode(),
        'lat': lat,
        'lon': lon,
        'dt': dt,
        'sentence': sentence,
        'speed': speed
    }



def gps_parse_sentence_rmc(bb):
    return gps_sentence_parse_whole(bb, b'$GPRMC')



def gps_parse_sentence_gga(bb):
    return gps_sentence_parse_whole(bb, b'$GPGGA')
