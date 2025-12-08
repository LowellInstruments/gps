import pynmea2
from pynmea2 import ChecksumError
from datetime import datetime, timezone
from .gps_adafruit import gps_adafruit_detect_usb_port
from .gps_puck import gps_puck_detect_usb_port
from .gps_quectel import gps_hat_detect_list_of_usb_ports
import time
import serial



p_mod = 'GPS'
g_last_ymd = ''



def pm(s):
    # stands for print module
    print(f'{p_mod}: {s}', flush=True)



def gps_find_any_usb_port():

    # prefer 1) adafruit or 2) puck or 3) hat
    p = gps_adafruit_detect_usb_port()
    if p:
        return p, None, 'adafruit'


    p = gps_puck_detect_usb_port()
    if p:
        return p, None, 'puck'


    ls_p = gps_hat_detect_list_of_usb_ports()
    if not ls_p:
        return None, None, None

    port_nmea = ls_p[1]
    port_ctrl = ls_p[-2]
    print(f'GPS: hat using USB ports {ls_p}')
    print(f'\tnmea {port_nmea} ctrl {port_ctrl}')
    return port_nmea, port_ctrl, 'hat'



def _gps_parse_time(m, b_type: bytes):

    # m: pynmea2 message type
    global g_last_ymd
    if b_type in (b'$GPRMC', b'$GNRMC'):
        g_last_ymd = m.datetime.strftime("%Y-%m-%d")
        return m.datetime.strftime("%Y-%m-%dT%H:%M:%SZ")

    # GPGGA only contains the HMS, not Ymd
    # ts: 14:24:17+00:00
    ts = m.timestamp.strftime("%H:%M:%S")
    if not g_last_ymd:
        g_last_ymd = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    return f'{g_last_ymd}T{ts}Z'



def _gps_parse_satellites_in_view(ls_bb):

    ns = 0
    for line in ls_bb:
        try:
            # ls_bb: list of binary $GPGSV sentences
            # line: b'$GPGSV,3,3,11,27,09,318,20,48,13,249,,20,02,107,*4A'
            sentence = line.decode()
            ns = sentence.split(',')[3]
        except (Exception,) as ex:
            print(f'error gps_parse_number_of_satellites_in_view -> {ex}')

    return ns




def _gps_contain_sentence_type(bb: bytes, s_type):
    # s_type: b'$GPGSV'
    assert type(s_type) is bytes
    return [x for x in bb.split(b'\r\n') if x.startswith(s_type) and
            x.count(b'$') == 1 and chr(x[-3]) == '*']



def gps_hardware_read(up, baud_rate, d: dict, debug=True):

    # up: '/dev/ttyUSB0'
    ser = None
    bb = bytes()

    try:
        ser = serial.Serial(up, baud_rate, timeout=0)

        for i in range(2):
            time.sleep(.5)
            bb = ser.read(ser.in_waiting)
            bb_gsv = _gps_contain_sentence_type(bb, b'$GPGSV')
            bb_gga = _gps_contain_sentence_type(bb, b'$GPGGA')
            bb_rmc = _gps_contain_sentence_type(bb, b'$GPRMC')
            bb_rmc += _gps_contain_sentence_type(bb, b'$GNRMC')

            if debug:
                for _ in bb.split(b'\r\n'):
                    print(_)

            if bb_gsv:
                d['ns'] = _gps_parse_satellites_in_view(bb_gsv)

            if bb_rmc or bb_gsv or bb_gga:
                break

    except (Exception,) as ex:
        print(f'GPS: error gps_read -> {ex}')
        time.sleep(1)
        d['error_gps'] = 1

    finally:
        # bb: bytes
        d['bb'] = bb
        if ser:
            ser.close()




def _gps_parse_sentence_type(bb: bytes, b_type: bytes) -> dict:
    # bb: GPS byte string
    # b_type: b'$GPRMC', b'$GPGGA'
    assert type(b_type) is bytes

    ll = bb.split(b'\r\n')
    ll = [i for i in ll if b_type in i and chr(i[-3]) == '*'
        and i.count(b'$') == 1
        and chr(i[0]) == '$'
    ]
    ok = False
    lat, lon, dt = '', '', ''
    sentence = ''
    speed = ''

    for line in ll:
        try:
            # case of b'.3$GPGGA,135850'
            line = line[line.index(b'$'):]
            sentence = line.decode()


            # --------------------------
            # parse with pynmea2 module
            # --------------------------
            m = pynmea2.parse(sentence, check=False)
            lat = m.latitude
            lon = m.longitude


            # time slightly different depending on sentence
            dt = _gps_parse_time(m, b_type)


            if lat and lon and dt:
                ok = True
                if b_type in (b'$GPRMC', b'$GNRMC'):
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

    d = {
        'ok': ok,
        'type': b_type.decode(),
        'lat': lat,
        'lon': lon,
        'dt': dt,
        'sentence': sentence,
        'speed': speed
    }

    return d




def gps_parse_sentence_type_rmc(bb):
    # bb: b'$GNRMC,185332.400,A,4136.5965,N,07036.5597,W,0.45,187.23,260825,,,D*6B\r\n'
    d_gp = _gps_parse_sentence_type(bb, b'$GPRMC')
    d_gn = _gps_parse_sentence_type(bb, b'$GNRMC')
    if d_gp and d_gp['ok']:
        return d_gp
    if d_gn and d_gn['ok']:
        return d_gn
    return None



def gps_parse_sentence_type_gga(bb):
    d_ga = _gps_parse_sentence_type(bb, b'$GPGGA')
    if d_ga and d_ga['ok']:
        return d_ga
    return None
