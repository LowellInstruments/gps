import time
import pynmea2
import serial
from pynmea2 import ChecksumError
from serial.tools import list_ports
from datetime import datetime, timezone
from gps.gps_adafruit import gps_adafruit_detect_usb_port, gps_adafruit_read
from gps.gps_puck import gps_puck_detect_usb_port, gps_puck_read
from gps.gps_quectel import gps_hat_detect_list_of_usb_ports, gps_hat_read

p_mod = 'GPS'
g_last_ymd = ''
using_puck = 0
using_hat = 0
using_adafruit = 0
ns_view = 0



def pm(s):
    # stands for print module
    print(f'{p_mod}: {s}', flush=True)



def gps_get_type_of_antenna():
    if using_adafruit:
        return 'external'
    if using_puck:
        return 'external'
    return 'internal'



def gps_get_number_of_satellites():
    return ns_view



def gps_find_any_usb_port():

    # we prefer adafruit, then puck, then hat
    global using_puck
    global using_hat
    global using_adafruit
    using_hat = 0
    using_puck = 0
    using_adafruit = 0

    p = gps_adafruit_detect_usb_port()
    if p:
        using_adafruit = 1
        return p, None, 'adafruit'


    p = gps_puck_detect_usb_port()
    if p:
        using_puck = 1
        return p, None, 'puck'


    ls_p = gps_hat_detect_list_of_usb_ports()
    if not ls_p:
        return None, None, None

    using_hat = 1
    port_nmea = ls_p[1]
    port_ctrl = ls_p[-2]
    return port_nmea, port_ctrl, 'hat'



def gps_hardware_read(usb_port) -> bytes:

    # usb_port: '/dev/ttyUSB0'
    d = dict()
    try:
        if using_adafruit:
            gps_adafruit_read(usb_port, d, show=False)
        elif using_puck:
            gps_puck_read(usb_port, d)
        else:
            gps_hat_read(usb_port, d)

        # is_there_rmc = [x for x in bb.split(b'\r\n') if x.startswith(b'$GPRMC') and
        #                 x.count(b'$') == 1 and chr(x[-3]) == '*']
        # is_there_gga = [x for x in bb.split(b'\r\n') if x.startswith(b'$GPGGA') and
        #                 x.count(b'$') == 1 and chr(x[-3]) == '*']


        if 'ns' in d.keys():
            global ns_view
            ns_view = d['ns']
            # todo: we can externally query ns_view

    except (Exception,) as ex:
        pm(f'error gps_hardware_read -> {ex}')
        time.sleep(1)

    finally:
        return d['bb']



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
    # bb: GPS byte string
    # b_type: b'$GPRMC' or b'$GPGGA'
    assert type(b_type) is bytes

    ll = bb.split(b'\r\n')
    ll = [i for i in ll if b_type in i and chr(i[-3]) == '*' and chr(i[0]) == '$']
    ok = False
    lat, lon, dt = '', '', ''
    sentence = ''
    speed = ''

    for line in ll:
        try:
            # case of b'.3$GPGGA,135850'
            line = line[line.index(b'$'):]
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



def gps_sentence_has_rmc_info(bb):
    # bb: b'$GNRMC,185332.400,A,4136.5965,N,07036.5597,W,0.45,187.23,260825,,,D*6B\r\n'
    d_gp = gps_sentence_parse_whole(bb, b'$GPRMC')
    d_gn = gps_sentence_parse_whole(bb, b'$GNRMC')
    if d_gp and d_gp['ok']:
        return d_gp
    if d_gn and d_gn['ok']:
        return d_gn



def gps_sentence_has_gga_info(bb):
    d_ga = gps_sentence_parse_whole(bb, b'$GPGGA')
    if d_ga and d_ga['ok']:
        return d_ga





# test
if __name__ == '__main__':
    _port_nmea, _, _port_type = gps_find_any_usb_port()
    print(_port_type)
    while 1:
        g = gps_hardware_read(_port_nmea)
        print(g)
