import time
import serial
from serial.tools import list_ports

from gps.gps_utils import gps_utils_parse_number_of_satellites_in_view

VID_PID_GPS_ADAFRUIT = "10C4:EA60"
p_mod = 'GPS'



def pm(s):
    # stands for print module
    print(f'{p_mod}: {s}', flush=True)



def gps_adafruit_detect_usb_port():
    for p, _, vp in sorted(list(list_ports.comports())):
        if VID_PID_GPS_ADAFRUIT in vp:
            # port: /dev/ttyUSB0
            pm(f'found adafruit USB port on {p}')
            return p
    return None



def gps_adafruit_init(up):
    _gps_adafruit_set_baud_rate_115200(up)
    _gps_adafruit_del_sentences(up)
    _gps_adafruit_set_refresh_rate(up)
    _gps_adafruit_set_sentences(up)





def _gps_adafruit_send(up, c, baud_rate=115200):

    # up: '/dev/ttyUSB2'
    bb = bytes()
    ser = None
    rv = False

    try:
        ser = serial.Serial(up, baudrate=baud_rate, timeout=0)
        # c: $PMTK251,115200*1F
        ser.write(f'{c}\r\n'.encode())

    except (Exception,) as ex:
        pm(f'error _gps_adafruit_send -> {ex}')

    finally:
        if ser:
            ser.close()
        return rv, bb




def _gps_adafruit_set_baud_rate_115200(up):
    return _gps_adafruit_send(up, '$PMTK251,115200*1F', 9600)




def _gps_adafruit_del_sentences(up):
    c = '$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
    return _gps_adafruit_send(up, c)




def _gps_adafruit_set_refresh_rate(up):
    c = '$PMTK220,200*2C'
    return _gps_adafruit_send(up, c)




def _gps_adafruit_set_sentences(up):
    # 0: GLL
    # 1: RMC
    # 2: VTG,
    # 3: GGA
    # 4: GSA
    # 5: GSV
    # only RMC
    # c = '$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29'
    # RMC and GSV
    c = '$PMTK314,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28'
    return _gps_adafruit_send(up, c)



def gps_adafruit_read(up, d, show=True) -> dict:

    # up: '/dev/ttyUSB0'
    bb = bytes()
    ser = None
    bb_gsv = []
    bb_rmc = []

    try:
        ser = serial.Serial(up, 115200, timeout=0)

        for _ in range(5):
            time.sleep(.1)
            bb += ser.read(ser.in_waiting)

            # parse things this GPS can do
            bb_gsv = [x for x in bb.split(b'\r\n') if x.startswith(b'$GPGSV') and
                            x.count(b'$') == 1 and chr(x[-3]) == '*']

            bb_rmc = [x for x in bb.split(b'\r\n') if x.startswith(b'$GNRMC') and
                            x.count(b'$') == 1 and chr(x[-3]) == '*']

            if show:
                # basic debug
                for i in bb.split(b'\r\n'):
                    print(i)

            if bb_rmc or bb_gsv:
                break

        if bb_gsv:
            d['ns'] = gps_utils_parse_number_of_satellites_in_view(bb_gsv)



    except (Exception,) as ex:
        pm(f'error gps_adafruit_read -> {ex}')
        time.sleep(1)

    finally:
        # bb: bytes, NOT list of byte strings
        d['bb'] = bb
        if ser:
            ser.close()
        return d
