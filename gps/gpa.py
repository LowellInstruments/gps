import sys
import time
import pynmea2
import serial
from pynmea2 import ChecksumError
from serial.tools import list_ports
from datetime import datetime, timezone



VID_PID_GPS_ADAFRUIT = "10C4:EA60"
p_mod = 'GPS'



def pm(s):
    # stands for print module
    print(f'{p_mod}: {s}', flush=True)



def gps_adafruit_find_usb_port():
    for p, _, vp in sorted(list(list_ports.comports())):
        if VID_PID_GPS_ADAFRUIT in vp:
            # port: /dev/ttyUSB0
            pm(f'found adafruit USB port on {p}')
            return p
    return None




def gps_adafruit_detect_baud_rate(p):
    for i in (9600, 38400, 115200):
        bb = gps_adafruit_read(p, show=False)
        if bb and b'$GP' in bb or b'$GN' in bb:
            return i




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
        pm(f'error gps_adafruit_send_command -> {ex}')

    finally:
        if ser:
            ser.close()
        return rv, bb




def gps_adafruit_set_baud_rate_115200(up):
    return _gps_adafruit_send(up, '$PMTK251,115200*1F', 9600)




def gps_adafruit_del_sentences(up):
    c = '$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
    return _gps_adafruit_send(up, c)




def gps_adafruit_set_refresh_rate(up):
    c = '$PMTK220,200*2C'
    return _gps_adafruit_send(up, c)




def gps_adafruit_set_sentences(up):
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



def gps_adafruit_read(up, show=True) -> bytes:
    # up: '/dev/ttyUSB0'
    bb = bytes()
    ser = None

    try:
        ser = serial.Serial(up, 115200, timeout=0)

        for _ in range(5):
            time.sleep(.1)
            bb += ser.read(ser.in_waiting)

        if show:
            # basic debug
            print('-----------------------')
            for i in bb.split(b'\r\n'):
                print(i)

    except (Exception,) as ex:
        pm(f'error gps_hardware_read -> {ex}')
        time.sleep(1)

    finally:
        if ser:
            ser.close()
        return bb




# test
if __name__ == '__main__':

    port = gps_adafruit_find_usb_port()
    if not port:
        pm('error: did not find port for adafruit')
        sys.exit(1)


    gps_adafruit_set_baud_rate_115200(port)
    gps_adafruit_del_sentences(port)
    gps_adafruit_set_refresh_rate(port)
    gps_adafruit_set_sentences(port)


    while 1:
        gps_adafruit_read(port)
