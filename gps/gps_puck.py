import time
import pynmea2
import serial
from pynmea2 import ChecksumError
from serial.tools import list_ports
from datetime import datetime, timezone
from gps.gps_adafruit import gps_adafruit_detect_usb_port



VID_PID_GPS_PUCK = "067B:23A3"
VID_PID_GPS_PUCK_2 = "067B:2303"
p_mod = 'GPS'
g_last_ymd = ''
ns_view = 0



def pm(s):
    # stands for print module
    print(f'{p_mod}: {s}', flush=True)



def gps_puck_detect_usb_port():
    for port, _, vp in sorted(list(list_ports.comports())):
        if (VID_PID_GPS_PUCK in vp) or (VID_PID_GPS_PUCK_2 in vp):
            # port: /dev/ttyUSB0
            pm(f'found puck USB port on {port}')
            return port
    return None



def gps_puck_read(up, show=True) -> bytes:
    # up: '/dev/ttyUSB0'
    bb = bytes()
    ser = None

    try:
        ser = serial.Serial(up, 9600, timeout=0, dsrdtr=True, rtscts=True)

        for _ in range(5):
            time.sleep(.1)
            bb += ser.read(ser.in_waiting)

        if show:
            # basic debug
            for i in bb.split(b'\r\n'):
                print(i)

    except (Exception,) as ex:
        pm(f'error gps_puck_read -> {ex}')
        time.sleep(1)

    finally:
        if ser:
            ser.close()
        return bb
