import time
import pynmea2
import serial
from pynmea2 import ChecksumError
from serial.tools import list_ports
from datetime import datetime, timezone
from gps.gps_adafruit import gps_adafruit_detect_usb_port



VID_PID_GPS_HAT = "2C7C:0125"
p_mod = 'GPS'
g_last_ymd = ''
ns_view = 0



def pm(s):
    # stands for print module
    print(f'{p_mod}: {s}', flush=True)



def gps_hat_init(usb_port):

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
        pm(f'error gps_hat_init -> {ex}')

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
        pm(f'error gps_hat_get_firmware_version -> {ex}')

    finally:
        if ser:
            ser.close()
        return ans_v, ans_m




def gps_hat_detect_list_of_usb_ports():
    ls = []
    for port, _, vp in sorted(list(list_ports.comports())):
        if not VID_PID_GPS_HAT in vp:
            continue
        ls.append(port)
        # ls: ['/dev/ttyUSB0' ... '/dev/ttyUSB3']
    if ls:
        pm(f'found hat list of USB ports on\n\t{ls}')
    return ls



def gps_hat_read(up, show=True) -> bytes:
    # up: '/dev/ttyUSB0'
    bb = bytes()
    ser = None

    try:
        ser = serial.Serial(up, 115200, timeout=0, dsrdtr=True, rtscts=True)

        for _ in range(5):
            time.sleep(.1)
            bb += ser.read(ser.in_waiting)

        if show:
            # basic debug
            for i in bb.split(b'\r\n'):
                print(i)

    except (Exception,) as ex:
        pm(f'error gps_hat_read -> {ex}')
        time.sleep(1)

    finally:
        if ser:
            ser.close()
        return bb