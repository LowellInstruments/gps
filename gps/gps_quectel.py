import time
import serial
from serial.tools import list_ports



VID_PID_GPS_HAT = "2C7C:0125"



def _gps_hat_flush(ser):
    if ser:
        ser.read(ser.in_waiting)


def gps_hat_init(usb_port):
    # usb_port: '/dev/ttyUSB2'
    ser = None
    rv = 0

    try:
        # todo: test this dummy read
        ser = serial.Serial(usb_port, baudrate=115200, timeout=0)
        _gps_hat_flush(ser)
        for i in range(3):
            # probably echo activated so will receive back this
            ser.write(b'AT+QGPS=1\r')
            time.sleep(.1)
            bb = ser.read(ser.in_waiting)

            # basic GPS debug
            # print('bb', bb)

            if b'OK' in bb:
                rv = 1
            if b'AT+QGPS=1' in bb:
                # this is the echo
                rv = 1
            if b'CME ERROR: 504' in bb:
                # means was already on
                rv = 1
            if rv:
                break

    except (Exception,) as ex:
        print(f'GPS: error gps_hat_init -> {ex}')

    finally:
        if ser:
            ser.close()
        return rv



def gps_hat_get_firmware_version(port_ctrl):
    # usb_port: '/dev/ttyUSB2'
    ser = None
    ans_v = bytes()
    ans_m = bytes()

    try:
        # todo: test this dummy read
        ser = serial.Serial(port_ctrl, baudrate=115200, timeout=0)
        _gps_hat_flush(ser)
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
        print(f'GPS: error gps_hat_get_firmware_version -> {ex}')

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
    return ls

