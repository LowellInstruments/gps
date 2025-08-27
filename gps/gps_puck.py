from serial.tools import list_ports



VID_PID_GPS_PUCK = "067B:23A3"
VID_PID_GPS_PUCK_2 = "067B:2303"



def gps_puck_detect_usb_port():
    for port, _, vp in sorted(list(list_ports.comports())):
        if (VID_PID_GPS_PUCK in vp) or (VID_PID_GPS_PUCK_2 in vp):
            print(f'GPS: puck found on USB port {port}')
            return port
    return None
