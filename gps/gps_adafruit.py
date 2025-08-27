import serial
from serial.tools import list_ports



VID_PID_GPS_ADAFRUIT = "10C4:EA60"



def gps_adafruit_detect_usb_port():
    for p, _, vp in sorted(list(list_ports.comports())):
        if VID_PID_GPS_ADAFRUIT in vp:
            print(f'GPS: found adafruit USB port on {p}')
            return p
    return None



def gps_adafruit_init(up):
    _gps_adafruit_set_baud_rate_115200(up)
    _gps_adafruit_del_sentences(up)
    _gps_adafruit_set_refresh_rate(up)
    _gps_adafruit_set_sentences(up)





def _gps_adafruit_send(up, c, baud_rate=115200):

    # up: '/dev/ttyUSB2'
    ser = None
    rv = 0

    try:
        ser = serial.Serial(up, baudrate=baud_rate, timeout=0)
        # c: $PMTK251,115200*1F
        ser.write(f'{c}\r\n'.encode())

    except (Exception,) as ex:
        print(f'GPS: error _gps_adafruit_send -> {ex}')
        rv = 1

    finally:
        if ser:
            ser.close()
        return rv




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
