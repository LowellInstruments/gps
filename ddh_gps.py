from rd_ctt.ddh import *
from gps import (
    gps_usb_ports_find_any,
    gps_hat_get_firmware_version,
    gps_hat_activate_nmea_output,
    gps_get_type_of_antenna,
    gps_read, gps_parse_sentence_rmc,
    gps_parse_sentence_gga
)
import redis
import json



r = redis.Redis(host='localhost', port=6379)
p_mod = 'GPS'



def pm(s):
    # stands for print module
    print(f'{p_mod}: {s}', flush=True)



def _gps_write_dict_to_redis(d: dict):
    if d['lat'] and d['lon'] and d['dt']:
        # j: { "ok": false,
        #      "type": "$GPGGA",
        #      "lat": "0.0000",
        #      "lon": "0.0000",
        #      "dt": "2025-07-29T18:40:28Z",
        #      "sentence": "$GPGGA,184028.163,,,,,0,00,,,M,0.0,M,,0000*55"
        # }
        j = json.dumps(d)
        r.set(RD_DDH_GPS_FIX, j)
        r.expire(RD_DDH_GPS_FIX, 2)
    else:
        pm(f"_send_dict_to_redis discarded sentence {d['sentence']}")



def _gps_write_speed_to_redis(d: dict):
    s = d['speed']
    r.set(RD_DDH_GPS_SPEED, s)
    r.expire(RD_DDH_GPS_SPEED, 2)



def _gps_redis_init():
    r.delete(RD_DDH_GPS_TH_QUIT)
    r.delete(RD_DDH_GPS_ANTENNA)




def main_gps(ignore_gui=False):

    # do process preparation
    pm(f'process start')
    _gps_redis_init()


    # find GPS ports
    port_nmea, port_ctrl = gps_usb_ports_find_any()
    pm(f'using NMEA port {port_nmea}')
    r.set(RD_DDH_GPS_ANTENNA, gps_get_type_of_antenna())


    # additional steps when using GPS shield
    if port_ctrl:
        gfv, gfm = gps_hat_get_firmware_version(port_ctrl)
        gfv = gfv.replace(b'AT+CVERSION\r', b'').decode()
        pm(f'hat firmware version is {gfv}')
        r.set(RD_DDH_GPS_HAT_GFV, gfv)

        pm(f'activating hat\'s NMEA output on port {port_nmea} by write to {port_ctrl}')
        rv, bb = gps_hat_activate_nmea_output(port_ctrl)
        if rv:
            pm('OK activate hat NMEA stream')
            r.set(RD_DDH_GPS_ANTENNA, 'internal')
        else:
            pm('error activate hat NMEA stream ')
            pm(f'bb is {bb}')


    # GPS infinite loop
    while 1:

        # check we were requested to quit
        if r.exists(RD_DDH_GPS_TH_QUIT):
            pm('detected thread quit flag')
            break

        # check GUI is alive
        if not ignore_gui and not r.exists(RD_DDH_GUI_ALIVE):
            pm('detected GUI killed')
            break

        # read USB port and parse NMEA sentences
        g = gps_read(port_nmea)
        if not g:
            continue
        d_rmc = gps_parse_sentence_rmc(g)
        d_gga = gps_parse_sentence_gga(g)
        dg = d_rmc
        if not d_rmc['ok']:
            dg = d_gga

        # write results to redis
        _gps_write_dict_to_redis(dg)
        _gps_write_speed_to_redis(d_rmc)



def main_gps_ignore_gui():
    main_gps(ignore_gui=True)




if __name__ == '__main__':
    main_gps(ignore_gui=True)
