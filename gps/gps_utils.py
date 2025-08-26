
def gps_utils_parse_number_of_satellites_in_view(bb):
    # bb: list of binary $GPGSV sentences

    for line in bb:
        try:
            # line: b'$GPGSV,3,3,11,27,09,318,20,48,13,249,,20,02,107,*4A'
            sentence = line.decode()
            ns_view = sentence.split(',')[3]
            return ns_view
        except (Exception,) as ex:
            print(f'error gps_utils_parse_number_of_satellites_in_view -> {ex}')