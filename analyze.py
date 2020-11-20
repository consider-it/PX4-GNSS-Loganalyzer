#!/usr/bin/env python3
"""
MyGalileoDrone - GNSS Log Analyzer

Extraction of GNSS UART communication and analysis of uBlox commands

Author:    Jannik Beyerstedt <beyerstedt@consider-it.de>
Copyright: (c) consider it GmbH, 2020
"""

import argparse
import logging
import os
import sys
from enum import Enum
from dataclasses import dataclass
import numpy
from pyulog.core import ULog


def ulog_data_list_search(data_list: list, topic_name: str) -> ULog.Data:
    """Search for a topic in the ULog data list"""

    selection = [d for d in data_list if d.name == topic_name]
    if len(selection) < 1:
        logger.warning("%s not found in log", topic_name)
        return None

    return selection[0]


if __name__ == "__main__":
    LOG_FORMAT = '%(asctime)s %(levelname)s:%(name)s: %(message)s'
    LOG_DATEFMT = '%Y-%m-%dT%H:%M:%S%z'
    logging.basicConfig(format=LOG_FORMAT,
                        datefmt=LOG_DATEFMT, level=logging.INFO)
    logger = logging.getLogger()

    parser = argparse.ArgumentParser(
        description='PX4 uBlox Communication Analyzer')
    parser.add_argument("-i", "--input_filename", action='store', required=True,
                        help="path to input file (output will be named similar)")
    parser.add_argument("-v", "--verbosity", action='count',
                        help="increase output and logging verbosity")
    args = parser.parse_args()

    if args.verbosity == 2:
        logger.setLevel(logging.DEBUG)
    elif args.verbosity == 1:
        logger.setLevel(logging.INFO)
    else:
        logger.setLevel(logging.WARNING)

    # create output file names
    output_file_prefix = os.path.basename(args.input_filename)
    if output_file_prefix.lower().endswith('.ulg'):
        output_file_prefix = output_file_prefix[:-4]

    gps_out_filename_to = output_file_prefix+'_to_device.dat'
    gps_out_filename_from = output_file_prefix+'_from_device.dat'
    position_out_filename = output_file_prefix+'.csv'

    # OPEN AND PARSE LOG FILE
    msg_filter = ['gps_dump', 'transponder_report',
                  'vehicle_global_position', 'vehicle_gps_position']

    ulog = ULog(args.input_filename, msg_filter, False)
    data = ulog.data_list

    if len(data) == 0:
        logger.error("File %s does not contain the necessary messages!", args.input_filename)
        sys.exit(0)

    if len(ulog.dropouts) > 0:
        logger.warning("File contains %i dropouts", len(ulog.dropouts))

    for d in data:
        logger.info("Found %i messages in %s", len(d.field_data), d.name)

    # GET gps_dump DATA AND CONVERT/ ANALYZE IT
    gpsdump_data = [d for d in data if d.name == 'gps_dump'][0]

    # message format check
    field_names = [f.field_name for f in gpsdump_data.field_data]
    if not 'len' in field_names or not 'data[0]' in field_names:
        logger.error("gps_dump: message has wrong format!")
        sys.exit(-1)

    logger.info("gps_dump: Creating output files %s and %s",
                gps_out_filename_to, gps_out_filename_from)

    # state of ublox protocol parser
    class UbxParserState(Enum):
        IDLE = 1
        START1_FOUND = 2
        START2_FOUND = 3
        MSG_CLASS_FOUND = 4
        MSG_ID_FOUND = 5
        LEN1_FOUND = 6
        LEN2_FOUND = 7
        READING = 20
    ubx_parser_state = UbxParserState.IDLE
    ubx_msg_payload_len = 0

    @dataclass
    class UbxMessage():
        timestamp: numpy.uint64 = None
        msg_class: numpy.uint8 = None
        msg_id: numpy.uint8 = None
        payload: numpy.array = None

        def __repr__(self):
            print_str = 'UbxMessage({0}, {1:#04x}-{2:#04x}:'.format(self.timestamp,
                                                                    self.msg_class, self.msg_id)

            if len(self.payload) > 30:
                print_str += ' {0:d} bytes'.format(len(self.payload))
            else:
                for p in self.payload:
                    print_str += ' {0:02x}'.format(p)

            print_str += ')'
            return print_str

    ubx_current_msg = UbxMessage()
    ubx_msg_payload_cnt = 0

    ubx_messages = []  # this contains the individual ubx messages

    # dump all message to binary file and disect communication
    with open(gps_out_filename_to, 'wb') as to_dev_file:
        with open(gps_out_filename_from, 'wb') as from_dev_file:
            msg_lens = gpsdump_data.data['len']
            for i in range(len(gpsdump_data.data['timestamp'])):
                msg_len = msg_lens[i]
                if msg_len & (1 << 7):
                    msg_len = msg_len & ~(1 << 7)
                    file_handle = to_dev_file
                else:
                    file_handle = from_dev_file

                for k in range(msg_len):
                    data_byte = gpsdump_data.data['data['+str(k)+']'][i]
                    file_handle.write(data_byte)

                    # extract ubx message
                    if ubx_parser_state == UbxParserState.IDLE:
                        if data_byte == 0xb5:
                            ubx_parser_state = UbxParserState.START1_FOUND
                    elif ubx_parser_state == UbxParserState.START1_FOUND:
                        if data_byte == 0x62:
                            ubx_current_msg = UbxMessage()
                            ubx_current_msg.timestamp = gpsdump_data.data['timestamp'][i]
                            ubx_parser_state = UbxParserState.START2_FOUND

                    elif ubx_parser_state == UbxParserState.START2_FOUND:
                        ubx_current_msg.msg_class = data_byte
                        ubx_parser_state = UbxParserState.MSG_CLASS_FOUND
                    elif ubx_parser_state == UbxParserState.MSG_CLASS_FOUND:
                        ubx_current_msg.msg_id = data_byte
                        ubx_parser_state = UbxParserState.MSG_ID_FOUND

                    elif ubx_parser_state == UbxParserState.MSG_ID_FOUND:
                        ubx_msg_payload_len = data_byte
                        ubx_parser_state = UbxParserState.LEN1_FOUND
                    elif ubx_parser_state == UbxParserState.LEN1_FOUND:
                        ubx_msg_payload_len = ubx_msg_payload_len + (data_byte << 8)
                        ubx_current_msg.payload = numpy.zeros(
                            (ubx_msg_payload_len), dtype=numpy.uint8)
                        ubx_parser_state = UbxParserState.LEN2_FOUND

                    elif ubx_parser_state == UbxParserState.LEN2_FOUND:
                        if ubx_msg_payload_cnt < ubx_msg_payload_len:
                            ubx_current_msg.payload[ubx_msg_payload_cnt] = data_byte
                            ubx_msg_payload_cnt += 1

                        if ubx_msg_payload_cnt >= ubx_msg_payload_len:
                            ubx_messages.append(ubx_current_msg)
                            ubx_msg_payload_cnt = 0
                            ubx_parser_state = UbxParserState.IDLE

    # for msg in ubx_messages:
    #     print(msg)  # dev only

    logger.info("gps_dump: Output finished")

    # GET VEHICLE POSITION DATA AND CONVERT/ ANALYZE IT
    CSV_DELIMITER = ';'
    utm_data = ulog_data_list_search(data, 'transponder_report')
    globalpos_data = ulog_data_list_search(data, 'vehicle_global_position')
    gps_data = ulog_data_list_search(data, 'vehicle_gps_position')
    # TODO: get position setpoint

    with open(position_out_filename, 'w') as csvfile:
        # TODO: combine all data vectors
        # utm:
        # globalos: timestamp;lat;lon;alt;alt_ellipsoid;delta_alt;eph;epv;terrain_alt;lat_lon_reset_counter;alt_reset_counter;terrain_alt_valid;dead_reckoning
        # gps: timestamp;time_utc_usec;lat;lon;alt;alt_ellipsoid;s_variance_m_s;c_variance_rad;eph;epv;hdop;vdop;noise_per_ms;jamming_indicator;vel_m_s;vel_n_m_s;vel_e_m_s;vel_d_m_s;cog_rad;timestamp_time_relative;heading;heading_offset;fix_type;vel_ned_valid;satellites_used
        data = gps_data

        # use same field order as in the log, except for the timestamp
        data_keys = [f.field_name for f in data.field_data]
        data_keys.remove('timestamp')
        data_keys.insert(0, 'timestamp')  # we want timestamp at first position

        # write the header
        csvfile.write(CSV_DELIMITER.join(data_keys) + '\n')

        # write the data
        last_elem = len(data_keys)-1
        for i in range(len(data.data['timestamp'])):
            for k, _ in enumerate(data_keys):
                csvfile.write(str(data.data[data_keys[k]][i]))
                if k != last_elem:
                    csvfile.write(CSV_DELIMITER)
            csvfile.write('\n')
