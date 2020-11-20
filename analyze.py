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
                    file_handle.write(gpsdump_data.data['data['+str(k)+']'][i])

    logger.info("gps_dump: Output finished")

    # GET VEHICLE POSITION DATA AND CONVERT/ ANALYZE IT
    CSV_DELIMITER = ';'
    utm_data = ulog_data_list_search(data, 'transponder_report')
    globalpos_data = ulog_data_list_search(data, 'vehicle_global_position')
    gps_data = ulog_data_list_search(data, 'vehicle_gps_position')
    # TODO: get position setpoint

    with open(position_out_filename, 'w') as csvfile:
        # TODO: combine all data vectors
        data = globalpos_data

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
