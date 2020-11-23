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
import struct
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


@dataclass
class UbxMsg():
    """
    Ublox Message Container

    Attributes:
    - timestamp: from uLog message
    - msg_class: ublox message class
    - msg_id: ublox message id (inside class)
    - raw_payload: array of payload bytes (without checksum)
    - payload: parsed ublox message
    """
    timestamp: numpy.uint64 = None
    msg_class: numpy.uint8 = None
    msg_id: numpy.uint8 = None
    raw_payload: numpy.array = None

    payload = None

    def __repr__(self):
        print_str = 'UbxMsg({0}, {1:#04x}-{2:#04x}:'.format(self.timestamp,
                                                            self.msg_class, self.msg_id)

        if len(self.raw_payload) > 30:
            print_str += ' {0:d} bytes'.format(len(self.raw_payload))
        else:
            for p in self.raw_payload:
                print_str += ' {0:02x}'.format(p)

        print_str += ')'
        return print_str


@dataclass
class UbxNavDop():
    """
    u-blox Protocol UBX-NAV-DOP Message

    - time_of_week in ms
    - Dilution of Precision (Geometric / Position / Time / Vertical / Horizontal / Northing / Easting)
        DOP is Dimensionless / scaled by factor 100
    """

    def __init__(self, payload):
        try:
            self.iTOW, self.gDOP, self.pDOP, self.tDOP, self.vDOP, self.hDOP, self.nDOP, self.eDOP = struct.unpack(
                '=L7H', payload)

            self.gDOP = self.gDOP / 100
            self.pDOP = self.pDOP / 100
            self.tDOP = self.tDOP / 100
            self.vDOP = self.vDOP / 100
            self.hDOP = self.hDOP / 100
            self.nDOP = self.nDOP / 100
            self.eDOP = self.eDOP / 100

        except struct.error:
            logger.error("UbxNavDop parse error %s, %s", sys.exc_info()[0], sys.exc_info()[1])

    def __repr__(self):
        format_str = 'UbxNavDop(iTOW={0}, gDOP={1}, pDOP={2}, tDOP={3}, vDOP={4}, hDOP={5}, nDOP={6}, eDOP={7})'

        return format_str.format(
            self.iTOW, self.gDOP, self.pDOP, self.tDOP, self.vDOP, self.hDOP, self.nDOP, self.eDOP)


@dataclass
class UbxNavPvt():
    """
    u-blox protocol UBX-NAV-PVT

    - Time of week in ms
    - Year(UTC), Month (UTC 1..12), Day (1..31)
    - Hour (0..23), Minute (0..59), Seconds (0..60)
    - Validity Flags
    - Time Accuracy Estimate in ns
    - Fraction of second (-1e9..1e9)
    - GNSS Fix Type / Fix Status Flags / Other Flags
    - Number of Sattelites
    - Longitude in deg (1e-7), Latitude in deg (1e-7), Height Ellipsoid in mm, Height Sea Level in mm
    - Horizontal Accuracy in mm / Vertical Accuracy in mm
    - North Velocity mm s^-1 / East Velocity mm s^-1 / Down Velocity mm s^-1
    - Ground speed in mm s^-1
    - Heading of Motion in deg (1e-5)
    - Speed accuracy mm s^-1, Heading accuracy estimate deg (1e-5)
    - Position DOP (0.01)
    - Heading of Vehicle in deg (1e-5), Magnetic Declination in deg (1e-2), Magnetic Declination Accuracy in deg (1e-2)
    """

    def __init__(self, payload):
        try:
            self.iTOW, self.year, self.month, self.day, self.hour, self.minute, self.second,\
                self.valid, self.tAcc, self.nano, self.fixType, self.flags, self.flags2,\
                self.numSV, self.lon, self.lat, self.height, self.hMSL, self.hAcc, self.vAcc,\
                self.velN, self.velE, self.velD, self.gSpeed, self.headMot,\
                self.sAcc, self.headAcc, self.pDOP, _, _, _, _, _, _, self.headVeh, self.magDec,\
                self.magAcc = struct.unpack('=LH5BBLlB2BB4l2L5lLLH6BlhH', payload)

            self.lon = self.lon / 1e7
            self.lat = self.lat / 1e7
            self.height = self.height / 1000
            self.hMSL = self.hMSL / 1000
            self.hAcc = self.hAcc / 1000
            self.vAcc = self.vAcc / 1000
            self.velN = self.velN / 1000
            self.velE = self.velE / 1000
            self.velD = self.velD / 1000
            self.gSpeed = self.gSpeed / 1000
            self.headMot = self.headMot / 1e5
            self.sAcc = self.sAcc / 1000
            self.headAcc = self.headAcc / 1e5
            self.pDOP = self.pDOP / 100
            self.headVeh = self.headVeh / 1e5
            self.magDec = self.magDec / 1e2
            self.magAcc = self.magAcc / 1e2

        except struct.error:
            logger.error("UbxNavPvt parse error %s, %s", sys.exc_info()[0], sys.exc_info()[1])

    def __repr__(self):
        format_str = 'UbxNavPvt(iTOW={0}, {1}-{2}-{3}T{4}-{5}-{6}, fixType={7}, numSV={8}' + \
            ', lon={9}, lat={10}, height={11}, hMSL={12}, hAcc={13}, vAcc={14}' + \
            ', velN={15}, velE={16}, velD={17}, gSpeed={18}, headMot={19}, sAcc={20}, headAcc={21}' + \
            ', pDOP={22}, headVeh={23}, magDec={24}, magAcc={25})'

        return format_str.format(self.iTOW, self.year, self.month, self.day, self.hour, self.minute, self.second, self.fixType, self.numSV,
                                 self.lon, self.lat, self.height, self.hMSL, self.hAcc,
                                 self.vAcc, self.velN, self.velE, self.velD, self.gSpeed, self.headMot, self.sAcc, self.headAcc,
                                 self.pDOP, self.headVeh, self.magDec, self.magAcc)


@dataclass
class UbxNavSat():
    """
    u-blox protocol UBX-NAV-SAT

    - Time of week in ms
    - Message version (0x01 for this version)
    - Number of satellites
    - N Blocks with:
        - GNSS identifier (see Satellite Numbering) for assignment
        - Satellite identifier (see Satellite Numbering) for assignment
        - Carrier to noise ratio (signal strength)
        - Elevation (range: +/-90), unknown if out of range
        - Azimuth (range 0-360), unknown if elevation is out of range
        - Pseudorange residual in metres
        - Bitmask (see graphic below)
    """

    @dataclass
    class SatInfo():
        """Repeated block in UBX-NAV-SAT message"""
        gnssId: numpy.uint8 = None
        svId: numpy.uint8 = None
        cno: numpy.uint8 = None
        elev: numpy.int8 = None
        azim: numpy.int16 = None
        prRes: numpy.int16 = None
        flags: numpy.uint32 = None

    def __init__(self, payload):
        try:
            payload_hdr = payload[0:8]
            self.iTOW, self.version, self.numSvs, _, _ = struct.unpack('=LBB2B', payload_hdr)

            self.satInfos = []
            for n in range(self.numSvs):
                playload_block = payload[(8+12*n):(21+12*n)]
                print("sliced block from %i to %i", 8+12*n, 21+12*n)  # TODO: dev only

                gnssId, svId, cno, elev, azim, prRes, flags = struct.unpack('=BBBbhhL', playload_block)
                prRes = prRes / 10

                self.satInfos.append(self.SatInfo(gnssId, svId, cno, elev, azim, prRes, flags))

        except struct.error:
            logger.error("UbxNavPvt parse error %s, %s", sys.exc_info()[0], sys.exc_info()[1])

    def __repr__(self):
        repr_str = 'UbxNavSat(iTOW={0}, version={1}, numSvs={2}'.format(self.iTOW, self.version, self.numSvs)

        for elem in self.satInfos:
            format_str = "\tgnssId={0}, svId={1}, cno={2}, elev={3}, azim={4}, prRes={5}\n"
            format_str.format(elem.gnssId, elem.svId, elem.cno, elem.elev, elem.azim, elem.prRes)

        repr_str += "\n)"
        return repr_str


@dataclass
class UbxMonHw():
    """
    u-blox Protocol UBX-MON-HW Message

    - Mask of pins set as peripheral/PIO
    - Mask of pins set as bank A/B
    - Mask of pins set as input/output
    - Mask of pins value low/high
    - Noise level as measured by the GPS core
    - AGC monitor(counts SIGHI xor SIGLO, range 0 to 8191)
    - Status of the antenna supervisor state machine(0=INIT, 1=DONTKNOW, 2=OK, 3=SHORT, 4=OPEN)
    - Current power status of antenna(0=OFF, 1=ON, 2=DONTKNOW)
    - Flags(see graphic below)
    - Reserved 17 Bytes
    - Mask of pins that are used by the virtual pin manager
    - Array of pin mappings for each of the 17 physical pins
    - CW jamming indicator, scaled(0=no CW jamming, 255=strong CW jamming)
    - Reserved 2 Bytes
    - Mask of pins value using the PIO Irq
    - Mask of pins value using the PIO pull high resistor
    - Mask of pins value using the PIO pull low resistor
    """

    def __init__(self, payload):
        try:
            self.pinSel, self.pinBank, self.pinDir, self.pinVal, self.noisePerMS, self.agcCnt, \
                self.aStatus, self.aPower, self.flags, _, _, _, _, _, _, _, _, _, _, _, _, _, _,\
                _, _, _, self.usedMask, self.VP, self.jamInd, _, _, self.pinIrq, self.pullH,\
                self.pullL = struct.unpack('=4L2H2BBBL17BB2B3L', payload)

        except struct.error:
            logger.error("UbxMonHw parse error %s, %s", sys.exc_info()[0], sys.exc_info()[1])


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
        """ublox Protocol Parser State Machine States"""
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

    ubx_current_msg = UbxMsg()
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
                            ubx_current_msg = UbxMsg()
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
                        ubx_current_msg.raw_payload = numpy.zeros(
                            (ubx_msg_payload_len), dtype=numpy.uint8)
                        ubx_parser_state = UbxParserState.LEN2_FOUND

                    elif ubx_parser_state == UbxParserState.LEN2_FOUND:
                        if ubx_msg_payload_cnt < ubx_msg_payload_len:
                            ubx_current_msg.raw_payload[ubx_msg_payload_cnt] = data_byte
                            ubx_msg_payload_cnt += 1

                        if ubx_msg_payload_cnt >= ubx_msg_payload_len:
                            ubx_messages.append(ubx_current_msg)
                            ubx_msg_payload_cnt = 0
                            ubx_parser_state = UbxParserState.IDLE

    logger.info("gps_dump: Output finished")

    # PARSE UBLOX PROTOCOL
    for msg in ubx_messages:

        if msg.msg_class == 0x01:  # NAV
            if msg.msg_id == 0x04:  # DOP
                msg.payload = UbxNavDop(msg.raw_payload)
            elif msg.msg_id == 0x07:  # PVT
                msg.payload = UbxNavPvt(msg.raw_payload)
            elif msg.msg_id == 0x35:  # SAT
                msg.payload = UbxNavSat(msg.raw_payload)

            else:
                logger.warning("U-Blox: NAV message ID 0x%02x not implemented", msg.msg_id)

        elif msg.msg_class == 0x0A:  # MON
            if msg.msg_id == 0x09:  # HW
                msg.payload = UbxMonHw(msg.raw_payload)

            else:
                logger.warning("U-Blox: MON message ID 0x%02x not implemented", msg.msg_id)

        else:
            logger.warning("U-Blox: Message Class 0x%02x, ID 0x%02x not implemented", msg.msg_class, msg.msg_id)

    # for msg in ubx_messages:
    #     print(msg.payload)
    # TODO!!!: write parsed data to CSV

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
