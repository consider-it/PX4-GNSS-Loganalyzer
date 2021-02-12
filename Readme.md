# PX4 uBlox GNSS Communication Analysis
Extraction of the uBlox GNSS module communication from a PX4 flight log (uLog format) and analysis of the logged raw data communication.

PX4 has the option (`GPS_DUMP_COMM`) to dump the GPS module communication to the flight log.
The standard log analysis tools do not evaluate this information, so a special solution is needed.


## Installation
```shell
pip3 install -r requirements.txt
```


## Usage
Run the python script with:
```shell
python3 analyze.py -i path/to/logfile_name.ulog
```

Output will be written to:

- logfile_name-from_device.dat: Raw binary UART data received from the ublox module
- logfile_name-to_device.dat: Raw binary UART data sent to the ublox module
- logfile_name-ubx.csv: Parsed ublox NAV-SVINFO messages
- logfile_name.csv: Data from satellite_info uorb messages



## ublox Protocol Reference
UBX-NAV-SAT is needed to get the IDs of the connected satellites, but PX4 does not enable this message by default.
To enable detailed information on the received satellites, the GPS driver must be started with the `-s` flag.
Then information on up to 20 satellites will be publish to uORB topic `satellite_info` and the `GPS_STATUS` MAVLink message.

So add to `etc/extras.txt`:
```
gps stop
gps start -b 115200 -s
```

### Satellite Numbering

- GPS G1-G32: UBX svId 1-32
- Galileo E1-E63: UBX svId 211-246
- SBAS S120-S158: UBX svId 120-158

### Data Types
Mapping of ublox data types to python struct format characters:

- U1: Unsigned Char     --> B
- I1: Signed Char       --> b
- X1: Bitfield          --> B
- U2: Unsigned Short    --> H
- I2: Signed Short      --> h
- X2: Bitfield          --> H
- U4: Unsigned Long     --> L
- I4: Signed Long       --> l
- X4: Bitfield          --> L
- R4: IEEE 754 Single Precision --> f
- R8: IEEE 754 Double Precision --> d
- CH: ASCII Char        --> c

### Messages
Message frame structure:

- `0xB5 0x62`: sync chars
- 1 byte message class
    * `0x01`: NAV - Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
    * `0x02`: RXM - Receiver Manager Messages: Satellite Status, RTC Status
    * `0x04`: INF - Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
    * `0x05`: ACK - Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
    * `0x06`: CFG - Configuration Input Messages: Configure the receiver.
    * `0x09`: UPD - Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
    * `0x0A`: MON - Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
    * `0x0B`: AIM - AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
    * `0x0D`: TIM - Timing Messages: Time Pulse Output, Time Mark Results
    * `0x10`: ESF - External Sensor Fusion Messages: External Sensor Measurements and Status Information
    * `0x13`: MGA - Multiple GNSS Assistance Messages: Assistance data for various GNSS
    * `0x21`: LOG - Logging Messages: Log creation, deletion, info and retrieval
    * `0x27`: SEC - Security Feature Messages
    * `0x28`: HNR - High Rate Navigation Results Messages: High rate time, position, speed, heading
- 1 byte message ID (excerpt)
    * MON:
        + `0x0A 0x32`: MON-BATCH - Polled - Data batching buffer status
        + `0x0A 0x28`: MON-GNSS - Polled - Information message major GNSS...
        + `0x0A 0x0B`: MON-HW2 - Periodic/Polled - Extended hardware status
        + `0x0A 0x09`: **MON-HW** - Periodic/polled - Hardware status
        + `0x0A 0x02`: MON-IO - Periodic/Polled - I/O system status
        + `0x0A 0x06`: MON-MSGPP - Periodic/Polled - Message parse and process status
        + `0x0A 0x27`: MON-PATCH - Poll Request - Poll request for installed patches
        + `0x0A 0x27`: MON-PATCH - Polled - Installed patches
        + `0x0A 0x07`: MON-RXBUF - Periodic/Polled - Receiver buffer status
        + `0x0A 0x21`: MON-RXR - Output - Receiver status information
        + `0x0A 0x2E`: MON-SMGR - Periodic/Polled - Synchronization manager status
        + `0x0A 0x08`: MON-TXBUF - Periodic/Polled - Transmitter buffer status
        + `0x0A 0x04`: MON-VER - Poll Request/ Polled - Poll receiver and software version
    * NAV:
        + `0x01 0x60`: NAV-AOPSTATUS - Periodic/Polled - AssistNow Autonomous status
        + `0x01 0x05`: NAV-ATT - Periodic/Polled - Attitude solution
        + `0x01 0x22`: NAV-CLOCK - Periodic/Polled - Clock solution
        + `0x01 0x31`: NAV-DGPS - Periodic/Polled - DGPS data used for NAV
        + `0x01 0x04`: **NAV-DOP** - Periodic/Polled - Dilution of precision
        + `0x01 0x61`: NAV-EOE - Periodic - End of epoch
        + `0x01 0x39`: NAV-GEOFENCE - Periodic/Polled - Geofencing status
        + `0x01 0x13`: NAV-HPPOSECEF - Periodic/Polled - High precision position solution in ECEF
        + `0x01 0x14`: NAV-HPPOSLLH - Periodic/Polled - High precision geodetic position solution
        + `0x01 0x28`: NAV-NMI - Periodic/Polled - Navigation message cross-check...
        + `0x01 0x09`: NAV-ODO - Periodic/Polled - Odometer solution
        + `0x01 0x34`: NAV-ORB - Periodic/Polled - GNSS orbit database info
        + `0x01 0x01`: NAV-POSECEF - Periodic/Polled - Position solution in ECEF
        + `0x01 0x02`: NAV-POSLLH - Periodic/Polled - Geodetic position solution
        + `0x01 0x07`: **NAV-PVT** - Periodic/Polled - Navigation position velocity time solution
        + `0x01 0x3C`: NAV-RELPOSNED - Periodic/Polled - Relative positioning information in...
        + `0x01 0x10`: NAV-RESETODO - Command - Reset odometer
        + `0x01 0x35`: _NAV-SAT_ - Periodic/Polled Satellite information
        + `0x01 0x32`: NAV-SBAS - Periodic/Polled - SBAS status data
        + `0x01 0x42`: NAV-SLAS - Periodic/Polled - QZSS L1S SLAS status data
        + `0x01 0x06`: NAV-SOL - Periodic/Polled - Navigation solution information
        + `0x01 0x03`: NAV-STATUS - Periodic/Polled - Receiver navigation status
        + `0x01 0x30`: NAV-SVINFO - Periodic/Polled - Space vehicle information
        + `0x01 0x3B`: NAV-SVIN - Periodic/Polled - Survey-in data
        + `0x01 0x24`: NAV-TIMEBDS - Periodic/Polled - BeiDou time solution
        + `0x01 0x25`: NAV-TIMEGAL - Periodic/Polled - Galileo time solution
        + `0x01 0x23`: NAV-TIMEGLO - Periodic/Polled - GLONASS time solution
        + `0x01 0x20`: NAV-TIMEGPS - Periodic/Polled - GPS time solution
        + `0x01 0x26`: NAV-TIMELS - Periodic/Polled - Leap second event information
        + `0x01 0x21`: NAV-TIMEUTC - Periodic/Polled - UTC time solution
        + `0x01 0x11`: NAV-VELECEF - Periodic/Polled - Velocity solution in ECEF
        + `0x01 0x12`: NAV-VELNED - Periodic/Polled - Velocity solution in NED
- 2 byte length field
- payload
- 2 byte checksum




