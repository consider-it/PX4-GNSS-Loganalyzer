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

- logfile_name-TODO.dat
- logfile_name.csv


**Attention**: At the time of writing, PX4 does not write the dumped GPS communication to the log file by default (see https://github.com/PX4/PX4-Autopilot/issues/16229).
The only workaround is, to use a config file for the logger in `etc/logging/logger_topics.txt` on the FMU's SD card.

It has to contain all topics, which should be logged, including:
```
gps_dump 0 0
transponder_report 0 0
```
