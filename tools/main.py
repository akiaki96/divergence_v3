from serial_receiver import SerialReceiver
from parser import LogParser
from realtime_plot import RealtimePlot
from data_logger import CSVLogger


receiver = SerialReceiver(
    "/dev/tty.usbmodem1103",
    921600
)

FLOAT = 4

while True:
    line = receiver.read_line()

    if line == "BIN_START":
        size_line = receiver.read_line()
        expected_size = int(
            size_line.split(":")[1]
        )
        header_line = receiver.read_line()
        headers = [
            h for h in header_line.split(",")
            if h
        ]
        print(headers)
        parser = LogParser(headers)
        logger = CSVLogger(
            "log.csv",
            headers
        )
        received_size = 0
        while received_size <= expected_size:
            binary = receiver.read_bytes(FLOAT*len(headers))
            if not binary:
                break
            received_size += FLOAT*len(binary)
            rows = parser.parse(binary)
            for row in rows:
                logger.write(row)
        logger.close()

    else:
        if line == "":
            continue
        print(line)
