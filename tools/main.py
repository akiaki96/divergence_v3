from datetime import datetime

from serial_receiver import SerialReceiver
from parser import LogParser
from realtime_plot import RealtimePlot
from data_logger import CSVLogger
from get_log import Saver


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

        binary = receiver.read_bytes(expected_size)
        print("Actually revieved:", len(binary))

        end_line = receiver.read_line()
        if end_line != "BIN_END":
            print("Warning: BIN_END not received correctly")

        data = parser.parse(binary)
        rows = (expected_size//4) // len(headers)
        
        saver = Saver()
        filename = saver.save_to_csv(data, headers, rows)

    else:
        if line == "":
            continue
        print(line)
