from serial_receiver import SerialReceiver
from parser import LogParser
from realtime_plot import RealtimePlot
from data_logger import CSVLogger


receiver = SerialReceiver(
    "/dev/tty.usbmodem1103",
    921600
)


while (line := receiver.read_line()) != "BIN_START":
# while True:
    # line = receiver.read_line()
    if line == "":
        continue
    print(line)


size_line = receiver.read_line()

size = int(
    size_line.split(":")[1]
)


header_line = receiver.read_line()

headers = [
    h for h in header_line.split(",")
    if h
]

print(headers)

parser = LogParser(headers)


# plot = RealtimePlot(
#     "velocity"
# )


logger = CSVLogger(
    "log.csv",
    headers
)


while True:

    binary = receiver.read_bytes(4*len(headers))


    rows = parser.parse(binary)


    for row in rows:

        logger.write(row)

        # plot.update(row)