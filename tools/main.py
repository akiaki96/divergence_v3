from datetime import datetime

from serial_receiver import SerialReceiver
from parser import LogParser
from realtime_plot import RealtimePlot
from data_logger import CSVLogger
from get_log import Saver
from plot_log import plot_csv

import argparse

# ArgumentParserの例を作成
arg_parser = argparse.ArgumentParser()

# オプションの設定
arg_parser.add_argument("--no_gui", action="store_true", help="Do not show graph")

# オプションの解釈
args = arg_parser.parse_args()

receiver = SerialReceiver(
    "/dev/tty.usbmodem1103",
    921600
)

FLOAT = 4

while True:
    line = receiver.read_line()

    if line == "BIN_START":
        # receiver.read_line() -> "SIZE:<バイト数>"(str)
        size_line = receiver.read_line()
        expected_size = int(
            size_line.split(":")[1]
        )
        # receiver.read_line() -> "h1,h2,...\r\n"(str) -> headers: list[str]
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

        # receiver.read_bytes(expected_size:int) -> binary: bytes(長さexpected_size)
        binary = receiver.read_bytes(expected_size)
        print("Actually revieved:", len(binary))

        end_line = receiver.read_line()
        if end_line != "BIN_END":
            print("Warning: BIN_END not received correctly")

        # parser.parse(binary:bytes) -> data: list[dict]（1要素=1サンプル）
        data = parser.parse(binary)

        # saver.save_to_csv(data, headers) -> filename: str（保存先CSVパス）
        saver = Saver()
        filename = saver.save_to_csv(data, headers)

        # plot_csv(filename:str) -> なし（グラフウィンドウを表示）
        if not args.no_gui:
            plot_csv(filename)

    else:
        if line == "":
            continue
        print(line)
