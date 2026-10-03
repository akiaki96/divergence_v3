from datetime import datetime

from serial_receiver import SerialReceiver
from parser import LogParser
from realtime_plot import RealtimePlot
from data_logger import CSVLogger
from get_log import Saver
import argparse
import csv
import os
import subprocess
import sys
import tempfile

# ArgumentParserの例を作成
arg_parser = argparse.ArgumentParser()

# オプションの設定
arg_parser.add_argument("--no_gui", action="store_true", help="Do not show graph")
arg_parser.add_argument("--no_save", action="store_true", help="Do not save received data to CSV")
arg_parser.add_argument(
    "--on_conflict",
    choices=["overwrite", "sequence"],
    default="sequence",
    help="同名CSVが既に存在する場合の挙動。sequence: '_1','_2',...の連番を付けて別ファイルとして両方保存する（デフォルト）, "
         "overwrite: 上書き",
)

# オプションの解釈
args = arg_parser.parse_args()

receiver = SerialReceiver(
    "/dev/tty.usbmodem1103",
    921600
)

FLOAT = 4

# グラフは別のプロセスで開く（plt.show() で受信が止まると，その間に届いた後続の表が
# USBシリアルのバッファからあふれて欠け，次の read_line() が表の途中のバイナリを読んでしまう）
PLOT_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plot_log.py")


def plot_in_background(filename):
    subprocess.Popen([sys.executable, PLOT_SCRIPT, filename])


# BIN_START の後の1つの表を受け取る。壊れていれば（行が欠けた・時間切れ）None
def receive_table():
    dirName = receiver.read_line()
    fileName = receiver.read_line()
    timestamp_line = receiver.read_line()
    size_line = receiver.read_line()
    header_line = receiver.read_line()
    if not timestamp_line.startswith("TIMESTAMP:") or not size_line.startswith("SIZE:"):
        print(f"Warning: broken table header ({timestamp_line!r}, {size_line!r}); waiting for the next BIN_START")
        return None
    includeTimestamp = timestamp_line.split(":")[1] == "1"
    expected_size = int(size_line.split(":")[1])
    headers = [h for h in header_line.split(",") if h]
    print(f"{dirName}/{fileName}: {headers}")
    if not headers or expected_size % (FLOAT * len(headers)) != 0:
        print(f"Warning: SIZE {expected_size} does not match {len(headers)} columns; waiting for the next BIN_START")
        return None
    try:
        binary = receiver.read_bytes(expected_size)
        end_raw = receiver.read_line_raw()
    except TimeoutError as e:
        binary, end_raw = e.args[0], b""
        print(f"Warning: timed out after BIN_START ({dirName}/{fileName}, {len(binary)}/{expected_size} bytes)")
    print("Actually received:", len(binary))
    if end_raw.rstrip(b"\r\n") != b"BIN_END":
        # 途中のバイトが欠けると，後ろの BIN_END や次の表まで本体として読んでしまう。この表は捨て，
        # 読みすぎた中に次の表の BIN_START があればそこから読み直す
        blob = binary + end_raw
        i = blob.find(b"BIN_START\r\n")
        if i >= 0:
            receiver.unread(blob[i:])
        print(f"Warning: BIN_END not received; {dirName}/{fileName} not saved (bytes were lost)"
              + ("; resyncing at the next BIN_START" if i >= 0 else ""))
        return None
    return dirName, fileName, includeTimestamp, headers, LogParser(headers).parse(binary)


while True:
    line = receiver.read_line()

    if line == "BIN_START":
        table = receive_table()
        if table is None:
            continue
        dirName, fileName, includeTimestamp, headers, data = table

        if args.no_save:
            print("Not saved (--no_save)")
            if not args.no_gui:
                with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow(headers)
                    writer.writerows([row[h] for h in headers] for row in data)
                plot_in_background(f.name)
        else:
            # saver.save_to_csv(data, headers, dirName, fileName, includeTimestamp) -> filename: str（保存先CSVパス）
            saver = Saver(on_conflict=args.on_conflict)
            filename = saver.save_to_csv(data, headers, dirName, fileName, includeTimestamp)
            if not args.no_gui:
                plot_in_background(filename)

    else:
        if line == "":
            continue
        print(line)
