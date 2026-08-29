import os
import serial
import struct
import datetime
import csv
from plot_log import plot_csv

# # COM, port rateを編集!!!
# ser = serial.Serial("/dev/tty.usbmodem1103", 921600, timeout=5)

# # =========================
# # BIN_START待ち
# # =========================
# while True:
#     line = ser.readline().decode().rstrip("\r\n")
#     # line = ser.readline().decode().strip()
#     if not line:
#         continue
#     print("MCU:", line)
#     if line == "BIN_START":
#         break

# # =========================
# # SIZE取得
# # =========================
# size_line = ser.readline().decode().strip()
# print("SIZE LINE:", size_line)

# size = int(size_line.split(":")[1])
# print("Expected size:", size)

# # =========================
# # ヘッダー取得
# # =========================
# header_line = ser.readline().decode().strip()
# # headers = header_line.split(",")
# headers = [h for h in header_line.split(",") if h != ""]
# print("Headers:", headers)

# # =========================
# # バイナリ受信（確実にsize分読む）
# # =========================
# binary = b''
# while len(binary) < size:
#     chunk = ser.read(size - len(binary))
#     if not chunk:
#         print("Timeout! Communication stopped.")
#         break
#     binary += chunk

# print("Actually received:", len(binary))

# if len(binary) != size:
#     print("ERROR: Incomplete binary data")
#     exit()

# # =========================
# # BIN_END確認
# # =========================
# end_line = ser.readline().decode().strip()
# print("END:", end_line)

# if end_line != "BIN_END":
#     print("Warning: BIN_END not received correctly")

# # =========================
# # float変換
# # =========================
# float_count = size // 4
# data = struct.unpack("<" + "f" * float_count, binary)

# rows = float_count // len(headers)

# # =========================
# # CSV保存（年月日時刻入り）
# # =========================
# timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
# filename = f"log/log_{timestamp}.csv"

# with open(filename, "w", newline="") as f:
#     writer = csv.writer(f)
#     writer.writerow(headers)

#     for i in range(rows):
#         start = i * len(headers)
#         end = start + len(headers)
#         writer.writerow(data[start:end])

# print("Saved:", filename)

# plot_csv(filename)


class Saver:
    def __init__(self):
        pass

    # in: data(list[dict], parser.parse()の出力), headers(list[str])
    # out: filename(str, 保存したCSVの絶対パス)
    def save_to_csv(self, data, headers, dirName):
        # =========================
        # CSV保存（年月日時刻入り）
        # =========================
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log", dirName)
        filename = os.path.join(log_dir, f"log_{timestamp}.csv")
        os.makedirs(log_dir, exist_ok=True)
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(headers)

            for row in data:
                writer.writerow([row[h] for h in headers])

        print("Saved:", filename)
        return filename