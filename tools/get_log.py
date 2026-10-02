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
    # in: on_conflict(str, 同名CSVが既に存在する場合の挙動のデフォルト。
    #                 save_to_csv()側で個別に上書き指定しない限りこれが使われる。
    #                 "sequence" : "_1", "_2", ... と連番を付けた別ファイルとして保存し，
    #                              既存ファイルを残す（デフォルト。同じ試験を繰り返してもログが消えない）
    #                 "overwrite": 上書きする。詳細はtools/DATA_FORMAT.md §4参照
    def __init__(self, on_conflict="sequence"):
        if on_conflict not in ("overwrite", "sequence"):
            raise ValueError(f"on_conflict must be 'overwrite' or 'sequence' (got {on_conflict!r})")
        self.on_conflict = on_conflict

    # in: data(list[dict], parser.parse()の出力), headers(list[str]),
    #     dirName(str, 保存先ディレクトリ。"sub/dir"のようにネストも可),
    #     fileName(str, 拡張子なしのファイル名。空文字なら"log"を使う。
    #              こちらも"sub/name"のようにネストを含められる),
    #     includeTimestamp(bool, Trueならファイル名末尾に年月日時刻を付与),
    #     on_conflict(str | None, 指定すればこの呼び出しに限り__init__のデフォルトを上書きする。
    #                 値は__init__と同じ"overwrite"/"sequence")
    # out: filename(str, 保存したCSVの絶対パス)
    def save_to_csv(self, data, headers, dirName, fileName="", includeTimestamp=True, on_conflict=None):
        # =========================
        # CSV保存（指定ディレクトリ/ファイル名、必要なら年月日時刻入り）
        # =========================
        base_name = fileName if fileName else "log"
        if includeTimestamp:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            base_name = f"{base_name}_{timestamp}"

        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log", dirName)
        filename = os.path.join(log_dir, f"{base_name}.csv")
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        conflict_mode = on_conflict if on_conflict is not None else self.on_conflict
        if conflict_mode not in ("overwrite", "sequence"):
            raise ValueError(f"on_conflict must be 'overwrite' or 'sequence' (got {conflict_mode!r})")
        if conflict_mode == "sequence" and os.path.exists(filename):
            # 同名ファイルが既に存在する場合，"_1", "_2", ... の連番で空いている名前を探す
            seq = 1
            while True:
                candidate = os.path.join(log_dir, f"{base_name}_{seq}.csv")
                if not os.path.exists(candidate):
                    filename = candidate
                    break
                seq += 1

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(headers)

            for row in data:
                writer.writerow([row[h] for h in headers])

        print("Saved:", filename)
        return filename