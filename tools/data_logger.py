import csv


class CSVLogger:


    # in: filename(str), headers(list[str]) / out: なし（CSVファイルを開きヘッダー行を書く）
    def __init__(self, filename, headers):

        self.file=open(
            filename,
            "w",
            newline=""
        )

        self.writer=csv.DictWriter(
            self.file,
            fieldnames=headers
        )

        self.writer.writeheader()


    # in: row(dict, headerをキーとする1行分のデータ) / out: なし（1行追記）
    def write(self,row):

        self.writer.writerow(row)


    # in: なし / out: なし（ファイルを閉じる）
    def close(self):

        self.file.close()