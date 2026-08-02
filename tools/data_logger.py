import csv


class CSVLogger:


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


    def write(self,row):

        self.writer.writerow(row)


    def close(self):

        self.file.close()