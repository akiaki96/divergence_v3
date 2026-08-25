import struct


class LogParser:

    # in: headers(list[str]) / out: なし
    def __init__(self, headers):
        self.headers = headers


    # in: binary(bytes, リトルエンディアンfloat32の連続データ)
    # out: list[dict]（1要素=1サンプル、キーはheaders、値はfloat）
    def parse(self, binary):

        count = len(binary)//4

        values = struct.unpack(
            "<" + "f"*count,
            binary
        )

        rows = []

        width = len(self.headers)

        for i in range(0, len(values), width):
            rows.append(
                dict(
                    zip(
                        self.headers,
                        values[i:i+width]
                    )
                )
            )

        return rows