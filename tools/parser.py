import struct


class LogParser:

    def __init__(self, headers):
        self.headers = headers


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