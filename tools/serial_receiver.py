import serial


class SerialReceiver:

    # in: port(str), baudrate(int) / out: なし（シリアルポートを開く）
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(
            port,
            baudrate,
            timeout=5
        )


    # in: なし / out: 受信した1行(str, 改行なし)
    def read_line(self):
        return self.ser.readline().decode().rstrip("\r\n")


    # in: size(int, 受信バイト数) / out: 受信したバイナリ(bytes, 長さsize)
    def read_bytes(self, size):
        data = b''

        while len(data) < size:
            chunk = self.ser.read(size - len(data))

            if not chunk:
                raise TimeoutError()

            data += chunk

        return data