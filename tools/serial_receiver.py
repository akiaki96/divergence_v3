import serial


class SerialReceiver:

    # in: port(str), baudrate(int) / out: なし（シリアルポートを開く）
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(
            port,
            baudrate,
            timeout=5
        )
        # 読みすぎて戻したバイト（unread()）。次の読み出しはここから
        self.pending = b''


    # in: なし / out: 受信した1行(bytes, 改行込み。時間切れなら途中まで)
    def read_line_raw(self):
        i = self.pending.find(b'\n')
        if i >= 0:
            line, self.pending = self.pending[:i + 1], self.pending[i + 1:]
            return line
        line, self.pending = self.pending + self.ser.readline(), b''
        return line


    # in: なし / out: 受信した1行(str, 改行なし)。表の本体（バイナリ）を行として読んでしまったときも
    # 落ちないように，UTF-8 でないバイトは置き換える（呼び出し側は次の BIN_START まで読み飛ばす）
    def read_line(self):
        return self.read_line_raw().decode("utf-8", errors="replace").rstrip("\r\n")


    # in: size(int, 受信バイト数) / out: 受信したバイナリ(bytes, 長さsize)。
    # 時間切れなら TimeoutError（args[0] にそれまでに受け取ったバイト）
    def read_bytes(self, size):
        data, self.pending = self.pending[:size], self.pending[size:]

        while len(data) < size:
            chunk = self.ser.read(size - len(data))

            if not chunk:
                raise TimeoutError(data)

            data += chunk

        return data


    # in: data(bytes) / out: なし。読みすぎたバイトを戻す（次の read_line / read_bytes が先に読む）
    def unread(self, data):
        self.pending = data + self.pending
