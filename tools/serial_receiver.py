import serial


class SerialReceiver:

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(
            port,
            baudrate,
            timeout=5
        )


    def read_line(self):
        return self.ser.readline().decode().rstrip("\r\n")


    def read_bytes(self, size):
        data = b''

        while len(data) < size:
            chunk = self.ser.read(size - len(data))

            if not chunk:
                raise TimeoutError()

            data += chunk

        return data