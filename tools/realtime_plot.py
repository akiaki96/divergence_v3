import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque


class RealtimePlot:


    # in: channel(str, dataの中でプロットするキー名) / out: なし
    def __init__(self, channel):

        self.channel = channel

        self.x = deque(maxlen=500)
        self.y = deque(maxlen=500)

        self.fig, self.ax = plt.subplots()

        self.line, = self.ax.plot([], [])

        self.counter = 0


    # in: data(dict, 1サンプル分。self.channelキーを持つ) / out: なし（内部バッファに追加）
    def update(self, data):

        self.x.append(self.counter)
        self.y.append(data[self.channel])

        self.counter += 1


    # in: frame(FuncAnimationから渡されるフレーム番号、未使用) / out: 更新した線オブジェクトのtuple
    def animate(self, frame):

        self.line.set_data(
            self.x,
            self.y
        )

        self.ax.relim()
        self.ax.autoscale_view()


        return self.line,


    # in: なし / out: なし（アニメーションを開始しグラフウィンドウを表示）
    def start(self):

        FuncAnimation(
            self.fig,
            self.animate,
            interval=50
        )

        plt.show()