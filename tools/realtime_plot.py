import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque


class RealtimePlot:


    def __init__(self, channel):

        self.channel = channel

        self.x = deque(maxlen=500)
        self.y = deque(maxlen=500)

        self.fig, self.ax = plt.subplots()

        self.line, = self.ax.plot([], [])

        self.counter = 0


    def update(self, data):

        self.x.append(self.counter)
        self.y.append(data[self.channel])

        self.counter += 1


    def animate(self, frame):

        self.line.set_data(
            self.x,
            self.y
        )

        self.ax.relim()
        self.ax.autoscale_view()


        return self.line,


    def start(self):

        FuncAnimation(
            self.fig,
            self.animate,
            interval=50
        )

        plt.show()