import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons
import csv


# in: filename(str, save_to_csv()が保存したCSVパス) / out: なし（グラフウィンドウを表示）
def plot_csv(filename):
    with open(filename, newline="") as f:
        reader = csv.reader(f)
        headers = next(reader)
        rows = list(reader)

    # 列ごとに変換
    data = list(zip(*rows))
    data = [[float(x) for x in col] for col in data]

    x = data[0]  # time列

    fig, ax = plt.subplots()
    plt.subplots_adjust(left=0.2)  # チェックボックス分スペース確保

    lines = []
    visibility = []

    # 各列を描画（最初は全部ON）
    for i in range(1, len(headers)):
        line, = ax.plot(x, data[i], label=headers[i])
        lines.append(line)
        visibility.append(True)

    ax.set_xlabel(headers[0])
    ax.legend()
    ax.grid(True)

    # ===== チェックボックス作成 =====
    rax = plt.axes([0.05, 0.4, 0.15, 0.4])
    check = CheckButtons(rax, headers[1:], visibility)

    def toggle(label):
        index = headers[1:].index(label)
        lines[index].set_visible(not lines[index].get_visible())
        plt.draw()

    check.on_clicked(toggle)

    plt.show()

if __name__ == "__main__":
    filename = input("Enter CSV filename to plot: ")
    plot_csv(filename)