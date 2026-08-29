# ログ受信〜保存のデータ形式

`main.py` がMCUからシリアル経由でログを受信し、CSVに保存するまでのデータ形式をまとめる。

## 1. シリアル通信プロトコル（MCU → PC）

`main.py` は1行ずつテキストを読み、`"BIN_START"` を受信したらバイナリログ転送シーケンスに入る。

```
"BIN_START"\r\n
<dirName>\r\n
"SIZE:<バイト数>"\r\n
"<header1>,<header2>,...,<headerN>,"\r\n
<バイナリデータ本体>  (SIZEバイト、ヘッダー行の直後から連続送信)
"BIN_END"\r\n
```

- `SIZE:<バイト数>` … バイナリ本体の総バイト数（`expected_size`）。float32(4byte)の並びなので `expected_size` は4の倍数。
- ヘッダー行 … カンマ区切りのフィールド名。末尾のカンマなど空文字は除去される（`parser.py` の `headers`、CSVの列名になる）。
- `BIN_END` が来なかった場合は警告を出すのみで処理は継続する。

`BIN_START` 以外の行はログとしてそのまま標準出力に表示される（`print(line)`）。

## 2. バイナリ本体のレイアウト

- リトルエンディアンの `float32` を `expected_size // 4` 個連続で並べたもの。
- ヘッダーの個数を `width = len(headers)` とすると、`width` 個ごとに1サンプル（1行）を構成する。

```
[s0_f0, s0_f1, ..., s0_f(width-1),  s1_f0, s1_f1, ..., s1_f(width-1),  ...]
```

サンプル数 `rows = (expected_size // 4) // width`。

## 3. パース後のPython表現（`LogParser.parse()` の出力）

`list[dict]`。1要素 = 1サンプル。キーはヘッダー名、値は `float`。

```python
[
    {"Global_time": 10.671682357788086, "left_encoder_velocity": 0.0,
     "right_encoder_velocity": 0.0, "battery": 1955.828, "Left Duty": 0.0, "Right Duty": 0.0},
    {"Global_time": 10.672682762145996, "left_encoder_velocity": 0.0, ...},
    ...
]
```

この `list[dict]` が `data` として `Saver.save_to_csv(data, headers)` に渡される。

## 4. 保存されるCSVファイル（`Saver.save_to_csv()` の出力）

- 保存先: `tools/log/log_<YYYYMMDD_HHMMSS>.csv`
- 1行目: ヘッダー（`headers` の順）
- 2行目以降: 各サンプルを `headers` の順で並べた**数値**（辞書ではない）

```csv
Global_time,left_encoder_velocity,right_encoder_velocity,battery,Left Duty,Right Duty
10.671682357788086,0.0,0.0,1955.8289794921875,0.0,0.0
10.672682762145996,0.0,0.0,1957.74609375,0.0,0.0
...
```

このCSVがそのまま `plot_csv(filename)`（`plot_log.py`）で読み込まれ、1列目（`headers[0]`）をX軸、それ以外の列をチェックボックスでON/OFFできる折れ線グラフとして表示される。

## データの流れまとめ

```
シリアル(テキスト行 + バイナリ)
  └─ SerialReceiver.read_line() / read_bytes()  → str / bytes
       └─ LogParser.parse(binary)                → list[dict]（1行=1サンプル、値はfloat）
            └─ Saver.save_to_csv(data, headers)   → str（保存したCSVファイルパス）
                 └─ plot_csv(filename)             → グラフ表示（戻り値なし）
```
