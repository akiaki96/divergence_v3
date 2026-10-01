# ログ受信〜保存のデータ形式

`main.py` がMCUからシリアル経由でログを受信し、CSVに保存するまでのデータ形式をまとめる。

## 1. シリアル通信プロトコル（MCU → PC）

`main.py` は1行ずつテキストを読み、`"BIN_START"` を受信したらバイナリログ転送シーケンスに入る。

```
"BIN_START"\r\n
<dirName>\r\n
<fileName>\r\n
"TIMESTAMP:<0 or 1>"\r\n
"SIZE:<バイト数>"\r\n
"<header1>,<header2>,...,<headerN>,"\r\n
<バイナリデータ本体>  (SIZEバイト、ヘッダー行の直後から連続送信)
"BIN_END"\r\n
```

- `dirName` … 保存先ディレクトリ（`tools/log/` からの相対パス）。`Logger::dirName`（`Logger::setDirName()`）で設定する。`"sub/dir"` のようにネストしたパスも指定できる。
- `fileName` … 保存するファイル名（拡張子なし）。`Logger::fileName`（`Logger::setFileName()`）で設定する。未指定（空文字）の場合はPC側で `"log"` が使われる。こちらも `"sub/name"` のようにネストを含められる。
- `TIMESTAMP:<0 or 1>` … `Logger::includeTimestamp`（`Logger::setIncludeTimestamp()`）の値。`1` ならPC側の保存ファイル名末尾に `_<YYYYMMDD_HHMMSS>` を付与し、`0` なら付与しない（同名ファイルは上書きされる）。
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

この `list[dict]` が `data` として `Saver.save_to_csv(data, headers, dirName, fileName, includeTimestamp, on_conflict)` に渡される。

## 4. 保存されるCSVファイル（`Saver.save_to_csv()` の出力）

- 保存先: `tools/log/<dirName>/<fileName または"log"><, includeTimestampがTrueなら "_<YYYYMMDD_HHMMSS>"を付与>.csv`
  - 例: `dirName="prbs_trans_t01"`, `fileName=""`, `includeTimestamp=True` → `tools/log/prbs_trans_t01/log_20260918_120000.csv`
  - 例: `dirName="prbs_trans_t01"`, `fileName="run1"`, `includeTimestamp=False` → `tools/log/prbs_trans_t01/run1.csv`
- 1行目: ヘッダー（`headers` の順）
- 2行目以降: 各サンプルを `headers` の順で並べた**数値**（辞書ではない）

### 4.1 同名ファイルが既に存在する場合の挙動（`on_conflict`）

`includeTimestamp=False` の場合（あるいはTrueでも同一秒内に複数回保存した場合），保存先パスが
既存ファイルと衝突することがある。`Saver` はこれを `on_conflict` で制御する：

| `on_conflict` | 挙動 |
|---|---|
| `"overwrite"`（デフォルト） | 既存ファイルをそのまま上書きする（従来の挙動） |
| `"sequence"` | 既存ファイルがあれば，ベース名に `_1`, `_2`, ... と連番を付けた**空いている名前**を探して別ファイルとして保存する（既存ファイルは残る）。例: `run1.csv` が既に存在する場合 → `run1_1.csv`（それも存在すれば `run1_2.csv`, ...） |

指定方法は2通り：

- コンストラクタ: `Saver(on_conflict="sequence")` … 以降そのインスタンスでの `save_to_csv()` 呼び出し全てに適用されるデフォルト
- 呼び出しごと: `saver.save_to_csv(..., on_conflict="sequence")` … その1回の呼び出しに限りコンストラクタの指定を上書きする

`main.py` では起動時オプション `--on_conflict {overwrite,sequence}`（デフォルト `overwrite`）で指定する：

```
python main.py --on_conflict sequence
```

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
