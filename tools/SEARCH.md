# 探索（足立法）と，シミュレータでの再現

探索のソルバーは `external/micromouse_simulator/solver` の adachi を，シミュレータと同じソースのまま使う。
実機は壁を読むたびにログを1行残し，シミュレータはそのログを再生して，実機が何を見てどう判断したかを再現する。

## ファイル

| ファイル | 役割 | 編集 |
|---|---|---|
| `search_presets.json` | 探索のプリセット（速度・加速度・使うターンの集合・超信地旋回・壁の補正） | 手で |
| `gen_search_presets.py` | `config/search_presets.hpp` を生成する。スラロームが設計されていなければビルドを止める | — |
| `Core/Src/app/search.cpp` | 探索のループ（壁を読む → ソルバー → 動作を積む）とログ | — |
| `Core/Src/common/wall_sensor.cpp` | IRセンサーの位置の対応と壁の判定 | 対応がずれていたら |
| `Core/Src/common/wall_control.cpp` | 直進中の横壁による向きの補正 | — |
| `Core/Inc/config/mouse_config.hpp` | `config::wall`（閾値・基準値・ゲイン），`config::search`（壁を読む位置・ゴール・ログの行数・電池） | 調整で |

プリセットの例（`search_presets.json`）：

```json
{
  "500": {"speed": 500, "accel": 3000, "turns": ["S90", "L90", "T180"],
          "pivot": {"omega": 360, "alpha": 2500}, "wall_control": true, "note": ""}
}
```

- `turns` に使うターンの種類を並べ，`speed` でそれぞれのスラロームが決まる（例 `S90` と 500 → `config::slalom::S90_500`）。
  探索中でも既知区間では大回りなどを使うので，複数持てる。種類ごとに `SearchPreset` の集合へ入る：
  - `OrthoTurns`（区画に沿ったターン）… `S90`, `L90`, `T180`
  - `DiagonalTurns`（斜めのターン）… `IN45`, `OUT45`, `V90`, `IN135`, `OUT135`。1つでも並べたときだけ作る
- `S90` は必須（未知区間は区画境界で1歩ずつ進むので，入口・出口とも区画境界の小回り90°で曲がる）。
  並べていない種類は `nullptr` で，使わない
- `slalom_params.json` にその速度の設計がないとビルドが止まる（設計済みの速度が表示される）
- `pivot` は超信地旋回（行き止まりの180°）の最大角速度 [dps] と角加速度 [dps/s]
- `fan` を `true` にするとファンを `config::fan::RUN_DUTY` で回して走り，ターンもファンONの設計（`"500_fan"`，
  `config::slalom::S90_500_FAN`）を使う。省略すると `false`
- `wall_control` は**実験中**の横壁による向きの補正。省略すると `false`
- 壁を読む位置はプリセットによらないので `config::search::READ_LEAD_MM` にある
- プリセットはメニュー `Run` → `Search` に，JSONに書いた順で並ぶ（`config::menu::MAX_CHILDREN` 個まで）

## 走らせるまで

1. **壁センサーの確認**：メニュー `Device` → `IR` → `Wall check`
   - シリアルに機体の位置ごとの値（`L FL FR R`），壁の判定，元の変数の値が出る。LEDバーは左から左・前・右の壁の判定
   - **各センサーを手でふさいで，位置の対応が合っているか確かめる**。変数名（`irL` など）は機体の位置と
     一致しないので，`wall_sensor.cpp` の `pick()` で対応をとっている（ずれていたらここだけ直す）。
     今は 左 = `irR`，前左 = `irFL`，前右 = `irFR`，右 = `irL`。前・横の区別は前壁スイープで実測済みで，
     左右（`irR`/`irL`，`irFL`/`irFR`）は LED の点け方からの推定なので，ここで確かめる
   - 区画境界の `READ_LEAD_MM` 手前（探索で壁を読む位置）に置き，壁があるとき・ないときの値の中間を
     `config::wall::THRESH_*` に，区画の中心線上で両側に壁があるときの左右の値を `REF_*` にする
   - 抜けるときはリセット
2. **探索**：機体の後端をスタート区画 (0,0) の後壁に当てて北へ向け，`Run` → `Search` → プリセットを選ぶ
   - ゴール（`config::search::GOAL_X/Y`，既定 (7,7)）に着くとスタートへ戻る探索を続け，スタート区画の中央で止まる
   - 次のときは止まって，LEDバーが左右交互に点滅する（理由はシリアルに出る）：
     - `wall ahead` … 壁のある向きへ進もうとした（壁の誤読）
     - `segment rejected` … 区間を積めなかった
     - `profile error` … PlanProfileが区間を落とした，または壁を読む前に止まった
     - `too many steps` … ログの行数（`MAX_STEPS`）を超えた
3. **ログの受け取り**：止まったら機体を持ち上げて置く（`haltByAccZ`）と，`tools/main.py` が2つ保存する
   - `tools/log/search/<preset>.csv` … 壁を読むたびに1行（位置・壁・ソルバーの動作・帰り探索中か・IRの値）
   - `tools/log/search/<preset>_trace.csv` … 走行中の目標・実測の位置と角度，壁の補正（50Hz）

## シミュレータで再現する

`external/micromouse_simulator` で（環境の準備はその readme を参照）：

```bash
# ログを再生する。位置・判断がずれた歩があれば一覧を出して終了コード1
python run_headless.py --replay ../../tools/log/search/500.csv

# 走らせた迷路の画像を渡すと，壁の誤読（どの区画のどの壁を，どのIR値で読み違えたか）も出す
python run_headless.py --replay ../../tools/log/search/500.csv --maze-image maze_image/<迷路>.png

# ブラウザで再生する（誤読した壁はオレンジの破線）
python main.py --replay ../../tools/log/search/500.csv --maze-image maze_image/<迷路>.png
```

- 壁の誤読が出たら，その行の `ir_*` と `config::wall::THRESH_*` を見比べて閾値を直す
- 判断の不一致が出たら，実機とシミュレータでソルバーのコードか設定（ゴール・帰り探索）が違う。
  サブモジュールのコミットがそろっているか確かめる
- 一度位置がずれると以降の行はすべて不一致になるので，最初の不一致から見る

## 注意

- **実機では未確認**。閾値・基準値・ゲイン（`config::wall`）は仮の値なので，まず `Wall check` で決める
- 横壁の補正は直進中（目標の角速度が0で，並進が `MIN_VELOCITY` より速い間）だけ効く。
  補正した角度は旋回の後も残る（旋回は相対角度で積むため）
- 区画境界の `READ_LEAD_MM` 手前で壁を読み，その間に次の動作を積む。`READ_LEAD_MM` が短すぎると
  ソルバーの計算が間に合わず `profile error` になる（500mm/sで10mmなら20ms）
- 探索のログは CCMRAM に置いている（スタートアップは CCMRAM を0にしないので，件数だけで有効な範囲を表す）
