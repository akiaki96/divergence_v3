# IRセンサーの値 → 壁までの距離の換算

ブランチ `feature/wall-distance`。スラロームの停止位置を壁センサーで自動で測る（`tools/SLALOM.md` の「次の作業」4）ための準備。

## 前のセンサーの校正（Device → IR → Front sweep）

1. 機体の後端を区画の後壁に当てて置く（スラロームの試験と同じ置き方）。前の壁を
   - `Front sweep 1 cell` … 1区画先の境界に置く（車軸から壁の面まで 126mm）
   - `Front sweep 2 cells` … 2区画先の境界に置く（306mm。間の区画には壁を置かない）
2. 機体は前の壁へ 40mm/s で近づき（車軸から壁まで `FRONT_TO_AXLE_MM + 5` = 65mm，前端と壁のすき間 約5mm），
   1秒止まって，同じ速さで元の位置へ戻る。ログは `tools/log/ir_sweep/front_<n>cell.csv`（既定で連番で残る）
3. `python3 tools/fit_ir.py` を実行する（`tools/log/ir_sweep/front_*.csv` をすべて使う）。結果は `tools/ir_calibration.json`

正解の距離はエンコーダから作る：車軸から前の壁の面まで `180·n − 12 − BACK_TO_AXLE_MM − 進んだ距離`。
`BACK_TO_AXLE_MM`（42）と `FRONT_TO_AXLE_MM`（60，概算）は `Core/Inc/config/mouse_config.hpp`。

### fit_ir.py がすること
- 4つの変数（`ir_var_L/FL/FR/R`）のうち距離で値が変わるものを前のセンサーと判断する。
  変数名と読み込んでいるピンは食い違っている可能性がある（`feature/search` の `wall_sensor.hpp`：
  irFL ← SENSOR_L，irR ← SENSOR_FL，irL ← SENSOR_FR，irFR ← SENSOR_R）。どの変数が前のセンサーかがデータで確かめられる
- 値が最大になる距離（ピーク）を探す。それより近いと発光・受光素子の位置のずれで値が下がり，同じ値が2つの距離に
  対応するので，ピークより遠い側だけを使う。ピークのすぐ外もまだ下がる効果が残るので，近い端の残差が1mmに
  収まるまで近い側を削る。遠い側は1サンプルの分解能が3mmより悪い所を外す
- モデル `d = A·v^p`（power）と `d = a/√(v − b) + c`（inv_sqrt）を当てはめ，残差の小さい方を選ぶ
- 距離の帯ごとの残差と分解能，近づくとき・離れるときの差を出す（`--table` で10mmごとの値も）

## 次の作業
1. 実機で `Front sweep 1 cell` と `2 cells` を走らせ，`fit_ir.py` の結果を見る（どの変数が前のセンサーか，ピークの距離，
   帯ごとの精度）。電池の電圧を変えて走らせ，換算が電圧で変わるかも見る
2. 横のセンサーの校正：横には走れないので，区画の中でその場旋回し，ジャイロの角度と幾何から壁までの距離を出す試験を作る
3. `ir_calibration.json` から換算のヘッダを生成する（スラロームと同じく JSON → 生成スクリプト → ヘッダ）。
   有効範囲の外（ピークより近い・遠すぎる）は無効として扱う
4. スラロームの試験で，止まった後の壁までの距離から外側・前後・向きを測り，`identify_slip.py` が読むようにする
   （`feature/slalom` と，センサーの対応を確かめた `feature/search` を合わせてから）
