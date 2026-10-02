# スラロームのパラメータと実機での調整

## ファイル

| ファイル | 役割 | 編集 |
|---|---|---|
| `slalom_profile_designer.py` | シミュレータ。「保存」で `slalom_params.json` に書く | — |
| `slalom_params.json` | 設計値（シミュレータの値）。ターンの表示名 → 速度 → パラメータ | デザイナーから |
| `slalom_tuning.json` | 実機での調整。設計値への**差分**だけを書く | 手で |
| `slalom_presets.py` | ターンの種類（表示名・角度・入口/出口の位置）。デザイナーと生成スクリプトで共有 | 種類を増やすとき |
| `gen_slalom_params.py` | 2つのJSONから `config/slalom_params.hpp` を生成する | — |

生成ヘッダは `build/<preset>/generated/config/slalom_params.hpp` にできる（gitには入れない）。
CMakeがビルドのたびに，JSONか生成スクリプトが変わっていれば作り直すので，`scripts/build.sh` /
`scripts/build_flash.sh` / IDEのビルドのどれでも最新のJSONの値が入る。中身の確認は
`python3 tools/gen_slalom_params.py`（標準出力へ出す）。

## 調整の流れ

1. デザイナーで設計して保存する（`slalom_params.json`）
2. `scripts/build_flash.sh` で書き込む
3. メニュー `Slalom` → `Slalom left` / `Slalom right` → 種類（`S90` / `L90` / `T180` …）→ 速度（`500` …）を選んで走らせる
   （種類は `slalom_presets.py` の順，速度は昇順に並ぶ）
   - 置き方：機体の後端を区画の後壁に当て，区画の中心線に沿って前へ向ける
   - 入口が区画境界（小回り90°）ならその区画の前の境界，区画中央（大回り90°・180°）なら1つ先の区画中央で曲がり始め，
     出口の基準点から次の区画中央で止まる
4. 止まった位置・向きのずれと，シリアルに出る結果を見る

   ```
   slalom result S90_500 right: angle -89.62 deg (target -90.0, error +0.38), distance 391.6 mm (target 390.9, error +0.7)
   ```

   距離の目標は区間ごとの端数（1tick未満）を含まないので，誤差が+1mm程度出るのは正常。
   ログは `tools/log/slalom/<name>_<left|right>.csv`
5. `slalom_tuning.json` に差分を書き，2へ戻る

```json
{
  "小回り90°": {
    "500": {
      "base_saved_at": "2026-10-02T03:34:10",
      "delta": { "Set_pri_offset": -2.0, "Set_post_offset": 1.5 },
      "note": "出口で右に1mmずれる。電池8.1V"
    }
  }
}
```

- キーは `slalom_params.json` と同じ（表示名 → 速度）
- `base_saved_at` は調整したときの設計値の `saved_at`。デザイナーで保存し直すと一致しなくなり，**ビルドがエラーで止まる**
  （古い差分を新しい設計値に足さないため）。差分がまだ有効なら新しい `saved_at` に書き換える
- `delta` に使えるキーは `Set_low_AngVel` / `Set_Low_AngAcl` / `Set_pri_offset` / `Set_post_offset`（速度は表のキーなので変えられない）

## 横滑りの係数（c, K）の同定

`tools/identify_slip.py` が，走行ログと定規で測った停止位置のずれから c, K を求める（デザイナーでの初期値はいらない）。

1. スラロームを走らせ，止まった位置の区画中央からのずれを測る：出口方向に直交する方向（ターンの外側＝入口と反対側が正）と，
   出口方向（目標より先が正）
2. `tools/slip_measurements.json` に1行足す（ログのファイル名から種類・速度・ファン・向きを読む）

   ```json
   {"log": "slalom/L90_500_right.csv", "outward": 10, "longitudinal": -8, "note": "ファンOFF，BACK_TO_AXLE_MM=42"}
   ```

   ログは同じ名前で上書きされるので，`tools/main.py --on_conflict sequence` で残し，連番の付いた名前を書く。
   `BACK_TO_AXLE_MM` を変える前の走行には `"back_to_axle_mm": 25` のように走ったときの値を書く
3. `python3 tools/identify_slip.py` を実行する。ファンON/OFFごとに，走行ごとの c_eff，c だけ（K=0）の値，
   2つ以上の速度があれば c と K，各パラメータの pre/post_offset の目安が出る
4. デザイナーに c, K を入れ，目安を参考に pre/post_offset を設計し直して保存する

1つの速度からは c_eff = c + K·v² しか決まらない。K を求めるには速度を大きく変えた走行（例 300 と 1000mm/s）を足す。

## 走らない（LEDバーが左右交互に点滅する）とき

シリアルに理由が出る。

- `rejected (accelLimit)` … 角加速度が `config::profile_limit::MAX_ALPHA` を超えている
- `rejected (tooShort)` … 最大角速度まで加速しきれない（生成スクリプトでも止まる）
- `diagonal entry/exit` … 斜めのターンは試験が未対応

## 注意

- `config::mouse::BACK_TO_AXLE_MM`（`Core/Inc/config/mouse_config.hpp`，機体の後端から車軸まで，42mm）がずれていると
  入口の位置がずれ，その分を `pre_offset` の調整で吸収してしまう。メニュー `Slalom` → `Axle check` → `n=1/2/4/8` で確かめる：
  車軸（車輪の中心）を床の線の真上に置くと，`BACK_TO_AXLE_MM + 90·n` 走って後端が90·n先の線の上で止まるはず。
  線より先に止まった量をδとすると，本当の値は `BACK_TO_AXLE_MM − δ`（δがnに比例して増えるならエンコーダの距離の誤差）
- デザイナーの滑り係数は軌道の予測だけに使うもので，実機には渡さない（機体幅 `Set_Width` も同じ）。
  スリップ角（速度の向きが機体の向きより外側へ遅れる角）は β = K·v·ω + c·ω/v（v [m/s]，ω [rad/s]，c [m]）：
  - `Set_K_SP`（K）… 横加速度 v·ω に比例する滑り（タイヤの横方向の弾性）
  - `Set_C_SP`（c [mm]）… 速度によらない横滑り（車軸の横すべり速度 c·ω。車軸より c 前の点を中心に回るのと同じ）
  - 2026-10-02 の実機（ファンOFF，右旋回5本）は c ≈ 10.5mm，K = 0 でずれをRMS 2mmで再現した。K だけで合わせると
    500mm/s で約0.04，700mm/s で約0.02 と速度で変わった
- 滑り係数はファンの有無で変わるので，デザイナーの「ファン: ON/OFF」を走る条件に合わせて保存する。
  ファンONは速度のキーに `_fan` が付き（`"500_fan"`），同じターン・速度でもファンOFF（`"500"`）と別々に持つ。
  ボタンで切り替えると，その条件の保存データがあれば読み込む。`slalom_tuning.json` のキーも同じ（`"500_fan"`）。
  生成ヘッダでは `L90_500_FAN`（メニューの速度は `500 fan`）になり，試験はファンを `config::fan::RUN_DUTY`（20%）で回して走る
- メニューに並べられるのは種類・1種類あたりの速度とも `config::menu::MAX_CHILDREN`（10）個まで。超えるとビルドが止まる
