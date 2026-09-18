# 並進ステップ応答同定レポート（duty 10% / 15% / 20%）

`tools/log/step_x/step_0_{10,15,20}.csv` の一定duty（同相）ステップ応答ログを用いて，
左右モーター実効入力電圧 $u = duty \times V_{batt}$ から並進速度 $v$（左右エンコーダ速度平均）までの
1次遅れモデル（P1）を同定し，さらに3点の定常値からアフィンモデル $v_{ss} = a(u-u_0)$ を推定した結果。

- 解析コード: [`step_analyze.m`](./step_analyze.m)
- 使用ツール: MATLAB R2026a, System Identification Toolbox (`procest`)
- 生データ: `tools/log/step_x/`（gitignore対象のため本リポジトリには含まれない。列: `Global_time, left_encoder_velocity, right_encoder_velocity, battery, Left_Duty, Right_Duty, gyro_z, accel_x`）
- 生成物: `results/`（本レポート内で参照する図・CSV一式）

---

## 1. 各duty水準のP1（1次遅れ）モデル同定結果

| duty | $K_p$ [mm/s/V] | 標準誤差 | $T_{p1}$ [s] | 標準誤差 | $u_{step}$ [V] | $v_{ss}=K_p u_{step}$ [mm/s] | フィット率 |
|---|---|---|---|---|---|---|---|
| 10% | 1401.6 | ±3.4 | 0.4212 | ±0.0023 | 0.802 | 1124.1 | 93.88% |
| 15% | 1483.4 | ±3.3 | 0.3930 | ±0.0021 | 1.187 | 1760.6 | 93.89% |
| 20% | 1536.6 | ±3.9 | 0.3988 | ±0.0024 | 1.571 | 2414.4 | 93.12% |

$u_{step}$は各試行でduty立ち上がり後0.3〜1.0秒区間の実効電圧平均。

生データ・フィット結果:

- ![duty10%](./results/step_duty10_fit.png)
- ![duty15%](./results/step_duty15_fit.png)
- ![duty20%](./results/step_duty20_fit.png)

（編集可能な `.fig` 版も `results/step_duty{10,15,20}_fit.fig` に保存）

$K_p$がduty水準間で約10%異なっており（1401.6→1536.6），不感帯（静止摩擦）の存在が疑われる。
$T_{p1}$はduty間でほぼ一定（0.39〜0.42s）であり，1次遅れ構造がduty水準に依らず妥当であることを支持する。

---

## 2. 不感帯の定量評価

3点 $(u_{step,i},\ v_{ss,i})$ に対しアフィンモデル $v_{ss}=a(u-u_0)$ を最小二乗直線フィット（`polyfit`）：

![v_ss vs u_step](./results/v_ss_vs_u_step.png)

**モンテカルロ法による不確かさの伝播**（各duty水準の$K_p$の共分散を用い，3点の直線フィットを$N=10^5$回繰り返し）：

| 量 | 値 |
|---|---|
| 真の速度ゲイン $a$ | 1677.3 ± 8.7 mm/s/V |
| 不感帯電圧 $u_0$ | 0.1336 ± 0.0047 V （95%区間: [0.1242, 0.1428]） |

分布:

- ![aの推定分布](./results/a_histogram.png)
- ![u0の推定分布](./results/u0_histogram.png)

$u_0$の95%区間が明確に0を含まないため，**不感帯は統計的に有意**。
`data_analysis/translational_system_identification_report.md`（duty 10%/20%の2点，$u_0\approx0.141V$）と近い値であり，同一機体の不感帯特性として整合的。

---

## 3. 数値データ（再利用用）

- [`results/step_identification_summary.csv`](./results/step_identification_summary.csv) — 各duty水準の$K_p$, $T_{p1}$, フィット率, $u_{step}$, $v_{ss}$
- [`results/step_identification_final.csv`](./results/step_identification_final.csv) — $a$, $u_0$（点推定・モンテカルロ平均・標準偏差・95%区間）

---

## 4. まとめ

- duty 10%/15%/20%のステップ応答はいずれもP1（1次遅れ）モデルで93%以上のフィット率を達成
- 定常速度は実効入力電圧に対しほぼ線形（$a\approx1677$ mm/s/V）
- 不感帯電圧 $u_0\approx0.134V$ は統計的に有意だが，運用duty範囲を不感帯から十分離すことで運用上は無視可能と判断（`data_analysis`の既存結論と同方針）
