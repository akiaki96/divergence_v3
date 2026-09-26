# data_analysis2 — システム同定・制御設計の解析

実機ログ（`tools/log/`）を MATLAB / Python で解析し，並進・回転のプラント同定と速度制御器の設計・検証を行った記録。
各フォルダは「作業の段階」ごとに分かれており，番号順が概ね実施順。

## 目次

1. [フォルダ構成](#フォルダ構成)
2. [01 並進：プラント同定](#01-並進プラント同定--01_trans_identification)
3. [02 並進：速度PI+FF制御](#02-並進速度piff制御--02_trans_velocity_control)
4. [03 回転：プラント同定](#03-回転プラント同定--03_rot_identification)
5. [04 回転：角速度PI制御とゲインスケジューリング](#04-回転角速度pi制御とゲインスケジューリング--04_rot_omega_control)
6. [05 吸引ファン電流推定](#05-吸引ファン電流推定--05_fan)
7. [実行方法とフォルダ間の依存](#実行方法とフォルダ間の依存)

## フォルダ構成

```
data_analysis2/
├── README.md                    この目次
├── 01_trans_identification/     並進：ステップ応答同定 → PRBS設計 → PRBS本同定
├── 02_trans_velocity_control/   並進：速度PI+FF制御の実機検証とλ（整定パラメータ）の検討
├── 03_rot_identification/       回転（並進700mm/s固定）：ステップ同定 → 動作点別フィット → PRBS
├── 04_rot_omega_control/        回転：角速度PI・ゲインスケジューリング・レート制限・2自由度FF
└── 05_fan/                      吸引ファン：LiPo電圧降下からの電流推定（Python）
```

各フォルダ内の `results/` に，そのフォルダのスクリプトが出力した図（.png/.fig）と表（.csv）が入る。

---

## 01 並進：プラント同定 — `01_trans_identification/`

| 順 | レポート (.md) | スクリプト (.m) | 内容 |
|---|---|---|---|
| 1 | [step_identification_report.md](01_trans_identification/step_identification_report.md) | [step_analyze.m](01_trans_identification/step_analyze.m) | duty 10/15/20% のステップ応答から1次遅れ(P1)同定，不感帯電圧 u0 |
| 2 | [prbs_design.md](01_trans_identification/prbs_design.md) | [prbs_design.m](01_trans_identification/prbs_design.m) | ステップ同定結果からPRBSのクロック周期・duty範囲を設計（距離制約の事前シミュレーション） |
| 3 | [prbs_identification_report.md](01_trans_identification/prbs_identification_report.md) | [prbs_analyze.m](01_trans_identification/prbs_analyze.m) | PRBS本同定（procest P1 + ETFE，holdout検証）。`mouse_config.hpp` の K_p / T_p1 の出典 |

入力ログ：`tools/log/step_x/`, `tools/log/prbs_trans_x/`

## 02 並進：速度PI+FF制御 — `02_trans_velocity_control/`

| 順 | レポート (.md) | スクリプト (.m) | 内容 |
|---|---|---|---|
| 1 | [velocity_step_verification_report.md](02_trans_velocity_control/velocity_step_verification_report.md) | [velocity_step_analyze.m](02_trans_velocity_control/velocity_step_analyze.m) | 速度ステップ（300/600/900/-600）の追従性・アンチワインドアップ・FF寄与 |
| 2 | [velocity_lambda_sweep_report.md](02_trans_velocity_control/velocity_lambda_sweep_report.md) | [velocity_lambda_sweep.m](02_trans_velocity_control/velocity_lambda_sweep.m) | IMC整定の λ を振った高速化時の安定性（シミュレーション） |
| 3 | [velocity_lambda03_hardware_report.md](02_trans_velocity_control/velocity_lambda03_hardware_report.md) | [velocity_step900_lambda03_check.m](02_trans_velocity_control/velocity_step900_lambda03_check.m) | λ=0.03 の実機検証と，900mm/sステップでの振動のシミュレーション再現 |

入力ログ：`tools/log/velocity_step_x/`

## 03 回転：プラント同定 — `03_rot_identification/`

| 順 | レポート (.md) | スクリプト (.m) | 内容 |
|---|---|---|---|
| 1 | [rot_step_v700_report.md](03_rot_identification/rot_step_v700_report.md) | [rot_step_v700_analyze.m](03_rot_identification/rot_step_v700_analyze.m) | 並進700mm/s下の左右duty差ステップに対するヨーレート応答（事前同定） |
| 2 | 〃 | [rot_step_v700_p1fit.m](03_rot_identification/rot_step_v700_p1fit.m) | 20水準の動作点別 P1D/P2 フィット。**`results/rot_step_v700_p1fit_model_params.csv` は 04 の多くのスクリプトが使う** |
| 3 | [rot_high_amplitude_test_plan.md](03_rot_identification/rot_high_amplitude_test_plan.md) | — | 高振幅域（〜430dps）の追加試験計画 |
| 4 | [prbs_rot_design_report.md](03_rot_identification/prbs_rot_design_report.md) | [prbs_rot_design.m](03_rot_identification/prbs_rot_design.m) | 回転方向PRBSの設計 |
| 5 | [prbs_rot_identification_report.md](03_rot_identification/prbs_rot_identification_report.md) | [prbs_rot_analyze.m](03_rot_identification/prbs_rot_analyze.m) | 回転方向PRBS本同定 |

入力ログ：`tools/log/rot_step_v700_x/`, `tools/log/prbs_rot_v700_x/`

## 04 回転：角速度PI制御とゲインスケジューリング — `04_rot_omega_control/`

中心となる文書は [rot_gain_scheduling_plan.md](04_rot_omega_control/rot_gain_scheduling_plan.md)（§ごとに実験 E1〜E12・ファーム変更 F3〜F6 を記録）。
スクリプトは下表のとおり，同計画書の段階に対応する。

| 段階 | スクリプト (.m) | 内容 |
|---|---|---|
| PI設計（シミュレーション） | [rot_omega_pi_sim.m](04_rot_omega_control/rot_omega_pi_sim.m) | 動作点依存プラント上の閉ループシミュレーション |
| 〃 | [rot_omega_pi_tune_sweep.m](04_rot_omega_control/rot_omega_pi_tune_sweep.m) | Kc一定＋Ti表（倍率）のスイープによるファーム実装案の選定 |
| 実機検証 E1 | [omega_step_analyze.m](04_rot_omega_control/omega_step_analyze.m) | 初期PIの実機ステップ試験とプラント仮説 H1/H2 の検証 |
| 実機検証 E3/E5 | [omega_step_e3_analyze.m](04_rot_omega_control/omega_step_e3_analyze.m) | スケジュール後ファームの実機試験（リップル・OS・再現性） |
| OS原因の調査 | [rot_plant_h4_check.m](04_rot_omega_control/rot_plant_h4_check.m) | プラントモデル修正(H4)とランプ指令の効果予測 |
| 〃 | [rot_local_model_fit.m](04_rot_omega_control/rot_local_model_fit.m) | 閉ループデータから動作点まわりの局所プラントを推定 |
| 〃 | [rot_pi_linear_os_map.m](04_rot_omega_control/rot_pi_linear_os_map.m) | PI(Kc,Ti)×局所プラントの OS 感度マップ |
| F3 レート制限 | [omega_ramp_analyze.m](04_rot_omega_control/omega_ramp_analyze.m) | ステップ vs ランプ指令の実機比較と合否判定 |
| F4 Ti延長 | [rot_f4_ti_sweep.m](04_rot_omega_control/rot_f4_ti_sweep.m) | +側 Ti(+430) を延ばすコストと効果 |
| F5 2自由度FF | [rot_f5_kc_ti_sweep.m](04_rot_omega_control/rot_f5_kc_ti_sweep.m) | +側の Kc・Ti(+430) の組合せ掃引 |
| 〃 | [rot_ff2dof_study.m](04_rot_omega_control/rot_ff2dof_study.m) | FF付き2自由度制御の効果試算 |
| 〃 | [omega_ff_paired_analyze.m](04_rot_omega_control/omega_ff_paired_analyze.m) | FF ON/OFF のペア差・トレンド補正回帰（先に `omega_ramp_analyze.m` を実行） |
| 性能まとめ | [omega_tracking_report.m](04_rot_omega_control/omega_tracking_report.m) → [omega_tracking_report.md](04_rot_omega_control/omega_tracking_report.md) | 現行ファームの目標追従性能と，2000mm/s・大角速度への外挿可否 |

入力ログ：`tools/log/omega_step_v700_x/`, `tools/log/omega_ramp_v700_x/`, `tools/log/omega_ff_v700_x/`（一部 `rot_step_v700_x/`）

## 05 吸引ファン電流推定 — `05_fan/`

| レポート (.md) | スクリプト (.py) | 内容 |
|---|---|---|
| [fan_current_estimation.md](05_fan/fan_current_estimation.md) | [fan_current_estimate.py](05_fan/fan_current_estimate.py) | ファンON/OFF時のLiPo電圧降下からファン電流を推定（手順は .md 参照） |
| 〃 | [fan_current_estimate_selftest.py](05_fan/fan_current_estimate_selftest.py) | 合成データによる推定スクリプトの自己検証 |

入力ログ：`tools/log/fan_vsag_m2/`

---

## 実行方法とフォルダ間の依存

- **MATLABスクリプトは各スクリプトのあるフォルダをカレントにして実行する**（`results/` と `../../tools/log/` を相対パスで参照）。
- Pythonスクリプトは CSV パスを引数で受け取るのでカレントは任意（例：`python data_analysis2/05_fan/fan_current_estimate.py tools/log/fan_vsag_m2/fan_*.csv`）。
- フォルダをまたぐ入力：

| 使う側 | 読むファイル | 作る側 |
|---|---|---|
| 04 の `rot_omega_pi_sim.m`, `rot_omega_pi_tune_sweep.m`, `omega_step_analyze.m`, `omega_step_e3_analyze.m`, `rot_plant_h4_check.m`, `rot_f4_ti_sweep.m`, `rot_f5_kc_ti_sweep.m` | `03_rot_identification/results/rot_step_v700_p1fit_model_params.csv` | 03 `rot_step_v700_p1fit.m` |
| 01 `prbs_design.m` | `results/step_identification_{summary,final}.csv` | 01 `step_analyze.m` |
| 04 `omega_ff_paired_analyze.m` | `results/omega_ramp_summary_by_run.csv` | 04 `omega_ramp_analyze.m` |
