#pragma once

#include <cstdint>

namespace config::mouse {
inline constexpr float ENCODER_RES = 4096.f;
inline constexpr float GEAR_RATIO = 13.f / 42.f;
inline constexpr float WHEEL_RADIUS_MM = 23.6f / 2;
inline constexpr float TREAD_MM = 60.f;
} // namespace config::mouse

namespace config::control {
inline constexpr float DT_S = 0.001f;
}

namespace config::imu {
inline constexpr uint16_t REFFERENCE_NUM = 1000; // 何回の平均をもってジャイロのリファレンス電圧とするか

// ジャイロ関連マクロ
inline constexpr float GYRO_Z_SIGN = 1.f;
inline constexpr float GYRO_SENSITIVITY = 16.4f;

// 加速度計関連マクロ
inline constexpr float ACCEL_X_SIGN = 1.f;
inline constexpr float ACCEL_SENSITIVITY = 4096.f;

inline constexpr float G = 9.80665f * 1000.f; // mm/s^2
} // namespace config::imu

namespace config::battery {
inline constexpr float IIR_ALPHA = 0.1f;
}

namespace config::motor {
inline constexpr float MAX_DUTY = 0.95;
inline constexpr uint16_t MAX_PWM = 1000;
inline constexpr float kVbattMinSafe = 5.0f;
}


// 並進速度 PI + feedforward 制御（translational_gain_tuning.md）
//
// プラントモデル: G_V(s) = K_p / (T_p1*s + 1)　（PIゲイン設計用の局所モデル）
// 不感帯込みアフィンモデル: v = A_GAIN * (u - U0_DEADZONE * sign(v))　（feedforward用）
//
// K_p / T_p1 は data_analysis2/prbs_identification_report.md のPRBS本同定結果
// （t01〜t08統合, holdoutフィット94.2%）を採用。
// A_GAIN / U0_DEADZONE は data_analysis2/step_identification_report.md
// （duty10/15/20%のアフィンモデル）に基づく値のまま（PRBSでは不感帯を跨ぐ多点同定は未実施）。
namespace config::pid_velocity_x {
// --- プラントモデル ---
inline constexpr float K_p  = 1518.9f;  // [mm/s/V] PRBS本同定(procest P1)
inline constexpr float T_p1 = 0.4451f;  // [s]      同上
inline constexpr float A_GAIN        = 1738.0f;  // [mm/s/V] アフィンモデルの真の速度ゲイン a（step応答由来）
inline constexpr float U0_DEADZONE   = 0.1360f;  // [V]      不感帯電圧 u0（step応答由来）

// --- PIフィードバックゲイン（IMC/λ整定, §3.1）---
// [要調整] λは未実験。実機でオーバーシュート/整定時間を見ながら調整する（目安 T_p1/2〜2*T_p1）
inline constexpr float LAMBDA = 0.03f;  // [s] 閉ループ時定数
inline constexpr float Kc = T_p1 / (K_p * LAMBDA);
inline constexpr float kp = Kc;
inline constexpr float ki = Kc / T_p1;
inline constexpr float kd = 0.0f;

// --- アンチワインドアップ back-calculation（§3.2）---
// Tt初期値はKp/Ki(=T_p1)を目安とする。飽和からの復帰が遅い/速すぎる場合はここを調整
inline constexpr float BACK_CALC_TT = T_p1;

// --- 出力飽和 ---
inline constexpr float voltage_limit_ratio = 0.95f;

// --- feedforwardのゼロ速度judgement（停止指令時に不感帯補償を入れずビビリを防ぐ）---
inline constexpr float ZERO_VELOCITY_EPS = 1.0f;  // [mm/s]
}

// 回転方向のstep応答事前同定（system_identification_flow.md §2 [2]）
// 並進速度を閉ループでTRANSLATION_VELOCITY_MM_Sに固定した状態で，左右duty差
// （DUTY_DIFF_1〜5, R-L）をステップ印加し非線形性・IMU飽和有無を確認する。
//
// duty_diff0.02/0.04/0.06実測結果（data_analysis2/rot_step_v700_report.md）：
//  - 定常ヨーレートは60dps止まり（IMU飽和±2000dpsの3%）で大きく余裕あり
//  - duty水準が大きいほどゲインも大きくなる非線形性を確認（707→804 dps/duty）
// 上記を踏まえ0.10, 0.14を追加。並進700mm/s時の基準duty実測値は約0.0714で，
// DUTY_DIFF_5=0.14は左右差の半分(0.07)が基準dutyとほぼ一致する上限
// （これを超えると片輪のduty符号が反転し，動作レジームが大きく変わる）。
namespace config::rot_step_v700 {
inline constexpr float TRANSLATION_VELOCITY_MM_S = 700.f;
inline constexpr uint32_t ACCEL_MS = 800;   // 並進速度700mm/sを閉ループで立ち上げる時間
inline constexpr uint32_t TEST_MS  = 600;   // duty差を保持する時間（励振時間，通常水準）
// duty_diff>=0.22付近の高振幅域では600msで整定しない（+0.28で確認済み）ため，
// 真の整定値・時定数を確認する試験専用に励振時間を延長する
// （data_analysis2/rot_high_amplitude_test_plan.md 試験1）
inline constexpr uint32_t TEST_MS_LONG = 2000;

inline constexpr float DUTY_DIFF_1 = 0.02f;
inline constexpr float DUTY_DIFF_2 = 0.04f;
inline constexpr float DUTY_DIFF_3 = 0.06f;
inline constexpr float DUTY_DIFF_4 = 0.10f;
inline constexpr float DUTY_DIFF_5 = 0.14f;
// 実運用目標（700mm/s時：角加速度目安2500deg/s^2，最高角速度目安430deg/s）に対し，
// duty_diff<=0.14までの実測ではヨーレートが最大でも171dps程度までしか届いておらず
// 大幅に不足している。運用域に向けた特性把握のため追加した水準（片輪はさらに深く
// 負転する領域に入る）
inline constexpr float DUTY_DIFF_6 = 0.20f;
inline constexpr float DUTY_DIFF_7 = 0.28f;
// 430dps付近の目標運用域に近い中間水準（TEST_MS_LONGで励振し真の整定を確認する）
inline constexpr float DUTY_DIFF_8  = 0.22f;
inline constexpr float DUTY_DIFF_9  = 0.24f;
inline constexpr float DUTY_DIFF_10 = 0.26f;
}

// 回転方向PRBS本同定（data_analysis2/prbs_rot_design.m，system_identification_flow.md §2 [3]）
// duty_diff振幅はrot_step_v700で確認した「クリーン」線形領域の上限(±0.06)を採用。
// Tc=4msはn=8固定のLFSRで高域分解能(tau_fast/2.8)側を優先した値
// （下限2.5*tau_slow/nとは両立しないため）。全周期255クロックが約1.02sに収まるため，
// 1試行1.1sでほぼ全周期を励振できる。
namespace config::prbs_rot {
inline constexpr float TRANSLATION_VELOCITY_MM_S = 700.f;
inline constexpr uint32_t ACCEL_MS   = 800;    // 並進速度700mm/sを閉ループで立ち上げる時間
inline constexpr float TC_SEC        = 0.004f; // クロック周期 [s]（1ms tick整数化）
inline constexpr float DUTY_DIFF_AMP = 0.06f;  // PRBS振幅（±）
inline constexpr float DURATION_SEC  = 1.1f;   // 1試行の励振時間

// 同定用4試行 + 検証(holdout)用2試行のPRBSシード
inline constexpr uint16_t SEED_T01   = 0x6A2D;
inline constexpr uint16_t SEED_T02   = 0x3F17;
inline constexpr uint16_t SEED_T03   = 0x9C84;
inline constexpr uint16_t SEED_T04   = 0x1E5B;
inline constexpr uint16_t SEED_VAL01 = 0x7D93;
inline constexpr uint16_t SEED_VAL02 = 0x4B26;
}

// 回転角速度PI制御（2自由度(FF)ではなく純粋PI。gain不確かさへの頑健性を優先）
//
// [設計方針] rot_step_v700_report.mdより，回転方向のプラントゲインKpは707〜2815dps/duty
// と振幅依存で大きく変動し（±正負非対称・duty不感帯突入による構造変化あり），単一のFFでは
// モデル誤差が大きい。PI（特に積分項）は定常ゲインの不確かさに対してロバストなため，FFを
// 使わずI主体で目標角速度へ追従させる。IMC整定 Kc=T/(K*λ), Ki=Kc/T ではTが小さいほど
// Ki/Kp=1/Tが大きくなり，結果的に「Iゲインが大きい」制御になる。
// 実機E1（旧ゲイン Kc=3.7e-4, Ti=23.1ms固定, 上限0.20）で ±200/+250dps を定常誤差1.5%以内で
// 追従できることを確認済み（data_analysis2/omega_step_analyze.m）。
//
// [ゲインスケジューリング] data_analysis2/rot_gain_scheduling_plan.md, rot_omega_pi_tune_sweep.m
// 動作点別のP1D同定で，時定数Tが振幅で10ms→160msと大きく変わる一方，初期角加速度K/Tは
// 1.7倍程度しか変わらないと判明した。IMC則 Kc=1/((K/T)*λ) は比K/Tで決まるため
// 比例ゲインKcはほぼ一定でよく，積分時間Ti(=Ki=Kc/Ti)のみを指令角速度ω_refでスケジュールする。
// Kc=5e-4は閉ループ模擬（プラント: 実測step応答から構成）で選定。スケジュール変数は
// レート制限後の指令値ω_ref（既知・無雑音）で，毎tick Ki・back-calculation時定数Ttを差し替える
// （MotorDriver::update）。積分項は出力単位で保持されるため，Kiを変えても出力は跳ばない（バンプレス）。
namespace config::pid_omega {
inline constexpr float KC_ROT = 5.0e-4f;   // [duty/dps] 比例ゲイン（動作点によらずほぼ一定）
inline constexpr float kp = KC_ROT;
inline constexpr float kd = 0.0f;

// Ti(|ω_ref|)の区分線形表 [s]。|ω_ref|>末尾は末尾値で固定。指令値ω_refの符号で正/負を選ぶ（方向別）
//
// [方向別Ti（F4で導入。現在の値は下の注記のとおり正負同一に戻した）] 実機E7で，+側(+430)はステップでもランプでもOSが10%前後出るのに対し，-側は
// 0.7〜4%と小さかった。閉ループ実測から推定した局所時定数は+側で約290ms(215〜376)，-側で約75ms
// (59〜92)。PIの零点(1/Ti)がプラント極(1/T_loc)より速い(Ti<T_loc)とOSが出るため，+側の高速域のみ
// Tiを延ばす（線形感度マップでは Ti=130〜180ms で+側相当のOSが10〜14% → 0〜4%，立上り+35〜60ms）。
// -側はTiを延ばすと立上りが急に遅くなる(T_loc≈75msで 173→404ms以上)ので現行のまま。
// 250dps以下は現状OSが許容範囲(3.5〜5.7%)なので緩やかに延ばすに留める。
// 0〜200dpsは正負同一（符号が切り替わる0付近でTiが跳ばない）。
// (data_analysis2/rot_gain_scheduling_plan.md §14.4)
inline constexpr int TI_TABLE_SIZE = 6;
inline constexpr float TI_OMEGA_BP[TI_TABLE_SIZE]   = {0.f, 100.f, 200.f, 250.f, 400.f, 430.f};   // [dps]
inline constexpr float TI_S_BP_POS[TI_TABLE_SIZE]   = {0.0090f, 0.01725f, 0.0345f, 0.0420f, 0.0615f, 0.06525f};  // ω_ref >= 0
inline constexpr float TI_S_BP_NEG[TI_TABLE_SIZE]   = {0.0090f, 0.01725f, 0.0345f, 0.0420f, 0.0615f, 0.06525f};  // ω_ref < 0
// [F5で+側Tiを現行(F3)値へ戻した] 実機E8で，+側のTi延長(F4: Ti(+430)=150ms)はOSを下げる一方で
// 90%到達が約150→341msと運用仕様(2500dps/s^2)の半分以下に悪化した。OS対策は下記の2自由度FFで行い，
// PIのTiは立上りを損なわない値(F3と同じ)に戻す。方向別にできる構造は残してある。

// ---- 2自由度FF（F5, data_analysis2/rot_gain_scheduling_plan.md §17, §18）----
// 出力: u_diff = u_ff_static(ω_ref) + a_ff(|ω_ref|)·dω_ref/dt + PI(ω_ref − ω)
// 従来は必要なduty(0.18〜0.27)を積分項だけで作っていたため，指令に対しωが遅れる間の誤差が積分に
// 積み上がりOSになっていた。FFで必要dutyを直接与え，PIはモデル誤差(±0.03程度)の補正に回す。
// 試算(rot_ff2dof_study.m, 実測に較正した21プラント，ランプ2500dps/s^2の+430)：OS 6.4→2.7%
// (最悪13.6→7.3%)，90%到達 229→164ms，指令に対する遅れ 47→7%。静的FFだけではOSが悪化(13%)し，
// 加速度FFで遅れを消すことが本質。加速度FF係数が半分だと効果が消え，過大側には寛容なので係数はやや大きめにする。
// ステップ指令は加速度が無限大でFFが意味を持たないため，FFはレート制限後のランプ指令(omega_ref)で使う前提。
inline constexpr bool OMEGA_FF_ENABLED = true;   // 既定。実行時は MotorDriver::setOmegaFFEnabled() で試験ごとに切替可

// 静的FF: 必要duty |u_ff|(|ω_ref|)（符号はω_refに従う）。補間点はTI_OMEGA_BPと共通。
// 実測の閉ループ定常duty u_ss（E1〜E8のrun間ばらつき）の**小さい側（保守側）**で設計する：
// FFが不足する分はPIの積分が補うので過大に出さない（過大だとOSになる）。
//   +側: 100dps 0.090〜0.096, 200dps 0.165〜0.187, 250dps 0.165〜0.196, 400dps 0.191〜0.206, 430dps 0.176〜0.222
//   -側: 100dps 0.102〜0.126, 200dps 0.191〜0.200, 250dps 0.186〜0.216, 400dps 0.234〜0.250, 430dps 0.220〜0.272
inline constexpr float FF_U_POS[TI_TABLE_SIZE] = {0.f, 0.090f, 0.170f, 0.172f, 0.178f, 0.178f};   // ω_ref >= 0
inline constexpr float FF_U_NEG[TI_TABLE_SIZE] = {0.f, 0.100f, 0.185f, 0.186f, 0.220f, 0.220f};   // ω_ref < 0（大きさ）

// 加速度FF係数 [duty/(dps/s)] ≈ T_loc/K_loc（動作点別。局所回帰: +側 T≈290ms/K≈6000, -側 T≈75ms/K≈1700 で
// 約4.4〜4.8e-5，0からのstep由来のH1では1.4〜2.0e-5）。2500dps/s^2で 0.04〜0.14 duty。
// 効果が消えるのは係数不足側で，過大側には寛容（試算 rot_ff2dof_study.m）。
// [方向別（F7）] 実機E10で，+側の加速度FF係数を**全動作点で**1.6倍（`ffhi`）にすると +430 の
// OS 4.3→2.1%，90%到達 141→126ms とさらに改善した（隣り合うffとの差: OS -3.1pt, -22ms）。
// 実機で検証済みのこの構成（+側の表を全域1.6倍）をそのまま採用する。250dps以上だけを上げる案は，
// 簡易プラント族の試算（tools/host_test）で速いプラント(T=100ms)のOSが増え平均OSが悪化
// （6.8→9.9%）したうえ実機未検証のため採らない。+250/+100dpsで悪化しないかは実機で確認する（E11）。
// -側は必要dutyが大きく上限0.28の余裕が乏しい（E9 7.7V: -430で1.6倍にすると静的0.22+加速度0.14が
// 上限を大きく超え90%到達296msに悪化）ため据え置く。
inline constexpr float FF_ACC_POS[TI_TABLE_SIZE] = {2.4e-5f, 3.2e-5f, 4.48e-5f, 4.8e-5f, 5.6e-5f, 5.6e-5f};   // ω_ref >= 0（F5の表の1.6倍）
inline constexpr float FF_ACC_NEG[TI_TABLE_SIZE] = {1.5e-5f, 2.0e-5f, 2.8e-5f, 3.0e-5f, 3.5e-5f, 3.5e-5f};   // ω_ref < 0
// 加速度FFに使う dω_ref/dt の上限 [dps/s]。ステップ指令(レート制限なし)で加速度FFが発散しないようにする
inline constexpr float OMEGA_ACCEL_FF_MAX = 5000.f;

// 目標角速度の最大角加速度 [dps/s]（指令のレート制限）。運用仕様（700mm/s時 角加速度2500deg/s²）に合わせる。
// 実機E5でステップ指令に対し+側高速(+400/+430)で6〜12%のオーバーシュートが再現したため，
// 指令をランプ化して誤差積分の行き過ぎを抑える（data_analysis2/rot_gain_scheduling_plan.md §12, §13）。
// 十分大きな値（例 1.0e9f）にすると従来のステップ指令と同じ動作になる。実行時は
// MotorDriver::setOmegaAccelLimit()で試験ごとに上書きできる（このconstexprが既定値）。
inline constexpr float OMEGA_ACCEL_MAX = 2500.f;

// 出力(duty_diff)飽和：実機E3で負方向は正方向より約0.03多くdutyを要し（機体の左右バイアス，
// 高速ほど大きい），-400/-430dpsの定常duty≈-0.25では旧上限0.26まで余裕0.01しかなく
// -430で飽和が144ms続いた。open-loopの±0.28 stepで974dpsまで検証済みの範囲である0.28まで許す
// （正方向は約0.195で足りるため上限には触れず，実質負側の余裕確保。
// data_analysis2/rot_gain_scheduling_plan.md §11.2）。0.28超は未検証のため上げない
inline constexpr float DUTY_DIFF_LIMIT = 0.28f;

// ---- F6：バッテリ電圧補償と電圧基準の出力上限（data_analysis2/rot_gain_scheduling_plan.md §23）----
// 回転のプラント感度は電圧にほぼ比例する（必要な出力電圧 u_ss×V は約一定：-430で1.9〜2.0V。E9〜E11）。
// そこで角速度制御全体（FF表・PI・Ti・back-calculationの上限）を，基準電圧 BATT_V_REF でのduty空間で行い，
// 出力の duty_diff = (PI+FF出力) × BATT_V_REF / V_batt として実dutyへ換算する。
//   - 表・Kc・Tiは従来の値（7.6〜8.2Vの実測から作ったもの，中央7.9V付近）のまま使え，電圧が変わっても
//     ループゲイン（Kc×K）と必要dutyの表が一定に保たれる（-側の感度ドリフトの約7割は電圧で説明できる）。
//   - 出力上限は電圧で持つ：実dutyの上限 = DUTY_DIFF_LIMIT_V / V_batt。基準電圧でのduty空間では
//     DUTY_DIFF_LIMIT_V / BATT_V_REF で一定（7.9Vで約0.329）。電圧が下がるほど実duty上限は増え，
//     -430で起きていた「FF合計(0.31) > 上限0.28」の構造的な飽和と低電圧での飽和が解消する。
//     速い輪のdutyは基本duty(0.07)+diff/2で，MAX_DUTY(0.95)に十分余裕がある。
//     上限0.28超はopen-loopでは未検証（初回はE12で±430を含めて確認する）。
// 実行時に MotorDriver::setOmegaBattCompEnabled(false) で従来動作（補償なし・上限0.28）へ戻せる（A/B試験用）。
inline constexpr bool OMEGA_BATT_COMP_ENABLED = true;
inline constexpr float BATT_V_REF = 7.9f;            // [V] 基準電圧（FF表を作った実測の中央値）
inline constexpr float BATT_V_MIN = 6.0f;            // [V] 換算に使う電圧の下限（電圧読み値の異常時の暴走防止）
inline constexpr float BATT_V_MAX = 9.0f;            // [V] 〃 上限
inline constexpr float DUTY_DIFF_LIMIT_V = 2.6f;     // [V] duty_diffの出力上限（電圧換算）
}

// PRBS入力設計（並進方向, data_analysis2/prbs_design.m）
// Tc下限(LFSRカバレッジ): 2.5*tau_slow/n, Tc上限(速い極を粗く均さない): tau_fast/2.8
// 採用: Tc=0.145s, n=8(PRBSクラスのタップ多項式に対応した固定値), duty=[0.08,0.16]
// 1試行の長さは走行距離を抑えるため3.0s→2.4s(80%)へ短縮。1試行あたりの励振ビット数が
// 減る分，同定用試行を6→8に増やしてデータの多様性を確保している（検証用2試行は据え置き）
namespace config::prbs_trans {
inline constexpr float TC_SEC       = 0.145f;
inline constexpr float DUTY_MIN     = 0.08f;
inline constexpr float DUTY_MAX     = 0.16f;
inline constexpr float DURATION_SEC = 2.4f;

// 同定用8試行 + 検証(holdout)用2試行のPRBSシード
inline constexpr uint16_t SEED_T01   = 0x1A2B;
inline constexpr uint16_t SEED_T02   = 0x3C4D;
inline constexpr uint16_t SEED_T03   = 0x5E6F;
inline constexpr uint16_t SEED_T04   = 0x7890;
inline constexpr uint16_t SEED_T05   = 0xABCD;
inline constexpr uint16_t SEED_T06   = 0xEF01;
inline constexpr uint16_t SEED_T07   = 0x4E2A;
inline constexpr uint16_t SEED_T08   = 0x9D31;
inline constexpr uint16_t SEED_VAL01 = 0x2468;
inline constexpr uint16_t SEED_VAL02 = 0x1357;
}

namespace config::menu {
inline constexpr uint8_t MAX_CHILDREN = 10;  // motor_sysid_prbs_(prbs 0~9)が最大
}

namespace config::mode_selector {
inline constexpr float ENC_THRESH = 10.f;
inline constexpr float IR_THRESH = 500.f;
inline constexpr float KORIKORI = 0.1f;

inline constexpr float ACC_THRESH = 0.f * config::imu::G; // mm/s^2
}