%% PRBS入力設計（回転方向, 並進700mm/s固定）
%
% rot_step_v700_report.md の結果（duty_diff ±0.02〜0.06の「クリーン」線形領域）から，
% 回転方向PRBS本同定実験のクロック周期Tc・duty_diff振幅を設計し，
% 同定済み1次近似モデル上で事前シミュレーション検証する。
% system_identification_flow.md §2 [3]「回転PRBS同定」に対応。
%
% 設計方針（prbs_design.m/system_identification_flow.mdと同じ考え方）:
%  - クロック周期Tc：最速点(最小τ)に対し Tc ≲ τ_fast/2.8
%  - 段数n：実装済みLFSR n=8固定。最遅点(最大τ)に対し nTc ≳ 2.5τ_slow を目安に確認
%  - duty_diff振幅：rot_step_v700で「クリーン」と確認された±0.06以内に収める
%    （±0.10以上は遅い方の車輪が不感帯付近/負転に入る非線形領域のため除外）
%  - Tcは1ms制御周期の整数tickでしか実現できない（PRBS::configureのticks_per_clock）

clear; clc;

results_dir = 'results';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% rot_step_v700の「クリーン」領域(±0.04, ±0.06)から時定数を推定
% t90（定常値の90%到達時間）から1次系換算: y=yss(1-e^-t/tau), 0.9=1-e^-t90/tau
% -> tau = t90 / ln(10) = t90 / 2.3026
t90_pos004 = 0.028; t90_pos006 = 0.030;
t90_neg004 = 0.036; t90_neg006 = 0.039;

tau = [t90_pos004, t90_pos006, t90_neg004, t90_neg006] / log(10);
labels_tau = {'pos004', 'pos006', 'neg004', 'neg006'};

[tau_fast, i_fast] = min(tau);
[tau_slow, i_slow] = max(tau);

fprintf('--- クリーン領域(duty_diff=0.04, 0.06)の時定数換算 ---\n');
for i = 1:numel(tau)
    fprintf('%s: tau=%.2f ms\n', labels_tau{i}, tau(i) * 1000);
end
fprintf('最速: %s (tau=%.2f ms), 最遅: %s (tau=%.2f ms)\n', ...
    labels_tau{i_fast}, tau_fast * 1000, labels_tau{i_slow}, tau_slow * 1000);

% ±0.02は静止摩擦の影響で正負差16.7%と最も非線形性が大きく，代表的な時定数の
% 見積もりには使わない（rot_step_v700_report.md §4参照）

%% クロック周期Tcの決定
n = 8;   % 実装済みLFSR段数（固定）
Tc_upper = tau_fast / 2.8;
Tc_lower = 2.5 * tau_slow / n;

fprintf('\nTc上限 (tau_fast/2.8) = %.3f ms\n', Tc_upper * 1000);
fprintf('Tc下限 (2.5*tau_slow/n) = %.3f ms\n', Tc_lower * 1000);

if Tc_lower > Tc_upper
    fprintf(['→ 両条件を同時に満たすTcは存在しない（回転の時定数が並進より短く，' ...
        'n=8固定のLFSRでは高域分解能と低域カバレッジを両立できない）\n']);
end

% Tcは1ms tick単位でしか実現できない(PRBS::configureのticks_per_clock=round(Tc*1000))。
% 高域分解能(Tc<=Tc_upper)を優先し，Tc=4msを採用（Tc=5msだとTc_upperを15%超過する一方，
% Tc=4msならTc_upperのマージン内に収まる）。カバレッジ不足は複数試行のmergeで補う。
Tc = 0.004;
fprintf('採用 Tc = %.0f ms（1ms tick整数化，高域分解能を優先）\n', Tc * 1000);

pulse_max = n * Tc;
coverage_target = 2.5 * tau_slow;
fprintf('最長パルス幅 nTc = %.1f ms（目安 %.1f ms の%.0f%%）\n', ...
    pulse_max * 1000, coverage_target * 1000, 100 * pulse_max / coverage_target);

%% duty_diff振幅の決定
DUTY_DIFF_AMP = 0.06;   % rot_step_v700で確認したクリーン領域の上限
fprintf('\n採用 duty_diff振幅 = ±%.2f（rot_step_v700実測でゲイン707〜804dps/duty，正負差1.9%%）\n', ...
    DUTY_DIFF_AMP);

%% 試行時間の決定：n=8の全周期(2^8-1=255クロック)をカバーする長さ
Ts_sim = 0.001;
full_period_ticks = 2^n - 1;
full_period_s = full_period_ticks * Tc;
fprintf('\nLFSR全周期 = %d クロック = %.3f s（Tc=%.0fmsなら1試行で全周期をほぼ包含できる）\n', ...
    full_period_ticks, full_period_s, Tc * 1000);

T_trial = 1.1;   % [s] 全周期(1.02s)に余裕を持たせた試行時間
fprintf('採用 試行時間 = %.2f s\n', T_trial);

%% 事前シミュレーション：クリーン領域代表値での応答確認
Kp_rot = mean([732.4, 731.2, 803.6, 788.8]);   % [dps/duty] rot_step_v700_report.mdの±0.04,0.06平均
Tp1_rot = mean(tau);                            % [s]

fprintf('\n--- シミュレーション用代表モデル ---\n');
fprintf('Kp_rot = %.1f dps/duty, Tp1_rot = %.4f s\n', Kp_rot, Tp1_rot);

Nclock = round(Tc / Ts_sim);
Nbits = round(T_trial / Tc);
prbs = idinput(Nbits * Nclock, 'prbs', [0 1 / Nclock], [-1 1]);
duty_diff_seq = DUTY_DIFF_AMP * prbs;   % 対称PRBS: ±DUTY_DIFF_AMP

G = tf(Kp_rot, [Tp1_rot 1]);
t_sim = (0:numel(duty_diff_seq) - 1)' * Ts_sim;
gyro_sim = lsim(G, duty_diff_seq, t_sim);
heading_cum = cumtrapz(t_sim, gyro_sim);   % 累積ヨー角 [deg]

fprintf('シミュレーション上の最大|gyro| = %.1f dps（IMU飽和2000dpsの%.1f%%）\n', ...
    max(abs(gyro_sim)), 100 * max(abs(gyro_sim)) / 2000);
fprintf('試行終了時の累積ヨー角 = %.1f deg\n', heading_cum(end));
fprintf('試行中の最大|累積ヨー角| = %.1f deg\n', max(abs(heading_cum)));

fig = figure;
subplot(3, 1, 1);
plot(t_sim, duty_diff_seq);
ylabel('duty\_diff'); title(sprintf('回転PRBS入力設計案 (Tc=%.0fms, ±%.2f)', Tc * 1000, DUTY_DIFF_AMP));
grid on;

subplot(3, 1, 2);
plot(t_sim, gyro_sim);
yline(2000, 'k:'); yline(-2000, 'k:', 'HandleVisibility', 'off');
ylabel('gyro_z [dps]'); grid on;

subplot(3, 1, 3);
plot(t_sim, heading_cum);
ylabel('累積ヨー角 [deg]'); xlabel('t [s]');
grid on;

savefig(fig, fullfile(results_dir, 'prbs_rot_design_check.fig'));
exportgraphics(fig, fullfile(results_dir, 'prbs_rot_design_check.png'));

%% 設計パラメータの保存
design = table(Tc, n, DUTY_DIFF_AMP, T_trial, Kp_rot, Tp1_rot, ...
    max(abs(gyro_sim)), heading_cum(end), max(abs(heading_cum)), ...
    'VariableNames', {'Tc_s', 'n_stages', 'duty_diff_amp', 'T_trial_s', 'Kp_rot', 'Tp1_rot_s', ...
    'gyro_max_dps', 'heading_end_deg', 'heading_max_abs_deg'});
writetable(design, fullfile(results_dir, 'prbs_rot_design_params.csv'));
