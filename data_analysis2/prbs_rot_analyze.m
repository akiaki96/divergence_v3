%% PRBS本同定（回転方向, 並進700mm/s固定）
%
% tools/log/prbs_rot_v700_x/prbs_rot_{t01..t04,val01,val02}.csv を読み込み，
% procest(P1)によるパラメトリック推定とETFEによるノンパラメトリック検証を行う。
% t01〜t04を同定用（merge統合），val01/val02をholdout検証用として使用する
% （system_identification_flow.md §2 [3][4][5]）。
%
% 【既知の不具合による回避策】ログの"duty_diff"列は，PRBSでduty差を駆動した場合に
% 実際に印加された値を反映しない（MotorDriver::getDutyDiff()が静的setDutyDiff()の値を
% 返す実装になっており，PRBS由来の値が反映されていない）。そのため本スクリプトでは
% 実効duty差を "RightDuty - LeftDuty" から直接算出する（これは実際にモータへ
% 印加された値であり，PRBS出力そのものと一致することを確認済み）。
%
% CSV列: Global_time,left_encoder_velocity,right_encoder_velocity,battery,
%        LeftDuty,RightDuty,gyro_z,accel_x,target_velocity_x,
%        pid_integral_term,pid_feedforward,pid_saturated,duty_diff(※使用しない)

clear; clc;

results_dir = 'results';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

id_labels  = {'t01', 't02', 't03', 't04'};
val_labels = {'val01', 'val02'};

Ts = 0.001;   % 1kHz制御周期

% 走行区間中の|accel_x|通常最大値を十分上回る閾値（並進PRBSと同じ考え方で壁衝突を検知）
ACCEL_COLLISION_THRESH = 8000;   % [mm/s^2]
TRIM_MARGIN_SAMPLES    = 10;

%% データ読み込み
n_id = numel(id_labels);
id_data = cell(n_id, 1);
fprintf('--- 同定用データの読み込み ---\n');
for i = 1:n_id
    fname = sprintf('../tools/log/prbs_rot_v700_x/prbs_rot_%s.csv', id_labels{i});
    [id_data{i}, info] = load_prbs_rot_trial(fname, Ts, ACCEL_COLLISION_THRESH, TRIM_MARGIN_SAMPLES);
    fprintf('%s: %s\n', id_labels{i}, info);
end

n_val = numel(val_labels);
val_data = cell(n_val, 1);
fprintf('\n--- 検証(holdout)用データの読み込み ---\n');
for i = 1:n_val
    fname = sprintf('../tools/log/prbs_rot_v700_x/prbs_rot_%s.csv', val_labels{i});
    [val_data{i}, info] = load_prbs_rot_trial(fname, Ts, ACCEL_COLLISION_THRESH, TRIM_MARGIN_SAMPLES);
    fprintf('%s: %s\n', val_labels{i}, info);
end

%% マルチ実験統合
id_merged  = merge(id_data{:});
val_merged = merge(val_data{:});

%% procest(P1)によるパラメトリック推定
opt = procestOptions('SearchMethod', 'auto', 'InitialCondition', 'zero');
m_p1 = procest(id_merged, 'P1', opt);

fprintf('\n--- 回転PRBS本同定結果 (P1, t01〜t04統合) ---\n');
present(m_p1);

cov_p1  = getcov(m_p1);
Kp_hat  = m_p1.Kp;  Kp_std  = sqrt(cov_p1(1, 1));
Tp1_hat = m_p1.Tp1; Tp1_std = sqrt(cov_p1(2, 2));
fit_id  = m_p1.Report.Fit.FitPercent;

fprintf('Kp_rot  = %.1f ± %.1f [dps/duty]\n', Kp_hat, Kp_std);
fprintf('Tp1_rot = %.4f ± %.4f [s]\n', Tp1_hat, Tp1_std);

%% ETFE（ノンパラメトリック）との比較
m_etfe = etfe(id_merged);

fig = figure;
bode(m_p1, m_etfe);
legend('procest P1', 'ETFE', 'Location', 'best');
title('回転PRBS同定: procest(P1) vs ETFE');
savefig(fig, fullfile(results_dir, 'prbs_rot_bode_p1_vs_etfe.fig'));
exportgraphics(fig, fullfile(results_dir, 'prbs_rot_bode_p1_vs_etfe.png'));

%% Holdoutデータでの検証
fig = figure;
compare(val_merged, m_p1);
title('回転PRBS Holdout検証（val01, val02統合）: procest P1モデル');
savefig(fig, fullfile(results_dir, 'prbs_rot_holdout_compare.fig'));
exportgraphics(fig, fullfile(results_dir, 'prbs_rot_holdout_compare.png'));

%% 残差診断
fig = figure;
resid(id_merged, m_p1);
title('残差診断（同定用データ, t01〜t04統合）');
savefig(fig, fullfile(results_dir, 'prbs_rot_resid.fig'));
exportgraphics(fig, fullfile(results_dir, 'prbs_rot_resid.png'));

%% rot_step_v700（クリーン領域）との比較
step_Kp_avg  = mean([732.4, 731.2, 803.6, 788.8]);   % duty_diff=0.04,0.06(正負)平均
fprintf('\n--- rot_step_v700クリーン領域との比較 ---\n');
fprintf('Step平均(±0.04,0.06): Kp=%.1f dps/duty\n', step_Kp_avg);
fprintf('PRBS本同定(±0.06励振): Kp=%.1f±%.1f dps/duty, Tp1=%.4f±%.4fs, 適合率(平均)=%.1f%%\n', ...
    Kp_hat, Kp_std, Tp1_hat, Tp1_std, mean(fit_id));

%% 結果保存
summary = table(Kp_hat, Kp_std, Tp1_hat, Tp1_std, mean(fit_id), step_Kp_avg, ...
    'VariableNames', {'Kp_rot', 'Kp_std', 'Tp1_rot', 'Tp1_std', 'fit_percent_id_mean', 'step_Kp_avg'});
writetable(summary, fullfile(results_dir, 'prbs_rot_identification_summary.csv'));

%% ローカル関数

function [data, info] = load_prbs_rot_trial(filename, Ts, accel_thresh, margin_samples)
    T = readtable(filename);
    t = T.Global_time - T.Global_time(1);

    % 既知の不具合によりduty_diff列は使えないため，Right-Leftから実効diffを直接算出
    eff_diff = T.RightDuty - T.LeftDuty;
    gyro     = T.gyro_z;

    drive_idx = find(abs(eff_diff) > 1e-4);
    drive_start = drive_idx(1);
    drive_end_natural = drive_idx(end);

    accel = T.accel_x;
    collision_rel = find(abs(accel(drive_start:drive_end_natural)) > accel_thresh, 1, 'first');

    if ~isempty(collision_rel)
        collision_idx = collision_rel + drive_start - 1;
        drive_end = max(drive_start, collision_idx - margin_samples);
        info = sprintf('壁衝突を検知(t=%.3fs) → 打ち切り(%d/%dサンプル使用)', ...
            t(collision_idx), drive_end - drive_start + 1, drive_end_natural - drive_start + 1);
    else
        drive_end = drive_end_natural;
        info = sprintf('衝突なし（励振%dサンプル使用）', drive_end - drive_start + 1);
    end

    % 並進速度の立ち上げ加速区間(0〜ACCEL_MS)はgyro_zに大きな外乱性ノイズが乗るため
    % （duty_diff=0なのにgyro_zが大きく振れる区間があり，モデルに含めるとフィットが
    % 悪化する），励振開始直前の300ms（並進700mm/sが安定した後の静止区間）のみを
    % ゼロ初期条件用のベースラインとして使用する
    baseline_samples = 300;
    range_start = max(1, drive_start - baseline_samples);
    range = range_start:drive_end;
    data  = iddata(gyro(range), eff_diff(range), Ts);
end
