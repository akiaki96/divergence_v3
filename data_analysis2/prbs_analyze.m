%% PRBS本同定（並進方向）
%
% tools/log/prbs_trans_x/prbs_trans_{t01..t08,val01,val02}.csv を読み込み，
% procest(P1)によるパラメトリック推定とETFEによるノンパラメトリック検証を行う。
% t01〜t08を同定用（merge統合），val01/val02をholdout検証用として使用する
% （system_identification_flow.md §1 [5]）。
%
% 実機データの一部（例: prbs_trans_t01, t02, t03, t05）は，走行終盤に壁へ衝突し
% IMU値（特にaccel_x）が異常値になっている区間を含む。これは同定に使えないため，
% 走行区間内でaccel_xが閾値を超える最初の時刻を検知し，それ以降を自動的に
% 打ち切ってからiddataを構築する（load_prbs_trial参照）。
%
% CSV列: Global_time,left_encoder_velocity,right_encoder_velocity,battery,
%        "Left Duty","Right Duty",gyro_z,accel_x
%        （readtableはデフォルトで"Left Duty"→LeftDutyに変換される）

clear; clc;

results_dir = 'results';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

id_labels  = {'t01', 't02', 't03', 't04', 't05', 't06', 't07', 't08'};
val_labels = {'val01', 'val02'};

Ts = 0.001;   % 1kHz制御周期（config::control::DT_S）

% 走行区間中の|accel_x|通常最大値(実測 ~3000-4700mm/s^2)を十分上回る閾値。
% 壁衝突時は実測30000超まで跳ね上がるため，明確に分離できる
ACCEL_COLLISION_THRESH = 8000;   % [mm/s^2]
TRIM_MARGIN_SAMPLES    = 10;     % 衝突検知点より手前に余裕を持たせるサンプル数(10ms)

%% データ読み込み＋壁衝突トリミング
n_id = numel(id_labels);
id_data = cell(n_id, 1);

fprintf('--- 同定用データの読み込み・トリミング ---\n');
for i = 1:n_id
    fname = sprintf('../tools/log/prbs_trans_x/prbs_trans_%s.csv', id_labels{i});
    [id_data{i}, info] = load_prbs_trial(fname, Ts, ACCEL_COLLISION_THRESH, TRIM_MARGIN_SAMPLES);
    fprintf('%s: %s\n', id_labels{i}, info);
end

n_val = numel(val_labels);
val_data = cell(n_val, 1);
fprintf('\n--- 検証(holdout)用データの読み込み・トリミング ---\n');
for i = 1:n_val
    fname = sprintf('../tools/log/prbs_trans_x/prbs_trans_%s.csv', val_labels{i});
    [val_data{i}, info] = load_prbs_trial(fname, Ts, ACCEL_COLLISION_THRESH, TRIM_MARGIN_SAMPLES);
    fprintf('%s: %s\n', val_labels{i}, info);
end

%% マルチ実験統合
id_merged  = merge(id_data{:});
val_merged = merge(val_data{:});

%% procest(P1)によるパラメトリック推定
opt = procestOptions('SearchMethod', 'auto', 'InitialCondition', 'zero');
m_p1 = procest(id_merged, 'P1', opt);

fprintf('\n--- PRBS本同定結果 (P1, t01〜t08統合) ---\n');
present(m_p1);

cov_p1  = getcov(m_p1);
Kp_hat  = m_p1.Kp;  Kp_std  = sqrt(cov_p1(1, 1));
Tp1_hat = m_p1.Tp1; Tp1_std = sqrt(cov_p1(2, 2));
fit_id  = m_p1.Report.Fit.FitPercent;

fprintf('Kp  = %.1f ± %.1f [mm/s/V]\n', Kp_hat, Kp_std);
fprintf('Tp1 = %.4f ± %.4f [s]\n', Tp1_hat, Tp1_std);
fprintf('同定データへの適合: %.2f%%\n', fit_id);

%% P2モデルとの比較（残差に未モデル化構造が疑われる場合の構造検証）
m_p2 = procest(id_merged, 'P2', opt);
fprintf('\n--- P2モデルとの比較（2次系構造の要否確認） ---\n');
present(m_p2);
fprintf('P1適合: %.2f%%（平均）, P2適合: %.2f%%（平均）\n', ...
    mean(m_p1.Report.Fit.FitPercent), mean(m_p2.Report.Fit.FitPercent));

%% ETFE（ノンパラメトリック）との比較
m_etfe = etfe(id_merged);

fig = figure;
bode(m_p1, m_etfe);
legend('procest P1', 'ETFE', 'Location', 'best');
title('PRBS同定: procest(P1) vs ETFE');
savefig(fig, fullfile(results_dir, 'prbs_bode_p1_vs_etfe.fig'));
exportgraphics(fig, fullfile(results_dir, 'prbs_bode_p1_vs_etfe.png'));

%% Holdoutデータでの検証
fig = figure;
compare(val_merged, m_p1);
title('Holdout検証（val01, val02統合）: procest P1モデル');
savefig(fig, fullfile(results_dir, 'prbs_holdout_compare.fig'));
exportgraphics(fig, fullfile(results_dir, 'prbs_holdout_compare.png'));

%% 残差診断
fig = figure;
resid(id_merged, m_p1);
title('残差診断（同定用データ, t01〜t08統合）');
savefig(fig, fullfile(results_dir, 'prbs_resid.fig'));
exportgraphics(fig, fullfile(results_dir, 'prbs_resid.png'));

%% ステップ応答同定（3水準平均）との比較
step_Kp_avg  = mean([1450.4, 1529.4, 1591.2]);
step_Tp1_avg = mean([0.4485, 0.4165, 0.4260]);

fprintf('\n--- ステップ応答(3水準平均, 暫定値)との比較 ---\n');
fprintf('Step平均   : Kp=%.1f, Tp1=%.4f\n', step_Kp_avg, step_Tp1_avg);
fprintf('PRBS本同定 : Kp=%.1f±%.1f, Tp1=%.4f±%.4f\n', Kp_hat, Kp_std, Tp1_hat, Tp1_std);

%% 結果保存
summary = table(Kp_hat, Kp_std, Tp1_hat, Tp1_std, fit_id, step_Kp_avg, step_Tp1_avg, ...
    'VariableNames', {'Kp', 'Kp_std', 'Tp1', 'Tp1_std', 'fit_percent_id', 'step_Kp_avg', 'step_Tp1_avg'});
writetable(summary, fullfile(results_dir, 'prbs_identification_summary.csv'));

%% ローカル関数

function [data, info] = load_prbs_trial(filename, Ts, accel_thresh, margin_samples)
    % CSVを読み込み，走行区間内で壁衝突（|accel_x|が閾値超過）を検知した場合は
    % それ以降を打ち切ってiddataを構築する
    T = readtable(filename);
    t = T.Global_time - T.Global_time(1);

    duty  = T.LeftDuty;   % 並進PRBSは左右同相（RightDutyと同一）
    v     = (T.left_encoder_velocity + T.right_encoder_velocity) / 2;
    u_eff = T.LeftDuty .* T.battery;

    drive_idx          = find(duty ~= 0);
    drive_start         = drive_idx(1);
    drive_end_natural   = drive_idx(end);

    accel = T.accel_x;
    collision_rel = find(abs(accel(drive_start:drive_end_natural)) > accel_thresh, 1, 'first');

    if ~isempty(collision_rel)
        collision_idx = collision_rel + drive_start - 1;
        drive_end = max(drive_start, collision_idx - margin_samples);
        info = sprintf('壁衝突を検知(t=%.3fs) → t=%.3fsで打ち切り(%d/%dサンプル使用)', ...
            t(collision_idx), t(drive_end), drive_end, drive_end_natural);
    else
        drive_end = drive_end_natural;
        info = sprintf('衝突なし（駆動%dサンプル全て使用）', drive_end - drive_start + 1);
    end

    range = 1:drive_end;   % 先頭の静止区間(オフセット/ゼロ初期条件用)を含めて使用
    data  = iddata(v(range), u_eff(range), Ts);
end
