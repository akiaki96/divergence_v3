%% 回転方向 事前同定（並進700mm/s固定 + 左右duty差ステップ）
%
% tools/log/rot_step_v700_x/rot_step_v700_duty_{pos,neg}{002,004,006,010,014,020,028}.csv を読み込み，
% 並進速度700mm/s一定下での左右duty差(R-L)ステップに対するヨーレート(gyro_z)応答を解析する。
% system_identification_flow.md §2 [2]「回転step実験」に対応。
%
% 目的:
%  - duty差 vs 定常ヨーレートの関係（ゲイン・非線形性）を確認
%  - duty差0でも直進しない機体バイアスを，正負両方向のデータから分離する
%    （gyro_ss = a*(duty_diff - duty_diff0) の形でduty_diff0（真の直進duty差）を推定）
%  - IMU飽和(±2000dps)に対する余裕を確認
%
% CSV列: Global_time,left_encoder_velocity,right_encoder_velocity,battery,
%        LeftDuty,RightDuty,gyro_z,accel_x,target_velocity_x,
%        pid_integral_term,pid_feedforward,pid_saturated,duty_diff

clear; clc;

results_dir = 'results';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

labels    = {'pos002', 'pos004', 'pos006', 'pos010', 'pos014', 'pos020', 'pos028', ...
             'neg002', 'neg004', 'neg006', 'neg010', 'neg014', 'neg020', 'neg028'};
duty_vals = [0.02, 0.04, 0.06, 0.10, 0.14, 0.20, 0.28, ...
             -0.02, -0.04, -0.06, -0.10, -0.14, -0.20, -0.28];   % ラベルに対応する符号付きduty_diff

n = numel(labels);
gyro_ss   = zeros(n, 1);
gyro_max  = zeros(n, 1);
vavg_ss   = zeros(n, 1);
settle_ms = zeros(n, 1);

fig_ts = figure('Position', [50 50 1000 700]);
colors = lines(n);

for i = 1:n
    fname = sprintf('../tools/log/rot_step_v700_x/rot_step_v700_duty_%s.csv', labels{i});
    T = readtable(fname);
    t = T.Global_time - T.Global_time(1);

    duty_diff = T.duty_diff;
    onset  = find(duty_diff ~= 0, 1, 'first');
    offset = onset - 1 + find(duty_diff(onset:end) == 0, 1, 'first');

    t_rel = t(onset:offset - 1) - t(onset);
    gyro_win = T.gyro_z(onset:offset - 1);
    vL_win = T.left_encoder_velocity(onset:offset - 1);
    vR_win = T.right_encoder_velocity(onset:offset - 1);

    % 定常区間：励振窓の後半2/3（最初の1/3は立ち上がり過渡とみなし除外）
    ss_mask = t_rel > (t_rel(end) / 3);
    gyro_ss(i)  = mean(gyro_win(ss_mask));
    gyro_max(i) = max(abs(gyro_win));
    vavg_ss(i)  = mean((vL_win(ss_mask) + vR_win(ss_mask)) / 2);

    % 整定時間の目安：定常値の90%に初めて到達する時刻
    target_ss = gyro_ss(i);
    if target_ss ~= 0
        reached = find(abs(gyro_win) >= 0.9 * abs(target_ss), 1, 'first');
        settle_ms(i) = t_rel(reached) * 1000;
    end

    % 収束判定：末尾1/6区間と，その直前1/6区間の平均値を比較し，15%以上動いていれば
    % 励振時間内に整定しきっていない可能性が高いとして警告する
    n_win = numel(gyro_win);
    last_sixth  = gyro_win(round(n_win * 5 / 6) + 1:end);
    prev_sixth  = gyro_win(round(n_win * 4 / 6) + 1:round(n_win * 5 / 6));
    rel_change  = abs(mean(last_sixth) - mean(prev_sixth)) / max(abs(mean(last_sixth)), 1e-6);
    if rel_change > 0.15
        fprintf('  [警告] %s: 末尾で依然%.0f%%変化しており，600msでは整定しきっていない可能性\n', ...
            labels{i}, rel_change * 100);
    end

    fprintf('%s: duty_diff=%+.2f, gyro_ss=%+.1f dps, |gyro|max=%.1f dps, v_avg_ss=%.0f mm/s, 90%%整定=%.0f ms\n', ...
        labels{i}, duty_vals(i), gyro_ss(i), gyro_max(i), vavg_ss(i), settle_ms(i));

    subplot(3, 1, 1);
    plot(t_rel, gyro_win, 'Color', colors(i, :), 'DisplayName', sprintf('%s (%+.2f)', labels{i}, duty_vals(i)));
    hold on;

    subplot(3, 1, 2);
    plot(t_rel, vL_win, 'Color', colors(i, :), 'DisplayName', labels{i}); hold on;

    subplot(3, 1, 3);
    plot(t_rel, vR_win, 'Color', colors(i, :), 'DisplayName', labels{i}); hold on;
end

subplot(3, 1, 1); ylabel('gyro_z [dps]'); legend('Location', 'eastoutside'); grid on;
title('回転step応答（並進700mm/s一定下，duty差励振区間）');
subplot(3, 1, 2); ylabel('v_L [mm/s]'); grid on;
subplot(3, 1, 3); ylabel('v_R [mm/s]'); xlabel('t [s]（励振開始基準）'); grid on;

exportgraphics(fig_ts, fullfile(results_dir, 'rot_step_v700_timeseries.png'));
savefig(fig_ts, fullfile(results_dir, 'rot_step_v700_timeseries.fig'));

%% duty_diff vs 定常ヨーレート：線形フィットでゲインとバイアスを推定
p = polyfit(duty_vals, gyro_ss', 1);
a_rot = p(1);              % [dps / duty]
duty_diff0 = -p(2) / p(1); % 真の直進を与えるduty_diff（バイアス）

fprintf('\n--- 線形フィット結果 ---\n');
fprintf('回転ゲイン a_rot = %.1f [dps/duty]\n', a_rot);
fprintf('直進バイアス duty_diff0 = %.4f （この値だけR-Lに加えると直進になる）\n', duty_diff0);

fig_fit = figure;
plot(duty_vals, gyro_ss, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); hold on;
duty_fit = linspace(min(duty_vals) * 1.2, max(duty_vals) * 1.2, 100);
plot(duty_fit, a_rot * (duty_fit - duty_diff0), 'r-');
xline(0, 'k:', 'HandleVisibility', 'off');
yline(0, 'k:', 'HandleVisibility', 'off');
xlabel('duty\_diff (R-L)'); ylabel('定常ヨーレート [dps]');
legend('実測', '線形フィット', 'Location', 'northwest');
title('duty差 vs 定常ヨーレート');
grid on;

exportgraphics(fig_fit, fullfile(results_dir, 'rot_step_v700_gain_fit.png'));
savefig(fig_fit, fullfile(results_dir, 'rot_step_v700_gain_fit.fig'));

%% 正負対称性の確認
fprintf('\n--- 正負対称性（|gyro_ss|/|duty_diff|の比較） ---\n');
n_half = n / 2;
for i = 1:n_half
    pos_gain = gyro_ss(i) / duty_vals(i);
    neg_gain = gyro_ss(i + n_half) / duty_vals(i + n_half);
    fprintf('|duty_diff|=%.2f: +方向ゲイン=%.1f, -方向ゲイン=%.1f, 差=%.1f%%\n', ...
        abs(duty_vals(i)), pos_gain, neg_gain, 100 * (pos_gain - neg_gain) / ((pos_gain + neg_gain) / 2));
end

%% IMU飽和余裕の確認
fprintf('\n--- IMU飽和(±2000dps)に対する余裕 ---\n');
fprintf('観測された最大|gyro_z| = %.1f dps（飽和値の%.1f%%）\n', max(gyro_max), 100 * max(gyro_max) / 2000);

%% 結果保存
summary = table(labels', duty_vals', gyro_ss, gyro_max, vavg_ss, settle_ms, ...
    'VariableNames', {'label', 'duty_diff', 'gyro_ss_dps', 'gyro_max_dps', 'v_avg_ss_mm_s', 'settle90_ms'});
writetable(summary, fullfile(results_dir, 'rot_step_v700_summary.csv'));
