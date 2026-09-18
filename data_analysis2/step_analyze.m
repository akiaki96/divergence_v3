%% 並進ステップ応答同定（duty 10%/15%/20%）
%
% tools/log/step_x/step_0_10.csv, step_0_15.csv, step_0_20.csv を読み込み，
% 各duty(一定値へのステップ入力)ごとに実効入力電圧 u = duty×Vbatt を入力，
% 左右エンコーダ速度の平均を出力として1次遅れモデル(P1)をprocestで同定する．
% さらに3点の(定常実効電圧, 定常速度)から，真の速度ゲイン a と
% 不感帯電圧 u0 を最小二乗直線フィットで推定する（v = a*(u - u0)）．
%
% CSV列: Global_time,left_encoder_velocity,right_encoder_velocity,battery,
%        Left_Duty,Right_Duty,gyro_z,accel_x

clear; clc;
rng(1);   % モンテカルロ結果を再現可能にする

results_dir = 'results';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

files = {
    '10', '../tools/log/step_x/step_0_10.csv';
    '15', '../tools/log/step_x/step_0_15.csv';
    '20', '../tools/log/step_x/step_0_20.csv';
};

n = size(files, 1);

opt = procestOptions('SearchMethod', 'auto', 'InitialCondition', 'zero');

data    = cell(n, 1);
models  = cell(n, 1);
u_step  = zeros(n, 1);
Kp_hat  = zeros(n, 1);
Kp_std  = zeros(n, 1);
Tp1_hat = zeros(n, 1);
Tp1_std = zeros(n, 1);
fit_pct = zeros(n, 1);

for i = 1:n
    label = files{i, 1};
    fname = files{i, 2};

    [data{i}, u_step(i)] = load_step_data(fname);

    models{i} = procest(data{i}, 'P1', opt);
    fprintf('\n--- duty %s%% ---\n', label);
    present(models{i});

    cov_i = getcov(models{i});
    Kp_hat(i)  = models{i}.Kp;
    Kp_std(i)  = sqrt(cov_i(1, 1));
    Tp1_hat(i) = models{i}.Tp1;
    Tp1_std(i) = sqrt(cov_i(2, 2));
    fit_pct(i) = models{i}.Report.Fit.FitPercent;

    fig = figure;
    compare(data{i}, models{i});
    title(sprintf('duty%s%% ステップ応答 vs 推定モデル', label));
    savefig(fig, fullfile(results_dir, sprintf('step_duty%s_fit.fig', label)));
    exportgraphics(fig, fullfile(results_dir, sprintf('step_duty%s_fit.png', label)));
end

%% 定常速度 v_ss = Kp * u_step の一覧
v_ss = Kp_hat .* u_step;

fprintf('\n--- 各dutyの同定結果 ---\n');
for i = 1:n
    fprintf('duty%s%%: Kp=%.1f±%.1f [mm/s/V], Tp1=%.4f±%.4f [s], fit=%.2f%%, u_step=%.3f [V], v_ss=%.1f [mm/s]\n', ...
        files{i, 1}, Kp_hat(i), Kp_std(i), Tp1_hat(i), Tp1_std(i), fit_pct(i), u_step(i), v_ss(i));
end

duty_pct = cellfun(@str2double, files(:, 1));
summary = table(duty_pct, u_step, Kp_hat, Kp_std, Tp1_hat, Tp1_std, fit_pct, v_ss, ...
    'VariableNames', {'duty_pct', 'u_step_V', 'Kp', 'Kp_std', 'Tp1_s', 'Tp1_std', 'fit_percent', 'v_ss_mm_s'});
writetable(summary, fullfile(results_dir, 'step_identification_summary.csv'));

%% 3点の(u_step, v_ss)から真の速度ゲインaと不感帯電圧u0を最小二乗推定
p = polyfit(u_step, v_ss, 1);
a_hat  = p(1);
u0_hat = -p(2) / a_hat;

fprintf('\n真の速度ゲイン a = %.2f [mm/s/V], 不感帯電圧 u0 = %.3f [V]\n', a_hat, u0_hat);

fig = figure;
plot(u_step, v_ss, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); hold on;
u_fit = linspace(0, max(u_step) * 1.1, 100);
plot(u_fit, a_hat * (u_fit - u0_hat), 'r-');
xlabel('u_{step} [V]'); ylabel('v_{ss} [mm/s]');
title('定常速度 vs 実効入力電圧');
legend('実測 (Kp×u_{step})', '線形フィット', 'Location', 'northwest');
grid on;
savefig(fig, fullfile(results_dir, 'v_ss_vs_u_step.fig'));
exportgraphics(fig, fullfile(results_dir, 'v_ss_vs_u_step.png'));

%% モンテカルロによるa, u0の不確かさ評価
Nmc = 100000;
a_mc  = zeros(Nmc, 1);
u0_mc = zeros(Nmc, 1);

for k = 1:Nmc
    Kp_mc   = Kp_hat + Kp_std .* randn(n, 1);
    v_ss_mc = Kp_mc .* u_step;
    p_mc    = polyfit(u_step, v_ss_mc, 1);
    a_mc(k)  = p_mc(1);
    u0_mc(k) = -p_mc(2) / p_mc(1);
end

a_mean = mean(a_mc); a_std = std(a_mc);
u0_mean = mean(u0_mc); u0_std = std(u0_mc);
u0_ci = prctile(u0_mc, [2.5, 97.5]);

fprintf('a  = %.1f ± %.1f [mm/s/V]\n', a_mean, a_std);
fprintf('u0 = %.4f ± %.4f [V]  (95%%区間: [%.4f, %.4f])\n', ...
    u0_mean, u0_std, u0_ci(1), u0_ci(2));

fig = figure; histogram(u0_mc, 50); xlabel('u_0 [V]'); title('不感帯電圧の推定分布');
exportgraphics(fig, fullfile(results_dir, 'u0_histogram.png'));

fig = figure; histogram(a_mc, 50);  xlabel('a [mm/s/V]'); title('速度ゲインの推定分布');
exportgraphics(fig, fullfile(results_dir, 'a_histogram.png'));

%% 最終推定値をテキストで保存
final = table(a_hat, a_mean, a_std, u0_hat, u0_mean, u0_std, u0_ci(1), u0_ci(2), ...
    'VariableNames', {'a_hat', 'a_mc_mean', 'a_mc_std', 'u0_hat', 'u0_mc_mean', 'u0_mc_std', 'u0_ci_lo', 'u0_ci_hi'});
writetable(final, fullfile(results_dir, 'step_identification_final.csv'));

%% ローカル関数

function [data, u_step] = load_step_data(filename)
    % CSVを読み込み，iddataを作成し，定常区間の実効入力電圧を返す
    brake_margin = 0.06;   % ログ終端の約50~60msはブレーキモードで同定に使えないため除外

    T = readtable(filename);

    t  = T.Global_time - T.Global_time(1);
    Ts = median(diff(t));

    % ログ終端はブレーキ指令により減速するため，同定対象から除外する
    valid = t <= (t(end) - brake_margin);
    T = T(valid, :);
    t = t(valid);

    v     = (T.left_encoder_velocity + T.right_encoder_velocity) / 2;
    u_eff = T.Left_Duty .* T.battery;   % 実効入力電圧 u = duty × Vbatt

    data = iddata(v, u_eff, Ts);

    % dutyが最初に立ち上がった時刻を基準に，定常区間(0.3s〜1.0s後)を推定
    step_idx = find(T.Left_Duty > 0, 1, 'first');
    t_step   = t(step_idx);
    mask_ss  = (t - t_step > 0.3) & (t - t_step < 1.0);
    u_step   = mean(u_eff(mask_ss));
end
