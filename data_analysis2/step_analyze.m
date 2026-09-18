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

files = {
    '10', '../tools/log/step_x/step_0_10.csv';
    '15', '../tools/log/step_x/step_0_15.csv';
    '20', '../tools/log/step_x/step_0_20.csv';
};

n = size(files, 1);

opt = procestOptions('SearchMethod', 'auto', 'InitialCondition', 'zero');

data   = cell(n, 1);
models = cell(n, 1);
u_step = zeros(n, 1);
Kp_hat = zeros(n, 1);
Kp_std = zeros(n, 1);

for i = 1:n
    label = files{i, 1};
    fname = files{i, 2};

    [data{i}, u_step(i)] = load_step_data(fname);

    models{i} = procest(data{i}, 'P1', opt);
    fprintf('\n--- duty %s%% ---\n', label);
    present(models{i});

    cov_i = getcov(models{i});
    Kp_hat(i) = models{i}.Kp;
    Kp_std(i) = sqrt(cov_i(1, 1));

    figure;
    compare(data{i}, models{i});
    title(sprintf('duty%s%% ステップ応答 vs 推定モデル', label));
end

%% 定常速度 v_ss = Kp * u_step の一覧
v_ss = Kp_hat .* u_step;

fprintf('\n--- 各dutyの同定結果 ---\n');
for i = 1:n
    fprintf('duty%s%%: Kp=%.1f±%.1f [mm/s/V], u_step=%.3f [V], v_ss=%.1f [mm/s]\n', ...
        files{i, 1}, Kp_hat(i), Kp_std(i), u_step(i), v_ss(i));
end

%% 3点の(u_step, v_ss)から真の速度ゲインaと不感帯電圧u0を最小二乗推定
p = polyfit(u_step, v_ss, 1);
a_hat  = p(1);
u0_hat = -p(2) / a_hat;

fprintf('\n真の速度ゲイン a = %.2f [mm/s/V], 不感帯電圧 u0 = %.3f [V]\n', a_hat, u0_hat);

figure;
plot(u_step, v_ss, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); hold on;
u_fit = linspace(0, max(u_step) * 1.1, 100);
plot(u_fit, a_hat * (u_fit - u0_hat), 'r-');
xlabel('u_{step} [V]'); ylabel('v_{ss} [mm/s]');
title('定常速度 vs 実効入力電圧');
legend('実測 (Kp×u_{step})', '線形フィット', 'Location', 'northwest');
grid on;

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

fprintf('a  = %.1f ± %.1f [mm/s/V]\n', mean(a_mc), std(a_mc));
fprintf('u0 = %.4f ± %.4f [V]  (95%%区間: [%.4f, %.4f])\n', ...
    mean(u0_mc), std(u0_mc), prctile(u0_mc, 2.5), prctile(u0_mc, 97.5));

figure; histogram(u0_mc, 50); xlabel('u_0 [V]'); title('不感帯電圧の推定分布');
figure; histogram(a_mc, 50);  xlabel('a [mm/s/V]'); title('速度ゲインの推定分布');

%% ローカル関数

function [data, u_step] = load_step_data(filename)
    % CSVを読み込み，iddataを作成し，定常区間の実効入力電圧を返す
    T = readtable(filename);

    t  = T.Global_time - T.Global_time(1);
    Ts = median(diff(t));

    v     = (T.left_encoder_velocity + T.right_encoder_velocity) / 2;
    u_eff = T.Left_Duty .* T.battery;   % 実効入力電圧 u = duty × Vbatt

    data = iddata(v, u_eff, Ts);

    % dutyが最初に立ち上がった時刻を基準に，定常区間(0.3s〜1.0s後)を推定
    step_idx = find(T.Left_Duty > 0, 1, 'first');
    t_step   = t(step_idx);
    mask_ss  = (t - t_step > 0.3) & (t - t_step < 1.0);
    u_step   = mean(u_eff(mask_ss));
end
