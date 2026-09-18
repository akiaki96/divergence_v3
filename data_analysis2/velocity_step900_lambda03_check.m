%% v step 900, lambda=0.03 の実機挙動をシミュレーションで再現できるか確認
%
% 実機ログ(velocity_step_900.csv, lambda=0.03)では，ステップ直後(t=0.1〜0.2s)に
% 速度が900mm/s付近で複数回振動してから収束する挙動が見られた。飽和(pid_saturated)は
% 立ち上がり直後の数tickだけ発生している。
% これが「同定済み線形P1モデル+back-calculationアンチワインドアップ」で説明できる
% 制御ループの現象なのか，それともモデル化されていない要因（車輪スリップ・エンコーダ
% ノイズ等）によるものかを見るため，同一条件でシミュレーションする。
% 合わせて back_calc_tt を変えた場合に振動が緩和するかも確認する。

clear; clc;
results_dir = 'results';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

Kp_plant = 1518.9;
Tp1      = 0.4451;
A_GAIN   = 1738.0;
U0_DEADZONE = 0.1360;
ZERO_VELOCITY_EPS = 1.0;

Vbatt = 7.9;
voltage_limit_ratio = 0.95;
Ts    = 0.001;
T_sim = 0.6;
target = 900;

lambda = 0.03;
Kc = Tp1 / (Kp_plant * lambda);
kp = Kc;
ki = Kc / Tp1;

tt_candidates = [Tp1, Tp1 / 2, Tp1 / 5, Tp1 / 10];   % back_calc_tt を変えて比較

encoder_noise_std = 4.7;   % velocity_step_300.csv定常区間の実測値を流用（900では未評価点に注意）

fig = figure('Position', [100 100 900 500]);
colors = lines(numel(tt_candidates));
for i = 1:numel(tt_candidates)
    Tt = tt_candidates(i);
    [t, v, u_hist, sat_hist] = simulate_velocity_pid( ...
        target, Ts, T_sim, Kp_plant, Tp1, A_GAIN, U0_DEADZONE, ZERO_VELOCITY_EPS, ...
        kp, ki, Tt, voltage_limit_ratio * Vbatt, encoder_noise_std);

    subplot(2, 1, 1);
    plot(t, v, 'Color', colors(i, :), 'DisplayName', sprintf('Tt=%.4fs (=T_{p1}/%.0f)', Tt, Tp1 / Tt));
    hold on;

    subplot(2, 1, 2);
    plot(t, u_hist, 'Color', colors(i, :), 'DisplayName', sprintf('Tt=%.4fs', Tt));
    hold on;

    fprintf('Tt=%.4fs: 最大速度=%.0f mm/s, 飽和割合=%.1f%%, 0.3s時点誤差=%.1f mm/s\n', ...
        Tt, max(v), 100 * mean(sat_hist), target - v(round(0.3 / Ts)));
end

subplot(2, 1, 1);
yline(target, 'k--', 'DisplayName', '目標');
ylabel('v [mm/s]');
legend('Location', 'southeast');
title(sprintf('v step %d, \\lambda=%.2f: back\\_calc\\_tt を変えたシミュレーション比較', target, lambda));
grid on;

subplot(2, 1, 2);
yline(voltage_limit_ratio * Vbatt, 'k:');
yline(-voltage_limit_ratio * Vbatt, 'k:', 'HandleVisibility', 'off');
ylabel('u [V]');
xlabel('t [s]');
grid on;

exportgraphics(fig, fullfile(results_dir, 'velocity_step900_lambda03_tt_check.png'));
savefig(fig, fullfile(results_dir, 'velocity_step900_lambda03_tt_check.fig'));

%% ローカル関数（velocity_lambda_sweep.mと同一構造）

function [t, v, u_hist, saturated_hist] = simulate_velocity_pid( ...
        target, Ts, T_sim, Kp_plant, Tp1, A_GAIN, U0_DEADZONE, zero_eps, ...
        kp, ki, Tt, limit, encoder_noise_std)
    N = round(T_sim / Ts);
    t = (0:N - 1)' * Ts;

    v = zeros(N, 1);
    u_hist = zeros(N, 1);
    saturated_hist = false(N, 1);

    integral_term = 0;

    rng(1);

    for k = 2:N
        current_meas = v(k - 1) + encoder_noise_std * randn();
        error = target - current_meas;

        if abs(target) < zero_eps
            ff = 0;
        else
            ff = target / A_GAIN + sign(target) * U0_DEADZONE;
        end

        u_unsat = kp * error + integral_term + ff;

        if u_unsat > limit
            u_sat = limit;
            saturated_hist(k) = true;
        elseif u_unsat < -limit
            u_sat = -limit;
            saturated_hist(k) = true;
        else
            u_sat = u_unsat;
        end

        integral_term = integral_term + (ki * error + (u_sat - u_unsat) / Tt) * Ts;

        u_hist(k) = u_sat;
        v(k) = v(k - 1) + (Kp_plant * u_sat - v(k - 1)) / Tp1 * Ts;
    end
end
