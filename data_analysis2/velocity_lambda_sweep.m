%% 並進速度PI+FF制御 λスイープ検証（高速化時の発散・不安定化リスクの事前確認）
%
% IMC整定 Kp=Tp1/(Kp_plant*lambda), Ki=Kp/Tp1 において，lambdaを小さくするほど
% 追従は速くなる（λ=大きい→遅い，小さい→速い。ゲインはlambdaに反比例して増大する）。
% ここでは同定済みP1プラント上に，ファームウェアと同一の離散実装
% （PI + back-calculationアンチワインドアップ + 電圧飽和 + feedforward）を再現し，
% λを小さくしていったときに発散・振動・過大な指令電圧ばたつきが生じないかを確認する。
%
% 重要な限界: このシミュレーションは同定済み線形P1モデル上での検証であり，
% IMC-PIは（モデルが正確なら）どれだけλを小さくしても連続時間では理論上安定である。
% 実機で本当に問題になるのは主に
%   (a) エンコーダ量子化ノイズがゲイン(∝1/λ)で増幅され指令電圧がばたつく
%   (b) 1kHzサンプリングによる離散化遅れ
%   (c) P1モデルに含まれない高速ダイナミクス（電気系・PWM・バックラッシュ等）
% であるため，本スクリプトはvelocity_step_300.csv実測から推定したエンコーダ
% ノイズを模擬入力に加え，指令電圧のばたつき・飽和頻度をλごとに定量化する。
% 「発散しない」ことの証明ではなく，「λをどこまで下げるとリスクが顕在化し始めるか」
% の当たりをつけるためのツールとして使うこと。

clear; clc;
results_dir = 'results';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% プラントモデル・feedforward（既存同定結果を使用）
Kp_plant = 1518.9;   % data_analysis2/prbs_identification_report.md
Tp1      = 0.4451;
A_GAIN   = 1738.0;    % data_analysis2/step_identification_report.md
U0_DEADZONE = 0.1360;
ZERO_VELOCITY_EPS = 1.0;

Vbatt = 7.9;
voltage_limit_ratio = 0.95;

Ts    = 0.001;   % config::control::DT_S
T_sim = 1.0;
target = 300;    % velocity_step_300 相当

% velocity_step_300.csv 定常区間(末尾0.5s)の実測速度標準偏差(~4.7mm/s)を
% エンコーダ観測ノイズとして模擬入力に加える
encoder_noise_std = 4.7;

lambdas = [0.10, 0.08, 0.06, 0.04, 0.03, 0.02, 0.01, 0.005];

fprintf('--- λスイープ結果（v step %d, エンコーダノイズ std=%.1fmm/s付き） ---\n', target, encoder_noise_std);
fprintf('%8s %10s %10s %14s %14s %12s\n', 'lambda', 'kp', 'ki', 'v_std(定常)', 'u_std(定常)', '飽和割合[%]');

fig = figure('Position', [100 100 900 700]);
colors = lines(numel(lambdas));

summary_rows = cell(numel(lambdas), 1);

for li = 1:numel(lambdas)
    lambda = lambdas(li);
    Kc = Tp1 / (Kp_plant * lambda);
    kp = Kc;
    ki = Kc / Tp1;
    Tt = Tp1;   % back-calculation時定数（config::pid_velocity_x::BACK_CALC_TT = T_p1 に合わせる）

    [t, v, u_hist, saturated_hist] = simulate_velocity_pid( ...
        target, Ts, T_sim, Kp_plant, Tp1, A_GAIN, U0_DEADZONE, ZERO_VELOCITY_EPS, ...
        kp, ki, Tt, voltage_limit_ratio * Vbatt, encoder_noise_std);

    ss_mask = t > (t(end) - 0.3);
    v_std = std(v(ss_mask));
    u_std = std(u_hist(ss_mask));
    sat_ratio = 100 * mean(saturated_hist);

    fprintf('%8.3f %10.5f %10.5f %14.2f %14.3f %12.1f\n', lambda, kp, ki, v_std, u_std, sat_ratio);
    summary_rows{li} = table(lambda, kp, ki, v_std, u_std, sat_ratio);

    subplot(2, 1, 1);
    plot(t, v, 'Color', colors(li, :), 'DisplayName', sprintf('\\lambda=%.3f', lambda));
    hold on;

    subplot(2, 1, 2);
    plot(t, u_hist, 'Color', colors(li, :), 'DisplayName', sprintf('\\lambda=%.3f', lambda));
    hold on;
end

subplot(2, 1, 1);
yline(target, 'k--', 'DisplayName', '目標');
ylabel('v [mm/s]');
legend('Location', 'southeast');
title(sprintf('\\lambdaスイープ: v step %d（エンコーダノイズ付きシミュレーション）', target));
grid on;

subplot(2, 1, 2);
yline(voltage_limit_ratio * Vbatt, 'k:', 'DisplayName', '電圧リミット');
yline(-voltage_limit_ratio * Vbatt, 'k:', 'HandleVisibility', 'off');
ylabel('u(指令電圧) [V]');
xlabel('t [s]');
legend('Location', 'southeast');
grid on;

savefig(fig, fullfile(results_dir, 'velocity_lambda_sweep.fig'));
exportgraphics(fig, fullfile(results_dir, 'velocity_lambda_sweep.png'));

summary = vertcat(summary_rows{:});
writetable(summary, fullfile(results_dir, 'velocity_lambda_sweep_summary.csv'));

%% ローカル関数

function [t, v, u_hist, saturated_hist] = simulate_velocity_pid( ...
        target, Ts, T_sim, Kp_plant, Tp1, A_GAIN, U0_DEADZONE, zero_eps, ...
        kp, ki, Tt, limit, encoder_noise_std)
    % ファームウェア(pid.cpp / motorDriver.cpp)と同一構造の離散シミュレーション
    N = round(T_sim / Ts);
    t = (0:N - 1)' * Ts;

    v = zeros(N, 1);
    u_hist = zeros(N, 1);
    saturated_hist = false(N, 1);

    integral_term = 0;
    previous_error = 0;

    rng(1);   % 実行ごとに条件を揃える（乱数の再現性）

    for k = 2:N
        current_meas = v(k - 1) + encoder_noise_std * randn();

        error = target - current_meas;
        derivative = (error - previous_error) / Ts;   %#ok<NASGU> kd=0のため未使用（実装と同型を保つため計算のみ残す）
        previous_error = error;

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
