%% 並進速度PI+FF制御 追従性・アンチワインドアップ・feedforward検証
%
% tools/log/velocity_step_x/velocity_step_{300,600,900,neg600}.csv を読み込み，
% 目標速度への追従性，PI積分項のワインドアップ有無，feedforwardの寄与を可視化する。
%
% 対象ログは motor_id.cpp の id_init_log_velocity() で以下の列を記録している：
%   target_velocity_x  : 目標速度 r(t) [mm/s]
%   pid_integral_term  : PIDController内部の積分項の出力寄与 [V]（ki既反映済み）
%   pid_feedforward    : velocity_x_ff(target)の出力 [V]
%   pid_saturated      : 電圧飽和中は1，そうでなければ0
%
% u(t) = kp*e(t) + pid_integral_term(t) + pid_feedforward(t) （kd=0のため微分項はなし）
% がほぼ成立するはずなので，実効入力電圧 LeftDuty*battery と比較して整合性も確認する。
%
% 使い方: 下のtargets{}から実施済みのログ（例: '300'のみ）だけ残して実行してもよい。

clear; clc;

results_dir = 'results';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

targets = {'300', '600', '900', 'neg600'};

for i = 1:numel(targets)
    label = targets{i};
    fname = sprintf('../tools/log/velocity_step_x/velocity_step_%s.csv', label);
    if ~isfile(fname)
        fprintf('%s: ファイルが見つからないためスキップ（%s）\n', label, fname);
        continue;
    end
    analyze_one_trial(fname, label, results_dir);
end

%% ローカル関数

function analyze_one_trial(fname, label, results_dir)
    T = readtable(fname);
    t = T.Global_time - T.Global_time(1);

    v_actual = (T.left_encoder_velocity + T.right_encoder_velocity) / 2;
    v_target = T.target_velocity_x;
    integral_term = T.pid_integral_term;
    ff = T.pid_feedforward;
    saturated = T.pid_saturated;
    u_eff = T.LeftDuty .* T.battery;   % 実効印加電圧（fb+ffの合算結果がdutyに変換されたもの）

    error = v_target - v_actual;

    fig = figure('Position', [100 100 900 800]);

    subplot(4, 1, 1);
    plot(t, v_target, 'r--', 'LineWidth', 1.2); hold on;
    plot(t, v_actual, 'b-');
    ylabel('v [mm/s]');
    legend('目標 target\_velocity\_x', '実測 (encoder平均)', 'Location', 'best');
    title(sprintf('速度追従性: velocity\\_step\\_%s', label));
    grid on;

    subplot(4, 1, 2);
    plot(t, error, 'k-');
    ylabel('追従誤差 [mm/s]');
    grid on;

    subplot(4, 1, 3);
    plot(t, integral_term, 'm-', 'DisplayName', '積分項 I(t) [V]'); hold on;
    plot(t, ff, 'g-', 'DisplayName', 'feedforward [V]');
    plot(t, u_eff, 'b:', 'DisplayName', '実効入力電圧 u=duty*Vbatt [V]');
    ylabel('電圧 [V]');
    legend('Location', 'best');
    grid on;

    subplot(4, 1, 4);
    stairs(t, saturated, 'r-', 'LineWidth', 1.2);
    ylim([-0.2 1.2]);
    ylabel('飽和中(1/0)');
    xlabel('t [s]');
    grid on;

    savefig(fig, fullfile(results_dir, sprintf('velocity_step_%s_diag.fig', label)));
    exportgraphics(fig, fullfile(results_dir, sprintf('velocity_step_%s_diag.png', label)));

    % --- 数値サマリ ---
    ss_mask = t > (t(end) - 0.5);   % 末尾0.5秒を定常区間とみなす
    ss_error_mean = mean(error(ss_mask));
    ss_error_std  = std(error(ss_mask));
    integral_max_abs = max(abs(integral_term));
    ff_ss_mean = mean(ff(ss_mask));
    fb_ss_mean = mean(integral_term(ss_mask));   % 定常区間はkp*e≈0のためI(t)がFBの主成分
    saturated_ratio = mean(saturated) * 100;

    fprintf('\n--- velocity_step_%s ---\n', label);
    fprintf('定常区間(末尾0.5s)追従誤差: 平均=%.1f mm/s, 標準偏差=%.1f mm/s\n', ss_error_mean, ss_error_std);
    fprintf('積分項|I(t)|の最大値: %.3f V\n', integral_max_abs);
    fprintf('定常区間の feedforward 平均: %.3f V, 積分項(≈FB)平均: %.3f V\n', ff_ss_mean, fb_ss_mean);
    if abs(ff_ss_mean) > 1e-6
        fprintf('  → FF/(FF+FB) ≈ %.1f%%（100%%に近いほどFFだけで定常状態を賄えている）\n', ...
            100 * ff_ss_mean / (ff_ss_mean + fb_ss_mean));
    end
    fprintf('飽和していた時間の割合: %.1f%%\n', saturated_ratio);
end
