%% 回転角速度PI制御 実機試験(E1)の解析 と プラントモデル仮説(H1/H2)の検証
%
% tools/log/omega_step_v700_x/omega_step_{pos200,neg200,pos250}.csv
% 並進700mm/sを閉ループで維持しつつ，config::pid_omega（純粋PI，Kc=3.7e-4, Ti=23.1ms, 上限0.20）で
% 目標角速度へステップ指令した実機ログ。
%
% 1) 追従指標（定常値・オーバーシュート・立上り・整定・飽和・積分項）
% 2) rot_omega_pi_sim.m の予測（H1プラント上の同一制御器）との重ね書き
% 3) ログされた duty_diff(t) をプラントモデルへ入力し実測gyro_zと比較（制御器に依存しない検証）
%      H1: d(w)/dt = (K(u) - w)/T(u)            0からのstepの(K,T)を現在のdutyの関数とみなす
%      H2: d(w)/dt = B(u) - B(u*(w)), B=K/T      初期加速度B(u)と，速度依存の減衰を静特性で表す
%    どちらも0からのstep応答は再現する。閉ループ中の(u,w)軌跡でどちらが合うかで局所ダイナミクスを判別する。
%    さらに機体の直進バイアス相当のdutyオフセットdeltaを1つ自由化した場合の当てはまりも見る。

clear; clc;
results_dir = 'results';

files  = {'pos200', 'neg200', 'pos250'};
nf = numel(files);

pr = readtable(fullfile('..', '03_rot_identification', 'results', 'rot_step_v700_p1fit_model_params.csv'));
mapA = pr.K_a; mapB = pr.K_b; mapN = pr.K_n; mapT0 = pr.T_T0; mapC = pr.T_c; mapM = pr.T_m;
Kmap  = @(x) sign(x) .* (mapA * abs(x) + mapB * abs(x) .^ mapN);
Tmap  = @(x) mapT0 + mapC * abs(x) .^ mapM;
Bmap  = @(x) Kmap(x) ./ Tmap(x);                       % 初期角加速度 [dps/s]
ug = linspace(0, 0.32, 3201);
Kg = Kmap(ug);
u_of_w = @(w) interp1(Kg, ug, abs(w), 'linear', ug(end)) .* sign(w);   % 静特性の逆写像

% 現行firmware（config::pid_omega）
KP_ROT = 1250; TP1 = 0.0231; LAMBDA = 0.05;
Kc0 = TP1 / (KP_ROT * LAMBDA); Ti0 = TP1; ULIM = 0.20;

smooth_n = 20;   % 20msの移動平均（指標算出用。生のノイズスパイクを過大にオーバーシュートと数えない）

M = struct('file', {}, 'target', {}, 'ss_dps', {}, 'ss_std', {}, 'u_ss', {}, 'rise90_ms', {}, 'os_pct', {}, ...
    'os_raw_pct', {}, 'settle5_ms', {}, 'sat_ms', {}, 'iss', {}, 'v_min', {});
fig1 = figure('Position', [50 50 1300 900]);
fit_tbl = zeros(nf, 5);

for f = 1:nf
    T = readtable(sprintf('../../tools/log/omega_step_v700_x/omega_step_%s.csv', files{f}), 'VariableNamingRule', 'modify');
    t = T.Global_time - T.Global_time(1);
    on = find(T.target_omega ~= 0, 1, 'first');
    tgt = T.target_omega(on);
    idx = on:height(T);
    tr = t(idx) - t(on);
    g  = T.gyro_z(idx);
    gs = movmean(g, smooth_n);
    u  = T.duty_diff(idx);
    base = mean(T.gyro_z(on - 300:on - 1));
    s = sign(tgt);

    ss = mean(g(tr >= 0.3));
    ss_std = std(g(tr >= 0.3));
    u_ss = mean(u(tr >= 0.3));
    r90 = tr(find(s * gs >= 0.9 * abs(tgt), 1, 'first')) * 1000;
    os = max(0, (max(s * gs) - abs(tgt)) / abs(tgt)) * 100;
    os_raw = max(0, (max(s * g) - abs(tgt)) / abs(tgt)) * 100;
    last_out = find(abs(gs - tgt) > 0.05 * abs(tgt), 1, 'last');
    settle = tr(last_out) * 1000;
    sat_ms = sum(T.omega_saturated(idx) > 0);
    vavg = (T.left_encoder_velocity(idx) + T.right_encoder_velocity(idx)) / 2;
    M(f) = struct('file', files{f}, 'target', tgt, 'ss_dps', ss, 'ss_std', ss_std, 'u_ss', u_ss, 'rise90_ms', r90, ...
        'os_pct', os, 'os_raw_pct', os_raw, 'settle5_ms', settle, 'sat_ms', sat_ms, 'iss', mean(T.omega_integral_term(idx(tr >= 0.3))), ...
        'v_min', min(vavg));

    % ---- 模擬(H1プラント上の現行制御器)
    Ts = 1e-3; N = numel(idx);
    w = 0; I = 0; wprev = 0; wsim = zeros(N, 1); usim = zeros(N, 1);
    for k = 1:N
        e = tgt - wprev;
        uu = Kc0 * e + I;
        uc = max(min(uu, ULIM), -ULIM);
        I = I + (Kc0 / Ti0 * e + (uc - uu) / Ti0) * Ts;
        w = w + Ts * (Kmap(uc) - w) / Tmap(uc);
        wprev = w; wsim(k) = w; usim(k) = uc;
    end

    % ---- 実測uをモデルへ入力（開ループ再生）
    w0 = base;
    yH1 = replay_h1(u, w0, Kmap, Tmap, 0);
    yH2 = replay_h2(u, w0, Bmap, u_of_w, 0);
    pct = @(y, yh) 100 * (1 - norm(y - yh) / norm(y - mean(y)));
    % dutyオフセットdelta（機体の直進バイアス/摩擦のrun間差に相当）を各モデルで最適化
    dH1 = fminsearch(@(d) norm(g - replay_h1(u, w0, Kmap, Tmap, d)), 0, optimset('TolX', 1e-6));
    dH2 = fminsearch(@(d) norm(g - replay_h2(u, w0, Bmap, u_of_w, d)), 0, optimset('TolX', 1e-6));
    yH1d = replay_h1(u, w0, Kmap, Tmap, dH1);
    yH2d = replay_h2(u, w0, Bmap, u_of_w, dH2);
    fit_tbl(f, :) = [pct(g, yH1), pct(g, yH2), dH1, pct(g, yH1d), pct(g, yH2d)];
    fit_tbl_d2(f) = dH2; %#ok<SAGROW>

    subplot(3, 3, f);
    plot(tr * 1000, g, 'Color', [0.7 0.7 0.7], 'DisplayName', '実測(raw)'); hold on;
    plot(tr * 1000, gs, 'k', 'LineWidth', 1.3, 'DisplayName', '実測(20ms平均)');
    plot(tr * 1000, wsim, 'r--', 'LineWidth', 1.2, 'DisplayName', '模擬(H1,現行制御器)');
    yline(tgt, 'b:', 'HandleVisibility', 'off');
    title(sprintf('%s: 追従', files{f})); xlabel('t [ms]'); ylabel('\omega [dps]'); grid on; legend('Location', 'southeast', 'FontSize', 7);

    subplot(3, 3, 3 + f);
    plot(tr * 1000, u, 'k', 'DisplayName', '実測'); hold on;
    plot(tr * 1000, usim, 'r--', 'DisplayName', '模擬');
    yline(s * ULIM, 'b:', 'HandleVisibility', 'off');
    title(sprintf('%s: duty\\_diff', files{f})); xlabel('t [ms]'); ylabel('u'); grid on; legend('Location', 'southeast', 'FontSize', 7);

    subplot(3, 3, 6 + f);
    plot(tr * 1000, g, 'Color', [0.8 0.8 0.8], 'DisplayName', '実測'); hold on;
    plot(tr * 1000, yH1, 'r', 'DisplayName', sprintf('H1 %.0f%%', fit_tbl(f, 1)));
    plot(tr * 1000, yH2, 'b', 'DisplayName', sprintf('H2 %.0f%%', fit_tbl(f, 2)));
    title(sprintf('%s: 実測uを入力した再生', files{f})); xlabel('t [ms]'); ylabel('\omega [dps]'); grid on; legend('Location', 'southeast', 'FontSize', 7);
end
exportgraphics(fig1, fullfile(results_dir, 'omega_step_e1.png'));
savefig(fig1, fullfile(results_dir, 'omega_step_e1.fig'));

%% 0からのstep(open loop)でのH1/H2の再現性（実データ20水準）
labels    = {'pos002', 'pos004', 'pos006', 'pos010', 'pos014', 'pos020', 'pos022_long', 'pos024_long', 'pos026_long', 'pos028', ...
             'neg002', 'neg004', 'neg006', 'neg010', 'neg014', 'neg020', 'neg022_long', 'neg024_long', 'neg026_long', 'neg028'};
duty_vals = [0.02 0.04 0.06 0.10 0.14 0.20 0.22 0.24 0.26 0.28, -0.02 -0.04 -0.06 -0.10 -0.14 -0.20 -0.22 -0.24 -0.26 -0.28];
step_fit = zeros(numel(labels), 2);
for i = 1:numel(labels)
    T = readtable(sprintf('../../tools/log/rot_step_v700_x/rot_step_v700_duty_%s.csv', labels{i}), 'VariableNamingRule', 'modify');
    t = T.Global_time - T.Global_time(1);
    [onset, offset] = find_longest_nonzero_run(T.duty_diff);
    y0 = mean(T.gyro_z(max(1, onset - 300):onset - 1));
    fe = min(offset - 1, onset + 799);
    y = T.gyro_z(onset:fe) - y0;
    uu = duty_vals(i) * ones(numel(y), 1);
    p = @(y, yh) 100 * (1 - norm(y - yh) / norm(y - mean(y)));
    step_fit(i, :) = [p(y, replay_h1(uu, 0, Kmap, Tmap, 0)), p(y, replay_h2(uu, 0, Bmap, u_of_w, 0))];
end

%% 表示・保存
fprintf('===== E1 追従指標（現行firmware, 上限0.20） =====\n');
fprintf('%-7s %7s | %9s %6s | %8s | %8s %7s %8s %9s | %7s %7s\n', 'file', 'target', 'ss[dps]', 'std', 'u_ss', 'rise90ms', 'OS%', 'OSraw%', 'settle5ms', 'sat_ms', 'vmin');
for f = 1:nf
    m = M(f);
    fprintf('%-7s %7.0f | %9.1f %6.1f | %8.4f | %8.0f %7.1f %8.1f %9.0f | %7d %7.0f\n', m.file, m.target, m.ss_dps, m.ss_std, m.u_ss, ...
        m.rise90_ms, m.os_pct, m.os_raw_pct, m.settle5_ms, m.sat_ms, m.v_min);
end
fprintf('\n===== 実測uをプラントモデルに入力したときの当てはまり(fit%%) =====\n');
fprintf('%-7s | %8s %8s | %10s %8s | %10s %8s\n', 'file', 'H1', 'H2', 'H1 delta', 'H1+d', 'H2 delta', 'H2+d');
for f = 1:nf
    fprintf('%-7s | %8.1f %8.1f | %10.4f %8.1f | %10.4f %8.1f\n', files{f}, fit_tbl(f, 1), fit_tbl(f, 2), fit_tbl(f, 3), fit_tbl(f, 4), fit_tbl_d2(f), fit_tbl(f, 5));
end
fprintf('\n===== 0からのstep20水準での再現性(fit%%)：中央値 H1=%.1f  H2=%.1f =====\n', median(step_fit(:, 1)), median(step_fit(:, 2)));
fprintf('大振幅(|u|>=0.20)のみ：H1=%.1f  H2=%.1f\n', median(step_fit(abs(duty_vals) >= 0.20, 1)), median(step_fit(abs(duty_vals) >= 0.20, 2)));

Sm = struct2table(M);
writetable(Sm, fullfile(results_dir, 'omega_step_e1_summary.csv'));
Fm = table(files', fit_tbl(:, 1), fit_tbl(:, 2), fit_tbl(:, 3), fit_tbl(:, 4), fit_tbl_d2', fit_tbl(:, 5), ...
    'VariableNames', {'file', 'H1_fit', 'H2_fit', 'H1_delta', 'H1_delta_fit', 'H2_delta', 'H2_delta_fit'});
writetable(Fm, fullfile(results_dir, 'omega_step_e1_model_replay.csv'));

%% ローカル関数
function [onset, offset] = find_longest_nonzero_run(duty_diff)
    is_nonzero = duty_diff ~= 0;
    d = diff([0; is_nonzero; 0]);
    run_starts = find(d == 1);
    run_ends   = find(d == -1) - 1;
    [~, idx] = max(run_ends - run_starts + 1);
    onset  = run_starts(idx);
    offset = run_ends(idx) + 1;
end

function w = replay_h1(u, w0, Kmap, Tmap, delta)
    Ts = 1e-3; n = numel(u); w = zeros(n, 1); x = w0;
    for k = 1:n
        uk = u(k) + delta;
        x = x + Ts * (Kmap(uk) - x) / Tmap(uk);
        w(k) = x;
    end
end

function w = replay_h2(u, w0, Bmap, u_of_w, delta)
    Ts = 1e-3; n = numel(u); w = zeros(n, 1); x = w0;
    for k = 1:n
        uk = u(k) + delta;
        x = x + Ts * (Bmap(uk) - Bmap(u_of_w(x)));
        w(k) = x;
    end
end
