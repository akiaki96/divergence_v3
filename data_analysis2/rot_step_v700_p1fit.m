%% 回転方向 動作点別ステップ応答フィット（P1D / P2）とゲインスケジューリング検討
%
% tools/log/rot_step_v700_x/ の20水準（±0.02〜±0.28）について，励振窓のgyro_z応答を
%   P1D: y = K*(1-exp(-(t-Td)/T))            （一次遅れ+むだ時間）
%   P2 : 2次系（過減衰/不足減衰を自動判別）+むだ時間
% でフィットし，動作点（duty_diff）ごとの K, T, 適合度を求める。
% 目的：回転PI制御(config::pid_omega)の設計前提（KP_ROT=1250, TP1_ROT=23ms）が
% 運用域(430dps付近, duty_diff≈0.22〜0.24)で成り立つかを検証し，P1が破綻していれば
% ゲインスケジューリングの根拠（K(duty_diff), T(duty_diff)）を得る。
%
% 注意：割線ゲイン K/duty_diff（ステップ全体の大きさ効果）と，接線ゲイン dK/dduty_diff
% （動作点まわりの微小変化に対する感度＝閉ループ安定性を決める実効ループゲイン）は
% 非線形性（不感帯・逆転域）が強い領域で大きく異なる。制御設計に効くのは後者。

clear; clc;

results_dir = 'results';
if ~exist(results_dir, 'dir'), mkdir(results_dir); end

labels    = {'pos002', 'pos004', 'pos006', 'pos010', 'pos014', 'pos020', 'pos022_long', 'pos024_long', 'pos026_long', 'pos028', ...
             'neg002', 'neg004', 'neg006', 'neg010', 'neg014', 'neg020', 'neg022_long', 'neg024_long', 'neg026_long', 'neg028'};
duty_vals = [0.02, 0.04, 0.06, 0.10, 0.14, 0.20, 0.22, 0.24, 0.26, 0.28, ...
             -0.02, -0.04, -0.06, -0.10, -0.14, -0.20, -0.22, -0.24, -0.26, -0.28];
n = numel(labels);

FIT_MS      = 800;   % フィットに使う励振開始後の区間 [ms]（600ms試行は窓末尾まで）
BASELINE_MS = 300;   % 励振直前のゼロ基準区間 [ms]（PRBS解析と同じ方針）

K1 = zeros(n,1); T1 = zeros(n,1); Td1 = zeros(n,1); fit1 = zeros(n,1);
K2 = zeros(n,1); wn2 = zeros(n,1); z2 = zeros(n,1); Td2 = zeros(n,1); fit2 = zeros(n,1);
gyro_ss = zeros(n,1);
base_dps = zeros(n,1);
data_t  = cell(n,1); data_y = cell(n,1);

for i = 1:n
    fname = sprintf('../tools/log/rot_step_v700_x/rot_step_v700_duty_%s.csv', labels{i});
    T = readtable(fname, 'VariableNamingRule', 'modify');
    t = T.Global_time - T.Global_time(1);
    [onset, offset] = find_longest_nonzero_run(T.duty_diff);

    base_idx = max(1, onset - BASELINE_MS):onset - 1;
    y0 = mean(T.gyro_z(base_idx));
    base_dps(i) = y0;

    fit_end = min(offset - 1, onset + FIT_MS - 1);
    tt = t(onset:fit_end) - t(onset);
    yy = T.gyro_z(onset:fit_end) - y0;
    data_t{i} = tt; data_y{i} = yy;

    n_tail = max(20, round(numel(yy) * 0.2));
    gyro_ss(i) = mean(yy(end - n_tail + 1:end));

    [K1(i), T1(i), Td1(i), fit1(i)] = fit_p1d(tt, yy);
    [K2(i), wn2(i), z2(i), Td2(i), fit2(i)] = fit_p2(tt, yy);

    fprintf('%-12s u=%+.2f  yss=%+7.1f | P1D: K=%+7.1f T=%5.1fms Td=%4.1fms fit=%5.1f%% | P2: wn=%6.1f z=%5.2f fit=%5.1f%%\n', ...
        labels{i}, duty_vals(i), gyro_ss(i), K1(i), T1(i)*1000, Td1(i)*1000, fit1(i), wn2(i), z2(i), fit2(i));
end

%% 割線ゲイン・接線ゲイン
u = duty_vals(:);
Ksec = K1 ./ u;                      % 割線ゲイン [dps/duty]
Ktan = zeros(n, 1);                  % 接線ゲイン（隣接水準間の傾き，原点(0,0)を含む）
for sgn = [1, -1]
    idx = find(sign(u) == sgn);
    [~, ord] = sort(abs(u(idx)));
    idx = idx(ord);
    uu = [0; u(idx)];
    yy_ss = [0; K1(idx)];
    g = diff(yy_ss) ./ diff(uu);     % 区間 [uu(k), uu(k+1)] の傾き
    Ktan(idx) = g;                   % 各水準に「その水準へ至る区間の傾き」を割り当てる
end

%% 結果表示
fprintf('\n===== 動作点別サマリ（P1D） =====\n');
fprintf('%-12s %8s %10s %10s %9s %8s %8s\n', 'label', 'duty', 'K[dps]', 'Ksec', 'Ktan', 'T[ms]', 'fit%');
for i = 1:n
    fprintf('%-12s %+8.2f %+10.1f %10.1f %9.1f %8.1f %8.1f\n', ...
        labels{i}, u(i), K1(i), Ksec(i), Ktan(i), T1(i)*1000, fit1(i));
end

% 初期角加速度 K/T [dps/s]：step直後の傾き。ゲインKと時定数Tは振幅で大きく変わるが，
% その比は変化が小さい（速度制御の応答性はこの量で決まる）
alpha0 = K1 ./ T1;
fprintf('\n===== 初期角加速度 K/T [dps/s]（目標運用: 2500dps/s^2） =====\n');
for i = 1:n
    fprintf('%-12s u=%+.2f  K/T=%+8.0f dps/s   (K/(T*u)=%7.0f dps/s/duty)\n', labels{i}, u(i), alpha0(i), alpha0(i) / u(i));
end

%% 平滑化モデル（点推定のバラつきを除いた接線ゲイン・時定数の取得用）
% 静特性: K(u) = sign(u)*(a|u| + b|u|^nexp)  （線形域+不感帯突入後の急増を表す2項）
% 時定数: T(u) = T0 + c|u|^m
absu = abs(u); sg = sign(u);
w = 1 ./ max(abs(K1), 20);
objK = @(p) sum((w .* (K1 - sg .* (p(1) * absu + exp(p(2)) * absu .^ p(3)))) .^ 2);
pK = fminsearch(objK, [750, log(5e5), 5], optimset('MaxFunEvals', 20000, 'MaxIter', 20000, 'TolX', 1e-9, 'TolFun', 1e-12));
mapA = pK(1); mapB = exp(pK(2)); mapN = pK(3);
Kmap  = @(x) sign(x) .* (mapA * abs(x) + mapB * abs(x) .^ mapN);
Ktanf = @(x) mapA + mapN * mapB * abs(x) .^ (mapN - 1);

wT = 1 ./ T1;
objT = @(p) sum((wT .* (T1 - (exp(p(1)) + exp(p(2)) * absu .^ p(3)))) .^ 2);
pT = fminsearch(objT, [log(0.012), log(20), 3], optimset('MaxFunEvals', 20000, 'MaxIter', 20000, 'TolX', 1e-9, 'TolFun', 1e-12));
mapT0 = exp(pT(1)); mapC = exp(pT(2)); mapM = pT(3);
Tmap = @(x) mapT0 + mapC * abs(x) .^ mapM;

fprintf('\n===== 平滑化モデル =====\n');
fprintf('K(u)  = sign(u)*(%.1f|u| + %.3g|u|^%.2f)  [dps]\n', mapA, mapB, mapN);
fprintf('T(u)  = %.4f + %.3g|u|^%.2f  [s]\n', mapT0, mapC, mapM);
fprintf('%-8s %10s %10s %10s %10s %14s\n', '|u|', 'K(u)', 'Ksec', 'Ktan', 'T[ms]', 'Ktan/T[dps/s/duty]');
for uu = [0.02 0.04 0.06 0.10 0.14 0.20 0.22 0.24 0.26 0.28]
    fprintf('%-8.2f %10.1f %10.1f %10.1f %10.1f %14.0f\n', uu, Kmap(uu), Kmap(uu) / uu, Ktanf(uu), Tmap(uu) * 1000, Ktanf(uu) / Tmap(uu));
end
res_K = K1 - Kmap(u);
fprintf('K(u)モデル残差RMS: %.1f dps（|K|平均%.0f dps）\n', sqrt(mean(res_K .^ 2)), mean(abs(K1)));

summary = table(labels', u, base_dps, gyro_ss, K1, Ksec, Ktan, T1*1000, Td1*1000, fit1, alpha0, ...
    wn2, z2, Td2*1000, fit2, ...
    'VariableNames', {'label', 'duty_diff', 'baseline_dps', 'gyro_ss_tail_dps', 'P1D_K_dps', 'Ksec_dps_per_duty', ...
    'Ktan_dps_per_duty', 'P1D_T_ms', 'P1D_Td_ms', 'P1D_fit_pct', 'K_over_T_dps_per_s', 'P2_wn_rad_s', 'P2_zeta', 'P2_Td_ms', 'P2_fit_pct'});
writetable(summary, fullfile(results_dir, 'rot_step_v700_p1fit_summary.csv'));
model_params = table(mapA, mapB, mapN, mapT0, mapC, mapM, ...
    'VariableNames', {'K_a', 'K_b', 'K_n', 'T_T0', 'T_c', 'T_m'});
writetable(model_params, fullfile(results_dir, 'rot_step_v700_p1fit_model_params.csv'));

%% 図1: 代表水準のフィット重ね書き
fig1 = figure('Position', [50 50 1200 800]);
show = {'pos006', 'pos014', 'pos020', 'pos022_long', 'pos024_long', 'pos026_long', 'neg022_long', 'neg024_long', 'pos028'};
for k = 1:numel(show)
    i = find(strcmp(labels, show{k}));
    subplot(3, 3, k);
    plot(data_t{i} * 1000, data_y{i}, 'k-', 'DisplayName', '実測'); hold on;
    tf = data_t{i};
    plot(tf * 1000, p1d_model(tf, K1(i), T1(i), Td1(i)), 'r--', 'LineWidth', 1.2, 'DisplayName', sprintf('P1D %.0f%%', fit1(i)));
    plot(tf * 1000, p2_model(tf, K2(i), wn2(i), z2(i), Td2(i)), 'b-.', 'LineWidth', 1.2, 'DisplayName', sprintf('P2 %.0f%%', fit2(i)));
    title(sprintf('%s (u=%+.2f)', strrep(labels{i}, '_', '\_'), u(i)));
    xlabel('t [ms]'); ylabel('\Delta gyro\_z [dps]'); grid on; legend('Location', 'southeast', 'FontSize', 7);
end
exportgraphics(fig1, fullfile(results_dir, 'rot_step_v700_p1fit_overlay.png'));
savefig(fig1, fullfile(results_dir, 'rot_step_v700_p1fit_overlay.fig'));

%% 図2: 動作点依存（K, T, fit）
fig2 = figure('Position', [50 50 1000 1100]);
[us, ord] = sort(u);
ug = linspace(-0.29, 0.29, 400);
subplot(4, 1, 1);
plot(us, Ksec(ord), 'o', 'DisplayName', '割線 K/u（点推定）'); hold on;
plot(us, Ktan(ord), 's', 'DisplayName', '接線 差分（点推定，ノイズ大）');
plot(ug, Kmap(ug) ./ ug, '-', 'DisplayName', '割線（平滑化モデル）');
plot(ug, Ktanf(ug), '-', 'LineWidth', 1.5, 'DisplayName', '接線（平滑化モデル）');
yline(1250, 'r--', 'KP\_ROT=1250', 'HandleVisibility', 'off');
ylim([0 12000]);
xlabel('duty\_diff'); ylabel('gain [dps/duty]'); legend('Location', 'north', 'FontSize', 7); grid on;
title('動作点別ゲイン');
subplot(4, 1, 2);
plot(us, T1(ord) * 1000, 'o'); hold on;
plot(ug, Tmap(ug) * 1000, '-');
yline(23.1, 'r--', 'TP1\_ROT=23.1ms', 'HandleVisibility', 'off');
xlabel('duty\_diff'); ylabel('T (P1D) [ms]'); grid on; title('動作点別時定数');
subplot(4, 1, 3);
plot(us, alpha0(ord), 'o-'); hold on;
yline(2500, 'r--', '目標 2500 dps/s', 'HandleVisibility', 'off');
xlabel('duty\_diff'); ylabel('K/T [dps/s]'); grid on; title('初期角加速度（step直後の傾き）');
subplot(4, 1, 4);
plot(us, fit1(ord), 'o-', 'DisplayName', 'P1D'); hold on;
plot(us, fit2(ord), 's-', 'DisplayName', 'P2');
xlabel('duty\_diff'); ylabel('fit [%]'); legend('Location', 'southwest'); grid on; title('フィット適合度');
exportgraphics(fig2, fullfile(results_dir, 'rot_step_v700_p1fit_gain_schedule.png'));
savefig(fig2, fullfile(results_dir, 'rot_step_v700_p1fit_gain_schedule.fig'));

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

function y = p1d_model(t, K, T, Td)
    tt = max(t - Td, 0);
    y = K * (1 - exp(-tt / T));
end

function y = p2_model(t, K, wn, z, Td)
    tt = max(t - Td, 0);
    if abs(z - 1) < 1e-4, z = 1 + 1e-4; end
    r = sqrt(complex(z^2 - 1));
    s1 = wn * (-z + r);
    s2 = wn * (-z - r);
    y = K * real(1 + (s2 * exp(s1 * tt) - s1 * exp(s2 * tt)) / (s1 - s2));
end

function f = fit_pct(y, yhat)
    f = 100 * (1 - norm(y - yhat) / norm(y - mean(y)));
end

function [K, T, Td, fit] = fit_p1d(t, y)
    K0 = mean(y(end - max(20, round(numel(y) * 0.2)) + 1:end));
    best = inf; bp = [];
    for T0 = [0.02, 0.06, 0.15]
        p0 = [K0, log(T0), log(0.003)];
        obj = @(p) sum((y - p1d_model(t, p(1), exp(p(2)), exp(p(3)))).^2);
        [p, fv] = fminsearch(obj, p0, optimset('MaxFunEvals', 4000, 'MaxIter', 4000, 'TolX', 1e-8, 'TolFun', 1e-8));
        if fv < best, best = fv; bp = p; end
    end
    K = bp(1); T = exp(bp(2)); Td = exp(bp(3));
    fit = fit_pct(y, p1d_model(t, K, T, Td));
end

function [K, wn, z, Td, fit] = fit_p2(t, y)
    K0 = mean(y(end - max(20, round(numel(y) * 0.2)) + 1:end));
    best = inf; bp = [];
    for wn0 = [10, 30, 80]
        for z0 = [0.5, 1.5]
            p0 = [K0, log(wn0), log(z0), log(0.003)];
            obj = @(p) sum((y - p2_model(t, p(1), exp(p(2)), exp(p(3)), exp(p(4)))).^2);
            [p, fv] = fminsearch(obj, p0, optimset('MaxFunEvals', 6000, 'MaxIter', 6000, 'TolX', 1e-8, 'TolFun', 1e-8));
            if fv < best, best = fv; bp = p; end
        end
    end
    K = bp(1); wn = exp(bp(2)); z = exp(bp(3)); Td = exp(bp(4));
    fit = fit_pct(y, p2_model(t, K, wn, z, Td));
end
