%% プラントモデルの修正（H4）と，目標角速度のレート制限（ランプ指令）の効果予測
%
% 背景：E3/E5の実機で +400/+430 に再現性のある約7%(5〜11%)のオーバーシュートが出たが，
% H1（時定数を「現在のduty」の関数とみなす）の模擬では約0%だった（rot_gain_scheduling_plan.md §11, §12）。
% H1は減速中（dutyが下がる時）に時定数も小さく見積もるため，速度を吐き出すのが速すぎる。
%
%   H1: d(w)/dt = (K(u) - w)/T(u)
%   H4: d(w)/dt = (K(u) - w)/T( max(|u|, u*(|w|)) )      u*(w)=静特性の逆写像
%
% H4は0からのstep（u>=u*(w)が常に成立）ではH1と同一で，dutyが速度に対応する値より下がった時
% だけ時定数が「速度に対応する値」に留まる（＝速度を保持しやすい）。
% 検証：(1) 実測uを入力した再生のfit%  (2) 同じ制御器の閉ループ模擬のOS(100ms平均)を実測と比較
% 予測：目標角速度を最大角加速度でレート制限（ランプ指令）した場合のOS・立上り

clear; clc;
results_dir = 'results';
datadir = '../../tools/log/omega_step_v700_x/';

pr = readtable(fullfile('..', '03_rot_identification', 'results', 'rot_step_v700_p1fit_model_params.csv'));
Kmap = @(x) sign(x) .* (pr.K_a * abs(x) + pr.K_b * abs(x) .^ pr.K_n);
Tmap = @(x) pr.T_T0 + pr.T_c * abs(x) .^ pr.T_m;
ug = linspace(0, 0.34, 3401); Kg = Kmap(ug);
ustar = @(w) interp1(Kg, ug, min(abs(w), Kg(end)), 'linear');

TI_BP_W = [0 100 200 250 400 430];
TI_BP_T = [0.0090 0.01725 0.0345 0.0420 0.0615 0.06525];
ti_of = @(w) interp1(TI_BP_W, TI_BP_T, min(abs(w), 430));

% 実機ログ {name, target, ulim(=fw)}
runs = { ...
    'pos200_1', 200, 0.26; 'neg200_1', -200, 0.26; 'pos250_1', 250, 0.26; 'neg250', -250, 0.26; ...
    'pos400', 400, 0.26; 'neg400', -400, 0.26; 'pos430', 430, 0.26; 'neg430', -430, 0.26; ...
    'pos400_1', 400, 0.28; 'pos400_2', 400, 0.28; 'pos430_1', 430, 0.28; 'pos430_2', 430, 0.28; ...
    'pos430_3', 430, 0.28; 'pos430_4', 430, 0.28};
nr = size(runs, 1);
Kc = 5e-4;

%% (1) 実測uの再生
fitH1 = zeros(nr, 1); fitH4 = zeros(nr, 1);
meas_os = zeros(nr, 1);
for r = 1:nr
    T = readtable([datadir sprintf('omega_step_%s.csv', runs{r, 1})], 'VariableNamingRule', 'modify');
    on = find(T.target_omega ~= 0, 1, 'first');
    idx = on:height(T);
    g = T.gyro_z(idx); u = T.duty_diff(idx);
    w0 = mean(T.gyro_z(on - 300:on - 1));
    yH1 = replay(u, w0, Kmap, Tmap, ustar, false);
    yH4 = replay(u, w0, Kmap, Tmap, ustar, true);
    pf = @(y) 100 * (1 - norm(g - y) / norm(g - mean(g)));
    fitH1(r) = pf(yH1); fitH4(r) = pf(yH4);
    tgt = runs{r, 2};
    meas_os(r) = max(0, (max(sign(tgt) * movmean(g, 100)) - abs(tgt)) / abs(tgt)) * 100;
end

%% (2) 閉ループ模擬：実機と同じ制御器（ステップ指令）で H1 と H4 のOSを比較
osH1 = zeros(nr, 1); osH4 = zeros(nr, 1);
for r = 1:nr
    tgt = runs{r, 2};
    [~, w1] = closed_loop(tgt, Kc, ti_of, runs{r, 3}, Kmap, Tmap, ustar, false, Inf);
    [~, w4] = closed_loop(tgt, Kc, ti_of, runs{r, 3}, Kmap, Tmap, ustar, true, Inf);
    osH1(r) = max(0, (max(sign(tgt) * movmean(w1, 100)) - abs(tgt)) / abs(tgt)) * 100;
    osH4(r) = max(0, (max(sign(tgt) * movmean(w4, 100)) - abs(tgt)) / abs(tgt)) * 100;
end

fprintf('%-9s %6s | %7s %7s | %8s %8s %8s\n', 'run', 'tgt', 'fitH1', 'fitH4', 'OS実測%', 'OS_H1%', 'OS_H4%');
for r = 1:nr
    fprintf('%-9s %6.0f | %7.1f %7.1f | %8.1f %8.1f %8.1f\n', runs{r, 1}, runs{r, 2}, fitH1(r), fitH4(r), meas_os(r), osH1(r), osH4(r));
end
fprintf('median fit: H1=%.1f  H4=%.1f\n', median(fitH1), median(fitH4));

%% (3) ランプ指令の効果予測（H4プラント）
accels = [Inf 5000 3000 2500 2000];
tg = [100 250 430 -430];
fprintf('\n===== H4プラント：目標角速度の最大角加速度制限（Inf=ステップ） =====\n');
fprintf('%6s | %s\n', 'tgt', '各行: accel[dps/s]  OS100%  90%到達[ms]  10%整定[ms]  |u|max');
rows = {};
fig = figure('Position', [50 50 1200 800]); hold on;
for it = 1:numel(tg)
    tgt = tg(it);
    for ia = 1:numel(accels)
        [ulog, wlog, wref_log] = closed_loop(tgt, Kc, ti_of, 0.28, Kmap, Tmap, ustar, true, accels(ia));
        s = sign(tgt);
        g100 = movmean(wlog, 100);
        os = max(0, (max(s * g100) - abs(tgt)) / abs(tgt)) * 100;
        r90 = find(s * wlog >= 0.9 * abs(tgt), 1, 'first');
        o10 = find(abs(g100 - tgt) > 0.1 * abs(tgt), 1, 'last');
        if isempty(o10), o10 = 0; end
        fprintf('%6d | %8.0f  %6.1f  %8d  %8d  %6.3f\n', tgt, accels(ia), os, r90, o10 + 1, max(abs(ulog)));
        rows(end + 1, :) = {tgt, accels(ia), os, r90, o10 + 1, max(abs(ulog))}; %#ok<SAGROW>
        if tgt == 430
            plot((0:numel(wlog) - 1), wlog, 'DisplayName', sprintf('accel %g', accels(ia)));
        end
    end
end
yline(430, 'k:', 'HandleVisibility', 'off'); grid on; legend('Location', 'southeast');
xlabel('t [ms]'); ylabel('\omega [dps]'); title('H4プラント +430dps：指令ランプの効果（現行Kc, Ti表）');
exportgraphics(fig, fullfile(results_dir, 'rot_plant_h4_ramp.png'));
R = cell2table(rows, 'VariableNames', {'target_dps', 'accel_dps2', 'os100_pct', 'rise90_ms', 'settle10_ms', 'u_peak'});
writetable(R, fullfile(results_dir, 'rot_plant_h4_ramp_summary.csv'));

%% ローカル関数
function y = replay(u, w0, Kmap, Tmap, ustar, h4)
    Ts = 1e-3; n = numel(u); y = zeros(n, 1); x = w0;
    for k = 1:n
        uu = abs(u(k));
        if h4, uu = max(uu, ustar(x)); end
        x = x + Ts * (Kmap(u(k)) - x) / Tmap(uu);
        y(k) = x;
    end
end

function [ulog, wlog, wref_log] = closed_loop(tgt, Kc, ti_of, ulim, Kmap, Tmap, ustar, h4, accel)
    Ts = 1e-3; N = 1500;
    w = 0; I = 0; wref = 0;
    ulog = zeros(N, 1); wlog = zeros(N, 1); wref_log = zeros(N, 1);
    for k = 1:N
        step = accel * Ts;
        wref = wref + max(min(tgt - wref, step), -step);
        Ti = ti_of(wref);
        e = wref - w;
        uu = Kc * e + I;
        u = max(min(uu, ulim), -ulim);
        I = I + (Kc / Ti * e + (u - uu) / Ti) * Ts;
        tu = abs(u);
        if h4, tu = max(tu, ustar(w)); end
        w = w + Ts * (Kmap(u) - w) / Tmap(tu);
        ulog(k) = u; wlog(k) = w; wref_log(k) = wref;
    end
end
