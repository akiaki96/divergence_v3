%% F4（+側のTi延長）のコストと効果：Ti(+430)を振って，2種類のプラントで OS と立上りを見る
%
% プラントA: H1（0からのstep実測由来。+430で必要なu≈0.233，T(u)≈85ms）。OSは原理的に出ない（§12）が，
%            積分の立ち上がり（=立上り時間）のコストを見るには使える（悲観側：必要な積分が実機より大きい）。
% プラントB/C: 局所線形 T_loc=150ms/K=4000, T_loc=200ms/K=6000（実機E7の+側のOSを再現する規模，§14.2）。
% Ti表：+400のTiは +430 の Ti に比例（現行F4案 140:150），+250は60ms固定。指令はステップ（1kHz，ノイズなし）。

clear; clc;
results_dir = 'results';
pr = readtable(fullfile('..', '03_rot_identification', 'results', 'rot_step_v700_p1fit_model_params.csv'));
Kmap = @(x) sign(x) .* (pr.K_a * abs(x) + pr.K_b * abs(x) .^ pr.K_n);
Tmap = @(x) pr.T_T0 + pr.T_c * abs(x) .^ pr.T_m;

BP_W = [0 100 200 250 400 430];
Kc = 5e-4; ulim = 0.28;
ti430_list = [0.065 0.090 0.110 0.130 0.150];
fprintf('%-22s | %s\n', 'Ti(+430) [ms]', sprintf('%14.0f', ti430_list * 1000));
names = {'A H1(T≈85ms)', 'B 線形 T=150 K=4000', 'C 線形 T=200 K=6000'};
rows = {};
for pi_ = 1:3
    line_os = ''; line_r = '';
    for ti430 = ti430_list
        tbl = [0.0090 0.01725 0.0345 0.0600 ti430 * 140 / 150 ti430];
        [os, r90, ts] = run_loop(430, Kc, tbl, BP_W, ulim, pi_, Kmap, Tmap);
        line_os = [line_os, sprintf('%6.1f/%4.0f/%4.0f ', os, r90, ts)]; %#ok<AGROW>
        rows(end + 1, :) = {names{pi_}, ti430 * 1000, os, r90, ts}; %#ok<SAGROW>
    end
    fprintf('%-22s | %s\n', names{pi_}, line_os);
end
fprintf('（各セル: OS%% / 90%%到達[ms] / 5%%整定[ms]）\n');
R = cell2table(rows, 'VariableNames', {'plant', 'ti430_ms', 'os_pct', 'r90_ms', 'settle5_ms'});
writetable(R, fullfile(results_dir, 'rot_f4_ti_sweep.csv'));

function [os, r90, ts] = run_loop(tgt, Kc, tbl, BP_W, ulim, plant, Kmap, Tmap)
    Ts = 1e-3; N = 1500;
    w = 0; I = 0; wprev = 0; wl = zeros(N, 1);
    Ti = interp1(BP_W, tbl, min(abs(tgt), 430));
    switch plant
        case 1, Kl = 0; Tl = 0;
        case 2, Kl = 4000; Tl = 0.150;
        case 3, Kl = 6000; Tl = 0.200;
    end
    for k = 1:N
        e = tgt - wprev;
        uu = Kc * e + I;
        u = max(min(uu, ulim), -ulim);
        I = I + (Kc / Ti * e + (u - uu) / Ti) * Ts;
        if plant == 1
            w = w + Ts * (Kmap(u) - w) / Tmap(u);
        else
            w = w + Ts * (Kl * u - w) / Tl;
        end
        wprev = w; wl(k) = w;
    end
    os = max(0, (max(wl) - tgt) / tgt) * 100;
    r = find(wl >= 0.9 * tgt, 1, 'first'); if isempty(r), r90 = NaN; else, r90 = r; end
    o = find(abs(wl - tgt) > 0.05 * tgt, 1, 'last'); if isempty(o), ts = 0; else, ts = o + 1; end
end
