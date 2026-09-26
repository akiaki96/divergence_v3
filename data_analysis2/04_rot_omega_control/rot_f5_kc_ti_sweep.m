%% F5検討：+側の Kc と Ti(+430) の組合せ掃引（OSと立上りの両立）
%
% E8実測（F4: Ti(+430)=150ms, Kc=5e-4, ステップ）: OS 1.0% / 90%到達 341ms。
% モデルによる予測との比較：局所線形 T=150/200 は r90 172/128ms（速すぎ），H1 は 404ms（実測に最も近い，やや悲観）。
%   → 立上りの予測は H1（×0.84で補正），OSの予測は局所線形（T=150/200，K=4000/6000）を使う。
% F4前(Ti=65ms)の実測：OS 8.1% / r90 145〜154ms。H1は0%/147msで立上りは合う。
%
% 掃引：Kc_pos ∈ {5,7,9,12}e-4, Ti(+430) ∈ {90,110,130,150}ms（+400のTiは+430に比例, +250は60ms固定）
% 目標：OS<5%（両局所線形）かつ r90(H1×0.84) <= 260ms，かつ ripple(47Hz)の比例通過 Kc*26dps が小さいこと。
% 出力の比例通過：ジャイロの47Hzリップル(σ≈26dps@+430)が u に Kc*σ [duty] で乗る（E5で確認: 5e-4→0.013）。

clear; clc;
results_dir = 'results';
pr = readtable(fullfile('..', '03_rot_identification', 'results', 'rot_step_v700_p1fit_model_params.csv'));
Kmap = @(x) sign(x) .* (pr.K_a * abs(x) + pr.K_b * abs(x) .^ pr.K_n);
Tmap = @(x) pr.T_T0 + pr.T_c * abs(x) .^ pr.T_m;
BP_W = [0 100 200 250 400 430];
Kc_list = [5e-4 7e-4 9e-4 12e-4];
Ti_list = [0.090 0.110 0.130 0.150];
ulim = 0.28; RIPPLE_DPS = 26;

fprintf('各セル: OS[lin150]/OS[lin200] %% | r90[H1x0.84] ms | 5%%整定[H1x0.84]\n');
fprintf('%-9s %s   ripple_u\n', 'Kc', sprintf('| Ti=%3.0fms                 ', Ti_list * 1000));
rows = {};
for Kc = Kc_list
    line = sprintf('%-9.1e', Kc);
    for Ti430 = Ti_list
        tbl = [0.0090 0.01725 0.0345 0.0600 Ti430 * 140 / 150 Ti430];
        [o1, ~, ~] = run_loop(430, Kc, tbl, BP_W, ulim, 2, Kmap, Tmap);
        [o2, ~, ~] = run_loop(430, Kc, tbl, BP_W, ulim, 3, Kmap, Tmap);
        [~, r, t5] = run_loop(430, Kc, tbl, BP_W, ulim, 1, Kmap, Tmap);
        line = [line, sprintf('| %4.1f/%4.1f  %4.0f  %4.0f   ', o1, o2, 0.84 * r, 0.84 * t5)]; %#ok<AGROW>
        rows(end + 1, :) = {Kc, Ti430 * 1000, o1, o2, 0.84 * r, 0.84 * t5}; %#ok<SAGROW>
    end
    fprintf('%s  %.3f\n', line, Kc * RIPPLE_DPS);
end
R = cell2table(rows, 'VariableNames', {'Kc', 'ti430_ms', 'os_lin150', 'os_lin200', 'r90_h1_scaled_ms', 'settle5_h1_scaled_ms'});
writetable(R, fullfile(results_dir, 'rot_f5_kc_ti_sweep.csv'));

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
