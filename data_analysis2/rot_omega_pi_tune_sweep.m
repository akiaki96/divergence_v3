%% 回転角速度PIのファーム実装案の選定：Kc一定 + Ti表(倍率)のスイープ
%
% rot_omega_pi_sim.m と同じプラント(H1)・制御器構造。Ti表は rot_gain_scheduling_plan.md §6 の値に倍率を掛ける。
% 評価点：目標100/200/250/400/430dps，ゲイン倍率1.0/2.0，測定ノイズ15dps，上限0.26。
% 実機E1で検証済みの現行(Kc=3.7e-4, Ti=23.1ms固定)を基準として併記する。

clear; clc;
results_dir = 'results';
pr = readtable(fullfile(results_dir, 'rot_step_v700_p1fit_model_params.csv'));
Kmap  = @(x) sign(x) .* (pr.K_a * abs(x) + pr.K_b * abs(x) .^ pr.K_n);
Tmap  = @(x) pr.T_T0 + pr.T_c * abs(x) .^ pr.T_m;

TI_BP_W = [0 100 200 250 400 430];
TI_BP_T = [0.012 0.023 0.046 0.056 0.082 0.087];
targets = [100 200 250 400 430];
gms = [1.0 2.0];
Kcs = [3.7e-4 5e-4 7e-4 1e-3];
scales = [0.5 0.75 1.0];

fprintf('%-9s %-6s | %s\n', 'Kc', 'Tiscl', '目標:  OS%% / rise90ms / settle5%%ms   (ゲイン1.0 | 2.0の最悪値)');
rows = {};
% 基準：現行
cfgs = {{3.7e-4, 'fixed', 0.0231}};
for Kc = Kcs
    for sc = scales
        cfgs{end + 1} = {Kc, 'table', sc}; %#ok<SAGROW>
    end
end
for c = 1:numel(cfgs)
    Kc = cfgs{c}{1}; kind = cfgs{c}{2}; sc = cfgs{c}{3};
    line = '';
    for wref = targets
        os = zeros(1, 2); rise = zeros(1, 2); settle = zeros(1, 2);
        for ig = 1:2
            if strcmp(kind, 'fixed'), Ti = sc; else, Ti = sc * interp1(TI_BP_W, TI_BP_T, min(wref, 430)); end
            [os(ig), rise(ig), settle(ig)] = run_loop(wref, Kc, Ti, 0.26, gms(ig), Kmap, Tmap);
        end
        line = [line, sprintf(' | %3d: %4.1f/%3.0f/%4.0f', wref, max(os), max(rise), max(settle))]; %#ok<AGROW>
    end
    if strcmp(kind, 'fixed'), tag = '現行固定Ti23ms'; else, tag = sprintf('Ti表x%.2f', sc); end
    fprintf('%-9.1e %-14s%s\n', Kc, tag, line);
end

function [os, tr, ts] = run_loop(wref, Kc, Ti, ulim, gm, Kmap, Tmap)
    rng(1);
    Ts = 1e-3; N = 1500; sigma = 15;
    w = 0; I = 0; wprev = 0; wlog = zeros(N, 1); noise = sigma * randn(N, 1);
    for k = 1:N
        e = wref - wprev;
        uu = Kc * e + I;
        uc = max(min(uu, ulim), -ulim);
        I = I + (Kc / Ti * e + (uc - uu) / Ti) * Ts;
        w = w + Ts * (gm * Kmap(uc) - w) / Tmap(uc);
        wprev = w + noise(k);
        wlog(k) = w;
    end
    t = (0:N - 1)' * Ts;
    os = max(0, (max(wlog) - wref) / wref) * 100;
    r = find(wlog >= 0.9 * wref, 1, 'first');
    if isempty(r), tr = NaN; else, tr = t(r) * 1000; end
    o = find(abs(wlog - wref) > 0.05 * wref, 1, 'last');
    if isempty(o), ts = 0; elseif o >= N, ts = NaN; else, ts = t(o + 1) * 1000; end
end
