%% +側高速(+430)の追従問題に対する 2自由度（FF付き）制御の効果の試算
%
% 簡易プラント族（不感帯付きアフィン＋一次遅れ）:  dω/dt = (K*max(u - u0, 0) - ω) / T
%   u_ss = u0 + 430/K  … 実機の+430で観測された必要duty 0.176〜0.222（E5〜E8）に対応
%   K, T : 局所ゲイン・時定数（E7の局所回帰: K_loc 4000〜6000, T_loc 150〜300ms）
% 手順：
%  (1) この族のうち，実機の現行PI・F4の実測（ステップ+430）と整合するプラントを選ぶ（較正）
%      実測: 現行PI(Ti=65ms) OS 4〜12%/r90 133〜163ms，F3ランプ OS 10〜14%/r90 193ms，
%            F4(Ti=150ms) ステップ OS 0〜3%/r90 333〜357ms，F4ランプ OS 0.3〜10%/r90 247〜285ms
%  (2) 較正済みの族に対して，(a)現行PI (b)F4 (c)FF付き2自由度 を比較する（指令：ステップ，ランプ2500）
% FF: u_ff = u0_ff + ω_ref/K_ff（静的）＋ (T_ff/K_ff)*d(ω_ref)/dt（加速度FF）。PIは偏差の補正に回す。
% 注意：これは構造の見積りで，実機の非線形（低速域の低ゲイン・速い時定数など）は含まない。

clear; clc;
results_dir = 'results';
ULIM = 0.28; TGT = 430; Kc = 5e-4;

Kl = [3000 4500 6000]; Tl = [0.10 0.15 0.20 0.30]; uss = [0.176 0.200 0.222];
grid = [];
for K = Kl
    for T = Tl
        for us = uss
            grid = [grid; K T us]; %#ok<AGROW>
        end
    end
end

%% (1) 較正
fprintf('===== (1) 較正：ステップ+430の応答（現行PI Ti=65 / F4 Ti=150） =====\n');
fprintf('%5s %5s %6s | %-22s | %-22s\n', 'K', 'T', 'u_ss', 'PI65: OS / r90', 'F4(150): OS / r90');
keep = false(size(grid, 1), 1);
for i = 1:size(grid, 1)
    p = plant_of(grid(i, :), TGT);
    m1 = run_ctrl(p, Kc, 0.065, [], 'step', 0);
    m2 = run_ctrl(p, Kc, 0.150, [], 'step', 0);
    % 実測と整合: PI65のr90が120〜180ms，F4のr90が280〜400ms
    ok = m1.r90 >= 120 && m1.r90 <= 180 && m2.r90 >= 280 && m2.r90 <= 400;
    keep(i) = ok;
    if ok
        fprintf('%5.0f %5.0f %6.3f | %5.1f%% / %4.0f ms        | %5.1f%% / %4.0f ms\n', grid(i, 1), grid(i, 2) * 1000, grid(i, 3), m1.os, m1.r90, m2.os, m2.r90);
    end
end
G = grid(keep, :);
fprintf('較正に合致したプラント: %d / %d\n', size(G, 1), size(grid, 1));
if isempty(G), error('較正に合致するプラントがありません'); end

%% (2) 制御器の比較
% FFの設計値（名目）: 感度状態の中間 u_ss=0.20 を想定，K_ff, T_ff は較正済み族の中央値
Kff = median(G(:, 1)); Tff = median(G(:, 2));
u0_mid = 0.200 - TGT / Kff;          % 名目（中間感度）
u0_low = 0.176 - TGT / Kff;          % 保守側（最も高感度＝FFが最も小さい）
fprintf('\nFF名目: K_ff=%.0f, T_ff=%.0fms, u0_ff(名目)=%.3f, u0_ff(保守)=%.3f\n', Kff, Tff * 1000, u0_mid, u0_low);

ctrls = { ...
    'PI Ti=65 (現行)',              0.065, [],                      false; ...
    'PI Ti=150 (F4)',               0.150, [],                      false; ...
    'FF静的(名目)+PI Ti=65',        0.065, [u0_mid, Kff, Tff],      false; ...
    'FF静的(保守)+PI Ti=65',        0.065, [u0_low, Kff, Tff],      false; ...
    'FF静的+加速度(名目)+PI Ti=65', 0.065, [u0_mid, Kff, Tff],      true; ...
    'FF静的+加速度(保守)+PI Ti=65', 0.065, [u0_low, Kff, Tff],      true; ...
    'FF静的+加速度(保守)+PI Ti=100', 0.100, [u0_low, Kff, Tff],     true; ...
    % --- FF設計値の誤差に対する頑健性（名目FFをずらす） ---
    'FF誤差: T/K x0.5 (加速度FF不足)',  0.065, [u0_mid, Kff, Tff * 0.5],      true; ...
    'FF誤差: T/K x1.5 (加速度FF過大)',  0.065, [u0_mid, Kff, Tff * 1.5],      true; ...
    'FF誤差: 必要duty -0.03 (不足)',    0.065, [u0_mid - 0.03, Kff, Tff],     true; ...
    'FF誤差: 必要duty +0.03 (過大)',    0.065, [u0_mid + 0.03, Kff, Tff],     true};
modes = {'step', 'ramp'};
rows = {};
for mi = 1:2
    fprintf('\n--- 指令: %s（較正済み %d プラントの 平均 / 最悪） ---\n', modes{mi}, size(G, 1));
    fprintf('%-32s | %-14s | %-16s | %-14s\n', '制御器', 'OS% 平均/最悪', '90%到達ms 平均/最悪', 'ランプ遅れ%');
    for ci = 1:size(ctrls, 1)
        os = zeros(size(G, 1), 1); r90 = os; lag = os; ss = os;
        for i = 1:size(G, 1)
            p = plant_of(G(i, :), TGT);
            m = run_ctrl(p, Kc, ctrls{ci, 2}, ctrls{ci, 3}, modes{mi}, ctrls{ci, 4});
            os(i) = m.os; r90(i) = m.r90; lag(i) = m.lag; ss(i) = m.settle5;
        end
        fprintf('%-32s | %5.1f / %5.1f  | %6.0f / %6.0f    | %5.1f\n', ctrls{ci, 1}, mean(os), max(os), mean(r90), max(r90), mean(lag));
        rows(end + 1, :) = {modes{mi}, ctrls{ci, 1}, mean(os), max(os), mean(r90), max(r90), mean(ss), mean(lag)}; %#ok<SAGROW>
    end
end
R = cell2table(rows, 'VariableNames', {'mode', 'ctrl', 'os_mean', 'os_max', 'r90_mean_ms', 'r90_max_ms', 'settle5_mean_ms', 'lag_pct'});
writetable(R, fullfile(results_dir, 'rot_ff2dof_study.csv'));

%% ローカル関数
function p = plant_of(row, tgt)
    p.K = row(1); p.T = row(2); p.u0 = row(3) - tgt / row(1);
end

function m = run_ctrl(p, Kc, Ti, ff, mode, use_acc)
    Ts = 1e-3; N = 1200; tgt = 430;
    if strcmp(mode, 'ramp'), accel = 2500; else, accel = 1e9; end
    w = 0; I = 0; wprev = 0; ref = 0; wl = zeros(N, 1); rl = zeros(N, 1);
    for k = 1:N
        step = accel * Ts;
        dref = max(min(tgt - ref, step), -step);
        ref = ref + dref;
        uff = 0;
        if ~isempty(ff)
            uff = ff(1) * (ref > 0) + ref / ff(2);                 % 静的FF（不感帯u0 + ω/K）
            if use_acc, uff = uff + ff(3) / ff(2) * (dref / Ts); end   % 加速度FF (T/K)*dref/dt
        end
        e = ref - wprev;
        uu = uff + Kc * e + I;
        u = max(min(uu, 0.28), -0.28);
        I = I + (Kc / Ti * e + (u - uu) / Ti) * Ts;               % back-calculation（Tt=Ti）
        w = w + Ts * (p.K * max(u - p.u0, 0) - w) / p.T;
        wprev = w; wl(k) = w; rl(k) = ref;
    end
    m.os = max(0, (max(wl) - tgt) / tgt) * 100;
    r = find(wl >= 0.9 * tgt, 1, 'first'); if isempty(r), m.r90 = NaN; else, m.r90 = r; end
    o = find(abs(wl - tgt) > 0.05 * tgt, 1, 'last'); if isempty(o), m.settle5 = 0; else, m.settle5 = o + 1; end
    m.lag = max(rl(1:round(0.2 * N)) - wl(1:round(0.2 * N))) / tgt * 100;   % 最初の240ms内の指令に対する遅れの最大 [%目標]
end
