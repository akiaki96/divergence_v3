%% PI(Kc, Ti) と局所プラント K/(Ts+1) の組合せに対するオーバーシュートの感度マップ（指令ステップ）
%
% 動作点(≈±430dps)まわりの局所線形モデルで，指令を10%ステップさせたときのオーバーシュートを計算する。
% 目的：実機の +側(T_loc 大, K_loc 大)で OS が大きく，-側(T_loc≈75ms)で小さい理由を，
% Ti と T_loc の比で説明できるか確認し，Ti/Kc をどう変えれば OS を下げられるか見積もる。
% （実測の局所T, Kは閉ループ回帰で不確かさが大きい: rot_local_model_fit.m。ここは感度の見積りで，
%  絶対値の予測ではない。1kHz離散, 1サンプル遅れ）

clear; clc;
results_dir = 'results';
Kc_list = [3e-4 5e-4 8e-4];
plants = [75 1700; 100 3000; 150 4000; 200 6000; 300 6000];   % [T_loc ms, K_loc dps/duty]
Ti_list = [0.0653 0.09 0.13 0.18 0.25];                       % [s]（現行 Ti(430)=65.3ms）

fprintf('OS%% (指令+10%%ステップ, 線形局所モデル)   行: プラント(T[ms],K)  列: Ti[ms]\n');
rows = {};
for Kc = Kc_list
    fprintf('\nKc = %.1e\n', Kc);
    fprintf('%-16s', 'plant \ Ti[ms]'); fprintf('%8.0f', Ti_list * 1000); fprintf('   |  Kc*K  |  90%%到達[ms]@現行Ti\n');
    for p = 1:size(plants, 1)
        Tp = plants(p, 1) / 1000; K = plants(p, 2);
        os = zeros(1, numel(Ti_list)); r90 = zeros(1, numel(Ti_list));
        for i = 1:numel(Ti_list)
            [os(i), r90(i)] = sim_lin(Kc, Ti_list(i), K, Tp);
            rows(end + 1, :) = {Kc, Ti_list(i), plants(p, 1), K, os(i), r90(i)}; %#ok<SAGROW>
        end
        fprintf('T=%3d K=%5d   ', plants(p, 1), K); fprintf('%8.1f', os); fprintf('   | %5.1f  | %6.0f\n', Kc * K, r90(1));
    end
end
R = cell2table(rows, 'VariableNames', {'Kc', 'Ti_s', 'T_loc_ms', 'K_loc', 'os_pct', 'r90_ms'});
writetable(R, fullfile(results_dir, 'rot_pi_linear_os_map.csv'));

function [os, r90] = sim_lin(Kc, Ti, K, Tp)
    % 出力 w [dps]，制御 u = Kc*e + I，プラント dw/dt = (K*u - w)/Tp，1サンプル遅れの測定
    Ts = 1e-3; N = 1500; ref = 430;
    w = 0; I = 0; wprev = 0; wl = zeros(N, 1);
    for k = 1:N
        e = ref - wprev;
        u = Kc * e + I;
        I = I + Kc / Ti * e * Ts;
        w = w + Ts * (K * u - w) / Tp;
        wprev = w; wl(k) = w;
    end
    wl = wl / ref;
    os = max(0, max(wl) - 1) * 100;
    i = find(wl >= 0.9, 1, 'first');
    if isempty(i), r90 = NaN; else, r90 = i; end
end
