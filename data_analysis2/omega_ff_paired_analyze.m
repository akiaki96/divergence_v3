%% F5（2自由度FF）の ON/OFF を「ペア差」と「感度補正」で比較する
%
% 前提：omega_ramp_analyze.m を先に実行し results/omega_ramp_summary_by_run.csv を作っておく。
%       tools/log/omega_ff_v700_x/ のCSVは `cp -p`（更新時刻を保持）でworktreeへコピーしておく（取得順の復元に使う）。
%
% 背景：プラントの感度（同じωを出す定常duty u_ss）は時間とともにドリフトし（E9では約3分で0.206→0.242），
% OSは感度に強く依存する（感度が高い=u_ssが小さいほどOSが大きい）。ON/OFFを別々に平均すると
% ドリフトと交絡するため，(1)取得順に隣り合う ON/OFF をペアにしてペア差を見る，
% (2) OS ~ b0 + b1*(u_ss - 平均) + b2*FF の回帰でFFの効果b2を感度補正して推定する。
%
% 使い方：ff と noff を A,B,B,A,... と交互（順序を反転）に取得すると，時間ドリフトの影響が相殺される。

clear; clc;
results_dir = 'results';
src_dir = '../tools/log/omega_ff_v700_x/';
R = readtable(fullfile(results_dir, 'omega_ramp_summary_by_run.csv'));
R = R(startsWith(R.group, 'F5'), :);
if isempty(R), error('F5のデータがありません。omega_ramp_analyze.m を先に実行してください。'); end
kind = extractAfter(R.group, 'F5 ');           % ff / ffhi / noff
mt = zeros(height(R), 1);
for i = 1:height(R)
    d = dir([src_dir R.file{i}]);
    mt(i) = d.datenum;
end
R.kind = kind; R.mtime = mt;

metrics = {'os100', 'r90', 'track_err', 'err_pct', 'sat_pct', 'u_ss'};
mlabel  = {'OS[%]', '90%到達[ms]', '指令への遅れ[%]', '定常誤差[%]', '飽和[%]', 'u_ss'};
rows = {};
targets = [430 -430];
for tg = targets
    X = R(R.target == tg & (strcmp(R.kind, 'ff') | strcmp(R.kind, 'noff')), :);
    X = sortrows(X, 'mtime');
    used = false(height(X), 1);
    pairs = zeros(0, 2);   % [ON行, OFF行]
    for i = 1:height(X) - 1
        if used(i) || used(i + 1), continue; end
        if ~strcmp(X.kind{i}, X.kind{i + 1})
            if strcmp(X.kind{i}, 'ff'), pairs(end + 1, :) = [i, i + 1]; else, pairs(end + 1, :) = [i + 1, i]; end %#ok<AGROW>
            used([i, i + 1]) = true;
        end
    end
    fprintf('\n===== target %+d：ペア数 %d（ON/OFFの取得順で隣り合うもの） =====\n', tg, size(pairs, 1));
    if isempty(pairs), continue; end
    fprintf('%-18s | %s\n', '指標', 'ON−OFF: 各ペア ...  | 平均 ± SE | 平均(ON)  平均(OFF)');
    for m = 1:numel(metrics)
        on_v = X.(metrics{m})(pairs(:, 1)); off_v = X.(metrics{m})(pairs(:, 2));
        dv = on_v - off_v;
        se = std(dv) / sqrt(numel(dv)); if numel(dv) < 2, se = NaN; end
        fprintf('%-18s | %s | %+7.2f ± %5.2f | %8.2f %8.2f\n', mlabel{m}, sprintf('%+7.2f ', dv), mean(dv), se, mean(on_v), mean(off_v));
        rows(end + 1, :) = {tg, metrics{m}, size(pairs, 1), mean(dv), se, mean(on_v), mean(off_v)}; %#ok<SAGROW>
    end
    % 感度補正の回帰: OS ~ 1 + (u_ss - mean) + isFF（ffhiは除外）
    Z = X;
    isff = strcmp(Z.kind, 'ff');
    uc = abs(Z.u_ss) - mean(abs(Z.u_ss));
    A = [ones(height(Z), 1), uc, double(isff)];
    if height(Z) >= 5
        b = A \ Z.os100;
        res = Z.os100 - A * b;
        s2 = sum(res .^ 2) / max(height(Z) - 3, 1);
        cov_b = s2 * inv(A' * A); %#ok<MINV>
        fprintf('感度補正回帰 OS = %.2f %+.1f*(|u_ss|-平均) %+.2f*FF   （FF効果 %+.2f ± %.2f [pt], n=%d）\n', b(1), b(2), b(3), b(3), sqrt(cov_b(3, 3)), height(Z));
    end
end
T = cell2table(rows, 'VariableNames', {'target', 'metric', 'n_pairs', 'mean_on_minus_off', 'se', 'mean_on', 'mean_off'});
writetable(T, fullfile(results_dir, 'omega_ff_paired_summary.csv'));
