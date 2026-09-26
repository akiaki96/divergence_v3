%% F5（2自由度FF）の ON/OFF を「隣り合う取得のペア差」と「トレンド補正回帰」で比較する
%
% 前提：omega_ramp_analyze.m を先に実行し results/omega_ramp_summary_by_run.csv を作っておく。
%       tools/log/omega_ff_v700_x/ のCSVは `cp -p`（更新時刻を保持）でworktreeへコピーしておく（取得順の復元に使う）。
%
% 背景：プラントの感度（同じωを出す定常duty u_ss）はバッテリ電圧・温度などで時間ドリフトし，OSは感度に強く依存する。
% ON/OFFを別々に平均するとドリフトと交絡する。そこで
%  (1) 取得順に隣り合う ff/noff の差（ON−OFF）を取る。ABAB順（ff,noff,ff,noff,...）で取ると
%      「ON先のペア」と「OFF先のペア」でドリフトの符号が逆になるため，両方向を平均すると線形ドリフトが相殺される。
%  (2) 回帰  指標 ~ 1 + 時間[分] + FF  … 時間トレンドを補正したFF効果
%      回帰  指標 ~ 1 + (|u_ss| - 平均) + FF … 感度(u_ss)を補正したFF効果
% セッション（取得の時間間隔が20分超で分割）ごと・目標ごとに集計する。
% F6（E12）：omega_f6_on_ を kind='ff'（F6 ON），omega_f6_off_ を kind='noff'（F6 OFF＝補償なしの従来動作）として扱う。
%   出力の「ON−OFF」「FF効果」は，F6では「バッテリ補償＋電圧基準上限の効果」と読む（±250も対象）。
% ウォームアップ：セッション全体の最初の1本が「同じ目標で同じ種類が2本続く」形（例 ff, ff, noff, ...）のとき除外する。

clear; clc;
results_dir = 'results';
src_dir = '../../tools/log/omega_ff_v700_x/';
R = readtable(fullfile(results_dir, 'omega_ramp_summary_by_run.csv'));
R = R(startsWith(R.group, 'F5') | startsWith(R.group, 'F7') | startsWith(R.group, 'F6'), :);
if isempty(R), error('F5/F7のデータがありません。omega_ramp_analyze.m を先に実行してください。'); end
R.kind = extractAfter(R.group, ' ');            % ff / ffhi / ffold / noff（F5/F7とも）
mt = zeros(height(R), 1);
for i = 1:height(R)
    d = dir([src_dir R.file{i}]);
    mt(i) = d.datenum;
end
R.mtime = mt;
R = sortrows(R, 'mtime');
gap = [inf; diff(R.mtime)] * 24 * 60;            % [分]
R.session = cumsum(gap > 20);

metrics = {'os100', 'r90', 'track_err', 'err_pct', 'sat_pct', 'u_ss'};
mlabel  = {'OS[%]', '90%到達[ms]', '指令への遅れ[%]', '定常誤差[%]', '飽和[%]', 'u_ss'};
rows = {};
for ss = unique(R.session)'
    for tg = [430 -430 250 -250]
        X = R(R.session == ss & R.target == tg, :);
        X = X(strcmp(X.kind, 'ff') | strcmp(X.kind, 'noff'), :);   % ffhiは別途
        if height(X) < 2, continue; end
        % ウォームアップ除外：セッション全体の最初の1本で，かつ同じ目標の次の1本と種類が同じとき
        % （ff, ff, noff, ... の形）。-430の先頭など，セッション先頭でない run は除外しない
        if strcmp(X.kind{1}, X.kind{2}) && X.mtime(1) == min(R.mtime(R.session == ss)), X(1, :) = []; end
        t_min = (X.mtime - X.mtime(1)) * 24 * 60;
        fprintf('\n===== セッション%d  %s  target %+d：ff %d本 / noff %d本（%s〜） =====\n', ss, datestr(X.mtime(1), 'mm/dd HH:MM'), tg, ...
            sum(strcmp(X.kind, 'ff')), sum(strcmp(X.kind, 'noff')), datestr(X.mtime(1), 'HH:MM'));
        % (1) 隣り合うペア（両方向）
        on_first = []; off_first = [];
        for i = 1:height(X) - 1
            if strcmp(X.kind{i}, 'ff') && strcmp(X.kind{i + 1}, 'noff'), on_first(end + 1) = i; %#ok<AGROW>
            elseif strcmp(X.kind{i}, 'noff') && strcmp(X.kind{i + 1}, 'ff'), off_first(end + 1) = i; end %#ok<AGROW>
        end
        fprintf('隣り合うペア：ON先 %d，OFF先 %d\n', numel(on_first), numel(off_first));
        fprintf('%-16s | %-22s | %-22s | %-22s\n', '指標(ON−OFF)', 'ON先ペア 平均', 'OFF先ペア 平均', '両方向の平均 ± SE (n)');
        for m = 1:numel(metrics)
            v = X.(metrics{m});
            d1 = v(on_first) - v(on_first + 1);            % ON先: ON=i, OFF=i+1
            d2 = v(off_first + 1) - v(off_first);          % OFF先: OFF=i, ON=i+1
            dall = [d1; d2];
            se = std(dall) / sqrt(numel(dall));
            fprintf('%-16s | %+9.2f            | %+9.2f            | %+7.2f ± %5.2f (%d)\n', mlabel{m}, mean(d1), mean(d2), mean(dall), se, numel(dall));
            rows(end + 1, :) = {ss, tg, metrics{m}, mean(d1), mean(d2), mean(dall), se, numel(dall), mean(v(strcmp(X.kind, 'ff'))), mean(v(strcmp(X.kind, 'noff')))}; %#ok<SAGROW>
        end
        % (2) 回帰（ff/noffのみ）
        isff = double(strcmp(X.kind, 'ff'));
        uc = abs(X.u_ss) - mean(abs(X.u_ss));
        for m = 1:3
            y = X.(metrics{m});
            out = sprintf('%-16s 回帰:', mlabel{m});
            for mode = 1:2
                if mode == 1, z = t_min - mean(t_min); nm = '時間'; else, z = uc; nm = 'u_ss'; end
                A = [ones(numel(y), 1), z, isff];
                if numel(y) < 5 || rank(A) < 3, continue; end
                b = A \ y; res = y - A * b; s2 = sum(res .^ 2) / (numel(y) - 3);
                cb = s2 * inv(A' * A); %#ok<MINV>
                out = [out, sprintf('  [%s補正 FF効果 %+.2f ± %.2f]', nm, b(3), sqrt(cb(3, 3)))]; %#ok<AGROW>
            end
            fprintf('%s\n', out);
        end
        % ffhi / ffold vs ff（同セッションの隣り合い。ffold=F5相当(係数0.625倍)なら ff−ffold がF7の効果）
        for other = {'ffhi', 'ffold'}
            Y = R(R.session == ss & R.target == tg & (strcmp(R.kind, other{1}) | strcmp(R.kind, 'ff')), :);
            dh = zeros(0, 3);
            for i = 1:height(Y) - 1
                if strcmp(Y.kind{i}, other{1}) && strcmp(Y.kind{i + 1}, 'ff')
                    dh(end + 1, :) = [Y.os100(i) - Y.os100(i + 1), Y.r90(i) - Y.r90(i + 1), Y.track_err(i) - Y.track_err(i + 1)]; %#ok<AGROW>
                elseif strcmp(Y.kind{i}, 'ff') && strcmp(Y.kind{i + 1}, other{1})
                    dh(end + 1, :) = [Y.os100(i + 1) - Y.os100(i), Y.r90(i + 1) - Y.r90(i), Y.track_err(i + 1) - Y.track_err(i)]; %#ok<AGROW>
                end
            end
            if ~isempty(dh)
                if strcmp(other{1}, 'ffold'), dh = -dh; lbl = 'F7(ff)−F5相当(ffold)（負=F7が改善）'; else, lbl = [other{1} '−ff']; end
                fprintf('%s（隣り合い %d組の平均）: OS %+.2f pt, 90%%到達 %+.1f ms, 指令への遅れ %+.1f pt\n', lbl, size(dh, 1), mean(dh(:, 1)), mean(dh(:, 2)), mean(dh(:, 3)));
            end
        end
    end
end
T = cell2table(rows, 'VariableNames', {'session', 'target', 'metric', 'on_first_mean', 'off_first_mean', 'both_mean', 'se', 'n_pairs', 'mean_on', 'mean_off'});
writetable(T, fullfile(results_dir, 'omega_ff_paired_summary.csv'));
