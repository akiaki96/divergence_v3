%% 角速度の目標追従性能の集計（レベル別）
%
% 対象：ランプ指令（最大角加速度2500dps/s^2，並進700mm/s）に対する gyro_z の追従。
%   FF ON : omega_f7_ff_*（現行ファーム F7）。E9/E10のF5（omega_ff_*）も±250/±100の参考に併記
%   FF OFF: omega_f7_noff_*（純PIの基準）
% 指標（run単位→レベル×構成で 平均±標準偏差）：
%   定常誤差[%]      : 励振開始0.4s以降のgyro平均 vs 目標
%   定常ばらつき[dps]: 同区間のgyroの標準偏差（47Hzリップルを含む）と，100ms移動平均の標準偏差（ゆっくりした変動）
%   OS[%]            : 100ms移動平均の最大値の目標超過
%   90%到達[ms]      : 20ms移動平均が目標の90%に達する時刻
%   10%整定[ms]      : 100ms移動平均が目標の±10%帯に最後に入る時刻
%   ランプ追従誤差   : ランプ区間（指令が目標に達するまで）の gyro(20ms平均)−omega_ref の RMS[dps]・最大[dps]
%   角度誤差[deg]    : 励振開始〜0.6sの ∫(gyro−omega_ref)dt（旋回角の指令からのずれ。旋回量の精度に直結）
% 前提：tools/log/omega_ff_v700_x/ にF5/F7のCSV（cp -pで更新時刻保持）。ウォームアップ omega_f7_ff_pos430.csv は除外。

clear; clc;
results_dir = 'results';
src = '../tools/log/omega_ff_v700_x/';
L = dir([src 'omega_*.csv']);

rows = struct('file', {}, 'group', {}, 'level', {}, 'err_pct', {}, 'sd_raw', {}, 'sd_ma', {}, 'os', {}, 'r90', {}, 'set10', {}, ...
    'ramp_rms', {}, 'ramp_max', {}, 'ang_err', {});
traces = {};
for k = 1:numel(L)
    fn = L(k).name;
    if strcmp(fn, 'omega_f7_ff_pos430.csv'), continue; end        % E11のウォームアップ
    if contains(fn, 'ffold') || contains(fn, 'ffhi'), continue; end % 係数を変えた比較用は除く
    if contains(fn, 'f7_ff_'), grp = 'F7 ff（FF ON）';
    elseif contains(fn, 'f7_noff_'), grp = 'FF OFF（純PI）';
    elseif contains(fn, 'noff'), continue;                          % E9/E10のnoffは感度状態が異なるため使わない
    elseif startsWith(fn, 'omega_ff_') && (contains(fn, '250') || contains(fn, '100')), grp = 'F5 ff（E9, 参考）';
    else, continue;
    end
    T = readtable([src fn], 'VariableNamingRule', 'modify');
    if ~ismember('omega_ref', T.Properties.VariableNames), continue; end
    on = find(T.target_omega ~= 0, 1, 'first'); tgt = T.target_omega(on);
    idx = on:height(T);
    t = T.Global_time - T.Global_time(1); tr = t(idx) - t(on);
    g = T.gyro_z(idx); ref = T.omega_ref(idx);
    g20 = movmean(g, 20); g100 = movmean(g, 100);
    s = sign(tgt);
    st = tr >= 0.4;
    r = struct();
    r.file = fn; r.group = grp; r.level = tgt;
    r.err_pct = 100 * (mean(g(st)) - tgt) / abs(tgt);
    r.sd_raw = std(g(st)); r.sd_ma = std(g100(st));
    r.os = max(0, (max(s * g100) - abs(tgt)) / abs(tgt)) * 100;
    i90 = find(s * g20 >= 0.9 * abs(tgt), 1, 'first'); r.r90 = ternary(isempty(i90), NaN, tr(i90) * 1000);
    i10 = find(abs(g100 - tgt) > 0.1 * abs(tgt), 1, 'last'); r.set10 = ternary(isempty(i10), 0, tr(i10) * 1000);
    ramp_end = find(abs(ref - tgt) < 1e-3 * abs(tgt) + 1e-6, 1, 'first'); if isempty(ramp_end), ramp_end = numel(ref); end
    e = g20(1:ramp_end) - ref(1:ramp_end);
    r.ramp_rms = sqrt(mean(e .^ 2)); r.ramp_max = max(abs(e));
    n6 = find(tr <= 0.598, 1, 'last');
    r.ang_err = trapz(tr(1:n6), g(1:n6) - ref(1:n6));
    rows(end + 1) = r; %#ok<SAGROW>
    traces{end + 1} = struct('tr', tr, 'g100', g100, 'ref', ref, 'tgt', tgt, 'grp', grp); %#ok<SAGROW>
end
Rt = struct2table(rows);
[G, gname, glevel] = findgroups(Rt.group, Rt.level);
metrics = {'err_pct', 'sd_raw', 'sd_ma', 'os', 'r90', 'set10', 'ramp_rms', 'ramp_max', 'ang_err'};
sm = table();
fprintf('%-18s %6s %3s | %s\n', 'group', 'level', 'n', 'err%  sd_raw sd_ma OS%  r90  set10 rampRMS rampMax angErr[deg]  (mean ± sd)');
for gi = 1:max(G)
    m = G == gi; x = Rt(m, :);
    v = zeros(1, numel(metrics)); sd = v;
    for j = 1:numel(metrics), v(j) = mean(x.(metrics{j}), 'omitnan'); sd(j) = std(x.(metrics{j}), 'omitnan'); end
    if sum(m) < 2, sd(:) = NaN; end
    fprintf('%-18s %6.0f %3d | %+5.1f %5.1f %5.1f %4.1f %4.0f %5.0f %6.1f %6.1f %+6.2f ± %.2f\n', gname{gi}, glevel(gi), sum(m), v(1), v(2), v(3), v(4), v(5), v(6), v(7), v(8), v(9), sd(9));
    row = table(gname(gi), glevel(gi), sum(m), 'VariableNames', {'group', 'level', 'n'});
    for j = 1:numel(metrics)
        row.([metrics{j} '_mean']) = v(j); row.([metrics{j} '_sd']) = sd(j);
    end
    sm = [sm; row]; %#ok<AGROW>
end
writetable(sm, fullfile(results_dir, 'omega_tracking_summary.csv'));
writetable(Rt, fullfile(results_dir, 'omega_tracking_by_run.csv'));

%% 図：レベル別に指令(点線)と実測(100ms平均，全run薄線＋平均)
levels = [430 250 100 -100 -250 -430];
fig = figure('Position', [30 30 1500 850]);
cols = struct('on', [0 0.45 0.74], 'off', [0.8 0.3 0.1], 'f5', [0.2 0.6 0.3]);
for li = 1:numel(levels)
    subplot(2, 3, li); hold on; grid on;
    lv = levels(li); s = sign(lv);
    for gk = {'FF OFF（純PI）', 'F5 ff（E9, 参考）', 'F7 ff（FF ON）'}
        gsel = gk{1};
        if contains(gsel, 'OFF'), c = cols.off; elseif contains(gsel, 'F5'), c = cols.f5; else, c = cols.on; end
        tt = []; Y = [];
        for q = 1:numel(traces)
            if traces{q}.tgt ~= lv || ~strcmp(traces{q}.grp, gsel), continue; end
            n = min(numel(traces{q}.tr), 598);
            plot(traces{q}.tr(1:n) * 1000, s * traces{q}.g100(1:n), 'Color', [c 0.25], 'HandleVisibility', 'off');
            if isempty(tt), tt = traces{q}.tr(1:n); Y = zeros(0, n); end
            Y(end + 1, :) = traces{q}.g100(1:n)'; %#ok<AGROW>
        end
        if ~isempty(Y), plot(tt * 1000, s * mean(Y, 1), 'Color', c, 'LineWidth', 1.8, 'DisplayName', sprintf('%s n=%d', gsel, size(Y, 1))); end
    end
    for q = 1:numel(traces)
        if traces{q}.tgt == lv && contains(traces{q}.grp, 'F7')
            n = min(numel(traces{q}.tr), 598);
            plot(traces{q}.tr(1:n) * 1000, s * traces{q}.ref(1:n), 'k:', 'LineWidth', 1.3, 'DisplayName', '指令 \omega\_ref'); break;
        end
    end
    yline(abs(lv), 'k-', 'HandleVisibility', 'off', 'Alpha', 0.3);
    yline([0.9 1.1] * abs(lv), 'k--', 'HandleVisibility', 'off', 'Alpha', 0.25);
    title(sprintf('目標 %+d dps（符号を揃えた100ms平均。破線=±10%%）', lv)); xlabel('t [ms]'); ylabel('\omega [dps]'); legend('Location', 'southeast', 'FontSize', 7);
end
exportgraphics(fig, fullfile(results_dir, 'omega_tracking_overview.png'));

function y = ternary(c, a, b)
    if c, y = a; else, y = b; end
end
