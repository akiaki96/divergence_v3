%% F3（目標角速度のレート制限）の実機検証：ステップ vs ランプの比較と合否判定
%
% 入力：tools/log/omega_step_v700_x/*.csv と tools/log/omega_ramp_v700_x/*.csv
%  - 列 omega_ref を持つCSV = F3ファーム（レート制限後の指令値をログ）。持たないCSV = F3以前（E3/E5/E6）のステップ指令
%  - omega_ramp_*     : 最大角加速度 2500 dps/s^2（運用仕様，config::pid_omega::OMEGA_ACCEL_MAX）
%  - omega_ramp4k_*   : 4000 dps/s^2（参考）
%  - omega_step_* (omega_ref列あり) : F3ファームでのステップ指令（回帰確認）
% 同名ファイルは --on_conflict sequence で _1,_2,... と連番になっているため，ファイル名の連番は無視して
% 列の有無・ファイル名の種別（ramp/ramp4k/step）でグループ化する。
%
% 合否基準（rot_gain_scheduling_plan.md §13.4）：
%   目標到達(定常誤差<3%)，OS<5%（100ms移動平均），立上り(90%到達)<=260ms(430dps)，
%   飽和時間比率<5%，並進速度低下<5%，±100dpsで10%整定<300ms

clear; clc;
results_dir = 'results';
dirs = {'../tools/log/omega_step_v700_x/', '../tools/log/omega_ramp_v700_x/'};

rows = struct('file', {}, 'group', {}, 'target', {}, 'fw', {}, 'accel', {}, 'ss', {}, 'err_pct', {}, 'sd', {}, ...
    'u_ss', {}, 'u_max', {}, 'os100', {}, 'r90', {}, 'set10', {}, 'sat_pct', {}, 'vdrop', {}, 'track_err', {});
traces = {};
for d = 1:numel(dirs)
    L = dir([dirs{d} 'omega_*.csv']);
    for k = 1:numel(L)
        fn = L(k).name;
        T = readtable([dirs{d} fn], 'VariableNamingRule', 'modify');
        on = find(T.target_omega ~= 0, 1, 'first');
        if isempty(on), continue; end
        tgt = T.target_omega(on);
        has_ref = ismember('omega_ref', T.Properties.VariableNames);
        if contains(fn, 'ramp4k'), acc = 4000; elseif contains(fn, 'ramp'), acc = 2500; else, acc = Inf; end
        if has_ref, fwn = 'F3'; else, fwn = 'pre-F3'; end
        if isinf(acc), kind = 'step'; else, kind = sprintf('ramp%d', acc); end
        grp = sprintf('%s %s', fwn, kind);

        idx = on:height(T);
        t = T.Global_time - T.Global_time(1); tr = t(idx) - t(on);
        g = T.gyro_z(idx); g100 = movmean(g, 100);
        u = T.duty_diff(idx);
        v = (T.left_encoder_velocity(idx) + T.right_encoder_velocity(idx)) / 2;
        s = sign(tgt);
        st = tr >= 0.4;
        r = struct();
        r.file = fn; r.group = grp; r.target = tgt; r.fw = fwn; r.accel = acc;
        r.ss = mean(g(st)); r.err_pct = 100 * (r.ss - tgt) / abs(tgt); r.sd = std(g(st));
        r.u_ss = mean(u(st)); r.u_max = max(abs(u));
        r.os100 = max(0, (max(s * g100) - abs(tgt)) / abs(tgt)) * 100;
        i90 = find(s * movmean(g, 20) >= 0.9 * abs(tgt), 1, 'first');
        r.r90 = ternary(isempty(i90), NaN, tr(i90) * 1000);
        i10 = find(abs(g100 - tgt) > 0.10 * abs(tgt), 1, 'last');
        r.set10 = ternary(isempty(i10), 0, tr(i10) * 1000);
        % F3ファームの初回ログは列数上限(Logger::MAX_FIELDS=16)でomega_saturatedが欠落している
        % （修正済み）。無い場合は出力が上限(0.28)に張り付いた区間で代用する
        if ismember('omega_saturated', T.Properties.VariableNames)
            r.sat_pct = 100 * mean(T.omega_saturated(idx) > 0);
        else
            r.sat_pct = 100 * mean(abs(u) >= 0.2795);
        end
        r.vdrop = 100 * (700 - min(movmean(v, 20))) / 700;
        if has_ref
            ref = T.omega_ref(idx);
            r.track_err = max(abs(ref - g100)) / abs(tgt) * 100;   % 指令ランプに対する追従遅れの最大値 [%目標]
        else
            ref = T.target_omega(idx); r.track_err = NaN;
        end
        rows(end + 1) = r; %#ok<SAGROW>
        traces{end + 1} = struct('tr', tr, 'g100', g100, 'ref', ref, 'u', u, 'tgt', tgt, 'grp', grp, 'file', fn); %#ok<SAGROW>
    end
end
if isempty(rows), error('omega_*.csv が見つかりません'); end
Rt = struct2table(rows);

%% 表示：グループ×目標ごとの平均（OSは最大・最小も）
[G, gname, gtgt] = findgroups(Rt.group, Rt.target);
fprintf('%-18s %6s %3s | %7s %6s | %6s %6s %6s | %6s %6s | %5s %6s | %5s\n', 'group', 'tgt', 'n', 'ss_err%', 'sd', 'OS100', 'OSmax', 'OSmin', 'r90', 'set10', 'sat%', 'vdrop', 'u_ss');
sm = table();
for gi = 1:max(G)
    m = G == gi; x = Rt(m, :);
    fprintf('%-18s %6.0f %3d | %7.1f %6.1f | %6.1f %6.1f %6.1f | %6.0f %6.0f | %5.1f %6.1f | %5.3f\n', gname{gi}, gtgt(gi), sum(m), ...
        mean(x.err_pct), mean(x.sd), mean(x.os100), max(x.os100), min(x.os100), mean(x.r90, 'omitnan'), mean(x.set10), mean(x.sat_pct), mean(x.vdrop), mean(x.u_ss));
    sm = [sm; table(gname(gi), gtgt(gi), sum(m), mean(x.err_pct), mean(x.sd), mean(x.os100), max(x.os100), min(x.os100), mean(x.r90, 'omitnan'), mean(x.set10), ...
        mean(x.sat_pct), mean(x.vdrop), 'VariableNames', {'group', 'target', 'n', 'err_pct', 'sd', 'os100_mean', 'os100_max', 'os100_min', 'r90_ms', 'set10_ms', 'sat_pct', 'vdrop_pct'})]; %#ok<AGROW>
end
writetable(sm, fullfile(results_dir, 'omega_ramp_summary_by_group.csv'));
writetable(Rt, fullfile(results_dir, 'omega_ramp_summary_by_run.csv'));

%% 合否判定（F3ファームのランプ2500）
fprintf('\n===== F3 ランプ(2500dps/s^2) 合否判定 =====\n');
mm = strcmp(Rt.group, 'F3 ramp2500');
if any(mm)
    X = Rt(mm, :);
    for i = 1:height(X)
        ok_reach = abs(X.err_pct(i)) < 3;
        ok_os = X.os100(i) < 5;
        ok_rise = (abs(X.target(i)) < 430) || (X.r90(i) <= 260);
        ok_sat = X.sat_pct(i) < 5;
        ok_v = X.vdrop(i) < 5;
        ok_tail = (abs(X.target(i)) > 100) || (X.set10(i) < 300);
        fprintf('%-26s tgt%5.0f | 到達%d OS%d 立上り%d 飽和%d 並進%d 低速尾%d  (err%+.1f%% OS%.1f%% r90 %.0fms sat%.1f%% vdrop%.1f%%)\n', X.file{i}, X.target(i), ...
            ok_reach, ok_os, ok_rise, ok_sat, ok_v, ok_tail, X.err_pct(i), X.os100(i), X.r90(i), X.sat_pct(i), X.vdrop(i));
    end
else
    fprintf('F3ランプのデータはまだありません。\n');
end

%% 重ね書き：目標ごとに，符号を揃えた100ms移動平均
levels = [430 -430 250 -250 100 -100];
keys = {'pre_F3_step', 'F3_step', 'F3_ramp2500', 'F3_ramp4000'};
colors = [0.2 0.2 0.2; 0.85 0.33 0.1; 0 0.45 0.74; 0.47 0.67 0.19];
fig = figure('Position', [30 30 1500 900]);
for li = 1:numel(levels)
    subplot(2, 3, li); hold on; grid on;
    shown = false(1, numel(keys));
    for k = 1:numel(traces)
        q = traces{k};
        if q.tgt ~= levels(li), continue; end
        ki = find(strcmp(keys, strrep(strrep(q.grp, '-', '_'), ' ', '_')), 1);
        if isempty(ki), continue; end
        if ~shown(ki), dn = strrep(q.grp, '_', '\_'); shown(ki) = true; else, dn = ''; end
        h = plot(q.tr * 1000, sign(q.tgt) * q.g100, 'Color', colors(ki, :));
        if isempty(dn), h.HandleVisibility = 'off'; else, h.DisplayName = dn; end
        if contains(q.grp, 'ramp') && ~contains(q.grp, 'pre')
            hr = plot(q.tr * 1000, sign(q.tgt) * q.ref, ':', 'Color', colors(ki, :)); hr.HandleVisibility = 'off';
        end
    end
    yline(abs(levels(li)), 'k:', 'HandleVisibility', 'off');
    title(sprintf('target %+d dps（符号を揃えた100ms平均, 点線=指令）', levels(li))); xlabel('t [ms]'); ylabel('\omega [dps]');
    if any(shown), legend('Location', 'southeast', 'FontSize', 7); end
end
exportgraphics(fig, fullfile(results_dir, 'omega_ramp_compare.png'));

%% ローカル関数
function y = ternary(c, a, b)
    if c, y = a; else, y = b; end
end
