%% 閉ループ実機データから動作点まわりの局所プラント（ゲイン・時定数）を推定し，オーバーシュートの原因を調べる
%
% 背景：+430/+400でステップ指令でもランプ指令でも同程度(8〜14%)のオーバーシュートが出る。
% 0からのstep応答から作ったプラントモデル(H1/H4)は実機のOSを再現できない(rot_gain_scheduling_plan.md §12, §14)。
% PI(Kc, Ti)＋一次遅れ(K, T)では，Ti < T のときPIの零点がプラントの極より速く，指令の形によらず
% オーバーシュートが出る。実機の閉ループデータで局所的な T を直接推定して確かめる。
%
% 手法：定常付近(ωが目標の70%超)の区間で，スムージングした ω̇ を u と ω に回帰
%       d(w)/dt = c + a*u - b*w        （K_loc = a/b [dps/duty], T_loc = 1/b [s]）
% 閉ループ(uがωに依存)のため同定はバイアスを持ちうる。グループ内のrun間ばらつきと，
% 推定した局所モデル上でのPI閉ループ模擬が実測OSを再現するかで妥当性を判断する。

clear; clc;
results_dir = 'results';
dirs = {'../tools/log/omega_step_v700_x/', '../tools/log/omega_ramp_v700_x/'};

% 対象：|目標|=430の全run（+と-）を，方向×指令方式で集計
L = [dir([dirs{1} 'omega_*430*.csv']); dir([dirs{2} 'omega_*430*.csv'])];
recs = struct('file', {}, 'tgt', {}, 'kind', {}, 'a', {}, 'b', {}, 'Tloc', {}, 'Kloc', {}, 'n', {});
smn = 30;   % ω, u, ω̇ のスムージング窓 [ms]
for k = 1:numel(L)
    fn = L(k).name; folder = L(k).folder;
    T = readtable(fullfile(folder, fn), 'VariableNamingRule', 'modify');
    on = find(T.target_omega ~= 0, 1, 'first');
    if isempty(on), continue; end
    tgt = T.target_omega(on);
    if contains(fn, 'ramp4k'), kind = 'ramp4k'; elseif contains(fn, 'ramp'), kind = 'ramp'; else, kind = 'step'; end
    idx = on:height(T);
    w = movmean(T.gyro_z(idx), smn);
    u = movmean(T.duty_diff(idx), smn);
    dw = gradient(w) / 1e-3;          % [dps/s]
    dw = movmean(dw, smn);
    s = sign(tgt);
    sel = (s * w > 0.7 * abs(tgt)) & ((1:numel(w))' > smn) & ((1:numel(w))' < numel(w) - smn);
    if sum(sel) < 100, continue; end
    % 符号を揃えて回帰（ω>0, u>0, ω̇の符号を統一）
    X = [ones(sum(sel), 1), s * u(sel), -s * w(sel)];
    y = s * dw(sel);
    p = X \ y;
    recs(end + 1) = struct('file', fn, 'tgt', tgt, 'kind', kind, 'a', p(2), 'b', p(3), 'Tloc', 1 / p(3), 'Kloc', p(2) / p(3), 'n', sum(sel)); %#ok<SAGROW>
end
R = struct2table(recs);
disp(R(:, {'file', 'tgt', 'kind', 'a', 'b', 'Tloc', 'Kloc', 'n'}));

for sg = [1 -1]
    m = sign(R.tgt) == sg & R.b > 0;
    fprintf('\n方向%+d: n=%d  T_loc 中央値=%.0f ms (四分位 %.0f〜%.0f), K_loc 中央値=%.0f dps/duty, a 中央値=%.0f\n', sg, sum(m), ...
        1000 * median(R.Tloc(m)), 1000 * prctile(R.Tloc(m), 25), 1000 * prctile(R.Tloc(m), 75), median(R.Kloc(m)), median(R.a(m)));
end
writetable(R, fullfile(results_dir, 'rot_local_model_fit.csv'));
