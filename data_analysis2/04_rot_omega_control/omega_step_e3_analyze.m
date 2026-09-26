%% 回転角速度PI 実機試験(E3)の解析：スケジュール後ファーム（Kc=5e-4, Ti表, 上限0.26）
%
% tools/log/omega_step_v700_x/ の 新ファーム10本（±100,±200_1,±250,±400,±430）を解析し，
% 同じ制御器の閉ループ模擬（rot_step_v700_p1fit.m のプラント）と比較する。
% E1（旧ファーム）3本も併記して，ゲイン変更の効果（リップル・OS・立上り）を見る。
%
% 追加観点：
%  - 定常区間(0.3s〜)のリップル：std とスペクトル（主成分の周波数・振幅），閉ループ極の減衰不足か外乱かの切り分け
%  - 定常duty u_ss の正負・大きさ依存（バイアス/非対称性）と，上限0.26に対する余裕
%  - 並進速度の低下（旋回による干渉）

clear; clc;
results_dir = 'results';
datadir = '../../tools/log/omega_step_v700_x/';

% {ファイル名, 目標, ファーム(1=旧E1, 2=Ti表(上限0.26), 3=F2(上限0.28))}
runs = { ...
    'pos200',   200, 1; 'neg200',  -200, 1; 'pos250',   250, 1; ...
    'pos100',   100, 2; 'neg100',  -100, 2; ...
    'pos200_1', 200, 2; 'neg200_1', -200, 2; ...
    'pos250_1', 250, 2; 'neg250',  -250, 2; ...
    'pos400',   400, 2; 'neg400',  -400, 2; ...
    'pos430',   430, 2; 'neg430',  -430, 2; ...
    'pos400_1', 400, 3; 'pos400_2', 400, 3; ...
    'pos430_1', 430, 3; 'pos430_2', 430, 3; 'pos430_3', 430, 3; 'pos430_4', 430, 3; ...
    'neg400_1', -400, 3; 'neg400_2', -400, 3; 'neg430_1', -430, 3; 'neg430_2', -430, 3};
nr = size(runs, 1);

pr = readtable(fullfile('..', '03_rot_identification', 'results', 'rot_step_v700_p1fit_model_params.csv'));
Kmap  = @(x) sign(x) .* (pr.K_a * abs(x) + pr.K_b * abs(x) .^ pr.K_n);
Tmap  = @(x) pr.T_T0 + pr.T_c * abs(x) .^ pr.T_m;

% firmware設定（config::pid_omega）
TI_BP_W = [0 100 200 250 400 430];
TI_BP_T = [0.0090 0.01725 0.0345 0.0420 0.0615 0.06525];
fw(1) = struct('Kc', 3.7e-4, 'Ti', @(w) 0.0231, 'ulim', 0.20);
fw(2) = struct('Kc', 5.0e-4, 'Ti', @(w) interp1(TI_BP_W, TI_BP_T, min(abs(w), 430)), 'ulim', 0.26);
fw(3) = fw(2); fw(3).ulim = 0.28;   % F2

smooth_n = 20;
ug0 = linspace(0, 0.32, 3201);
res = struct();
fig = figure('Position', [20 20 1700 1800]);
for r = 1:nr
    name = runs{r, 1}; tgt = runs{r, 2}; v = runs{r, 3};
    T = readtable([datadir sprintf('omega_step_%s.csv', name)], 'VariableNamingRule', 'modify');
    t = T.Global_time - T.Global_time(1);
    on = find(T.target_omega ~= 0, 1, 'first');
    idx = on:height(T);
    tr = t(idx) - t(on);
    g = T.gyro_z(idx); gs = movmean(g, smooth_n);
    u = T.duty_diff(idx);
    vavg = (T.left_encoder_velocity(idx) + T.right_encoder_velocity(idx)) / 2;
    s = sign(tgt);
    steady = tr >= 0.3;

    % --- 指標
    m.name = name; m.fw = v; m.target = tgt;
    m.ss = mean(g(steady)); m.ss_err_pct = 100 * (m.ss - tgt) / abs(tgt);
    m.sd = std(g(steady)); m.sd_pct = 100 * m.sd / abs(tgt);
    m.u_ss = mean(u(steady)); m.u_max = max(abs(u));
    m.margin_to_limit = fw(v).ulim - abs(m.u_ss);
    m.r90 = tr(find(s * gs >= 0.9 * abs(tgt), 1, 'first')) * 1000;
    m.os = max(0, (max(s * gs) - abs(tgt)) / abs(tgt)) * 100;
    m.set10 = tr(find(abs(gs - tgt) > 0.10 * abs(tgt), 1, 'last')) * 1000;
    g100 = movmean(g, 100);   % 47Hz前後のリップル(機体固有, 開ループでも同振幅)を除いた応答
    m.os100 = max(0, (max(s * g100) - abs(tgt)) / abs(tgt)) * 100;
    m.set10_100 = tr(find(abs(g100 - tgt) > 0.10 * abs(tgt), 1, 'last')) * 1000;
    m.sat_ms = sum(T.omega_saturated(idx) > 0);
    m.v_min = min(vavg); m.v_drop_pct = 100 * (700 - min(movmean(vavg, smooth_n))) / 700;

    % --- リップルのスペクトル（定常区間, 平均除去, Hann窓）
    gg = g(steady) - mean(g(steady));
    Nf = numel(gg); w = 0.5 - 0.5 * cos(2 * pi * (0:Nf - 1)' / (Nf - 1)); Y = abs(fft(gg .* w)) / sum(w) * 2;
    f = (0:Nf - 1)' / (Nf * 1e-3);
    band = f >= 3 & f <= 60;
    [pk, ip] = max(Y(band)); fb = f(band); m.ripple_hz = fb(ip); m.ripple_amp = pk;
    m.ripple_amp_pct = 100 * pk / abs(tgt);

    % --- 模擬（同じ制御器・ノイズなし）
    Ts = 1e-3; N = numel(idx);
    Ti = fw(v).Ti(tgt); Kc = fw(v).Kc; ulim = fw(v).ulim;
    wp = 0; I = 0; wprev = 0; wsim = zeros(N, 1);
    for k = 1:N
        e = tgt - wprev; uu = Kc * e + I; uc = max(min(uu, ulim), -ulim);
        I = I + (Kc / Ti * e + (uc - uu) / Ti) * Ts;
        wp = wp + Ts * (Kmap(uc) - wp) / Tmap(uc); wprev = wp; wsim(k) = wp;
    end
    m.sim_r90 = tr(find(s * wsim >= 0.9 * abs(tgt), 1, 'first')) * 1000;
    m.sim_os = max(0, (max(s * wsim) - abs(tgt)) / abs(tgt)) * 100;
    m.sim_os100 = max(0, (max(s * movmean(wsim, 100)) - abs(tgt)) / abs(tgt)) * 100;
    m.sim_u_ss = interp1(Kmap(ug0), ug0, abs(tgt), 'linear', ug0(end)) * s;   % 0からのstep由来の静特性が要求するu
    res(r).m = m; %#ok<SAGROW>
    res(r).tr = tr; res(r).g = g; res(r).gs = gs; res(r).u = u; res(r).wsim = wsim; res(r).vavg = vavg;

    subplot(6, 4, r);
    plot(tr * 1000, g, 'Color', [0.8 0.8 0.8]); hold on;
    plot(tr * 1000, gs, 'k', 'LineWidth', 1.2);
    plot(tr * 1000, wsim, 'r--', 'LineWidth', 1);
    yline(tgt, 'b:');
    title(sprintf('%s (fw%d) ss%+.0f sd%.0f', strrep(name, '_', '\_'), v, m.ss, m.sd)); grid on;
    xlabel('t [ms]'); ylabel('\omega [dps]');
end
exportgraphics(fig, fullfile(results_dir, 'omega_step_e3_tracking.png'));
savefig(fig, fullfile(results_dir, 'omega_step_e3_tracking.fig'));

%% 表示
fprintf('%-9s %2s %5s | %7s %6s | %5s %6s | %6s %6s %6s | %5s %5s %5s | %4s %5s | %5s %6s\n', ...
    'file', 'fw', 'tgt', 'ss', 'err%', 'sd', 'sd%', 'u_ss', 'umax', 'margin', 'r90', 'sr90', 'OS%', 'sat', 'vdrop%', 'rHz', 'rAmp%');
for r = 1:nr
    m = res(r).m;
    fprintf('%-9s %2d %5.0f | %7.1f %6.1f | %5.1f %6.1f | %6.3f %6.3f %6.3f | %5.0f %5.0f %5.1f | %4d %5.1f | %5.1f %6.1f\n', ...
        m.name, m.fw, m.target, m.ss, m.ss_err_pct, m.sd, m.sd_pct, m.u_ss, m.u_max, m.margin_to_limit, m.r90, m.sim_r90, m.os, m.sat_ms, m.v_drop_pct, m.ripple_hz, m.ripple_amp_pct);
end

fprintf('\n===== リップル除去後（100ms移動平均）の指標：OS と 10%%整定，模擬OS =====\n');
fprintf('%-9s %2s %5s | %7s %8s %9s | %6s\n', 'file', 'fw', 'tgt', 'OS100%', 'simOS100', 'set10_100', 'sat%');
for r = 1:nr
    m = res(r).m;
    fprintf('%-9s %2d %5.0f | %7.1f %8.1f %9.0f | %6.1f\n', m.name, m.fw, m.target, m.os100, m.sim_os100, m.set10_100, 100 * m.sat_ms / numel(res(r).tr));
end

%% 正負ペアの定常duty：対称成分 u_sym と バイアス b（新ファームのペア）
fprintf('\n===== 定常dutyの対称/バイアス分解（新ファーム）：u_sym=(u+ - u-)/2, b=(u+ + u-)/2 =====\n');
pairs = {'pos100', 'neg100'; 'pos200_1', 'neg200_1'; 'pos250_1', 'neg250'; 'pos400', 'neg400'; 'pos430', 'neg430'};
fprintf('%6s | %7s %7s | %7s %8s | %10s\n', 'level', 'u+', 'u-', 'u_sym', 'bias b', '0からのstep由来u*');
for k = 1:size(pairs, 1)
    ip = find(strcmp(runs(:, 1), pairs{k, 1})); in_ = find(strcmp(runs(:, 1), pairs{k, 2}));
    up = res(ip).m.u_ss; un = res(in_).m.u_ss;
    fprintf('%6.0f | %7.4f %7.4f | %7.4f %8.4f | %10.4f\n', abs(runs{ip, 2}), up, un, (up - un) / 2, (up + un) / 2, res(ip).m.sim_u_ss);
end

%% リップルの詳細（+430 と -430）：スペクトルと時系列
fig2 = figure('Position', [50 50 1300 700]);
for k = 1:2
    nm = {'pos430', 'neg430'}; i = find(strcmp(runs(:, 1), nm{k}));
    R = res(i); st = R.tr >= 0.3;
    gg = R.g(st) - mean(R.g(st)); Nf = numel(gg); w = 0.5 - 0.5 * cos(2 * pi * (0:Nf - 1)' / (Nf - 1));
    Y = abs(fft(gg .* w)) / sum(w) * 2; f = (0:Nf - 1)' / (Nf * 1e-3);
    subplot(2, 2, k); plot(R.tr(st) * 1000, R.g(st)); grid on; title([nm{k} ' 定常区間']); xlabel('t [ms]'); ylabel('\omega [dps]');
    subplot(2, 2, 2 + k); plot(f(f <= 100), Y(f <= 100)); grid on; title([nm{k} ' スペクトル']); xlabel('f [Hz]'); ylabel('amp [dps]');
end
exportgraphics(fig2, fullfile(results_dir, 'omega_step_e3_ripple.png'));


%% 遅い振れの再現性（+400/+430）：100ms移動平均の応答を重ねて，タイミング・大きさが再現するかを見る
fprintf('\n===== 高速(400/430)の振れ（符号を揃えた100ms移動平均, t>=0.25s の目標からの偏差） =====\n');
fprintf('%-9s %2s %5s | %8s %8s %8s | %6s %6s | %8s %8s\n', 'file', 'fw', 'tgt', 'dev_max%', 'dev_min%', 'p2p%', 't_max', 't_min', 'corr(v)', 'vdrop%');
fig3 = figure('Position', [50 50 1400 800]);
groups = {[400], [430]};
for gi = 1:2
    subplot(2, 2, gi); hold on; grid on; title(sprintf('|%d| dps 応答（符号を揃えた100ms平均）', groups{gi})); xlabel('t [ms]'); ylabel('\omega [dps]');
    subplot(2, 2, 2 + gi); hold on; grid on; title(sprintf('+%d dps 並進速度（100ms平均）', groups{gi})); xlabel('t [ms]'); ylabel('v [mm/s]');
end
for r = 1:nr
    m = res(r).m;
    if abs(m.target) ~= 400 && abs(m.target) ~= 430, continue; end
    R = res(r); st = R.tr >= 0.25;
    d = sign(m.target) * (movmean(R.g, 100) - m.target) / abs(m.target) * 100;
    dv = movmean(R.vavg, 100);
    [dmax, imx] = max(d(st)); [dmin, imn] = min(d(st));
    trs = R.tr(st);
    c = corrcoef(d(st), dv(st));
    fprintf('%-9s %2d %5.0f | %8.1f %8.1f %8.1f | %6.0f %6.0f | %8.2f %8.1f\n', m.name, m.fw, m.target, dmax, dmin, dmax - dmin, ...
        trs(imx) * 1000, trs(imn) * 1000, c(1, 2), m.v_drop_pct);
    gi = (abs(m.target) == 430) + 1;
    subplot(2, 2, gi); plot(R.tr * 1000, sign(m.target) * movmean(R.g, 100), 'DisplayName', strrep(m.name, '_', '\_'));
    subplot(2, 2, 2 + gi); plot(R.tr * 1000, dv, 'DisplayName', strrep(m.name, '_', '\_'));
end
for gi = 1:2
    subplot(2, 2, gi); tv = [400 430]; yline(tv(gi), 'k:', 'HandleVisibility', 'off'); legend('Location', 'southeast', 'FontSize', 7);
    subplot(2, 2, 2 + gi); legend('Location', 'southeast', 'FontSize', 7);
end
exportgraphics(fig3, fullfile(results_dir, 'omega_step_e5_repeatability.png'));

Sm = struct2table(arrayfun(@(x) x.m, res));
writetable(Sm, fullfile(results_dir, 'omega_step_e3_summary.csv'));

%% 高周波リップル（>約10Hz）の切り分け：閉ループ(PI) vs 開ループ(0からのduty step)
% 10Hz超の成分だけを見るため，100ms移動平均を引く。閉ループuが同周波数で動いているか（制御器が励起/増幅していないか）も見る。
hp = @(x) x - movmean(x, 100);
domf = @(x) dominant_freq(x, 20, 80);
fprintf('\n===== 高周波リップル（100ms移動平均を除去した成分） =====\n');
fprintf('%-12s %6s | %8s %7s %6s | %8s\n', 'run', 'level', 'gyro_hp', '%level', 'f[Hz]', 'u_hp');
for r = 1:nr
    R = res(r); m = R.m; st = R.tr >= 0.4;
    gh = hp(R.g); uh = hp(R.u);
    fprintf('%-12s %6.0f | %8.1f %7.1f %6.1f | %8.4f\n', ['CL ' m.name], abs(m.target), std(gh(st)), 100 * std(gh(st)) / abs(m.target), domf(gh(st)), std(uh(st)));
end
ol = {'pos020', 'pos022_long', 'pos024_long', 'pos026_long', 'pos028', 'neg020', 'neg022_long', 'neg024_long', 'neg026_long', 'neg028'};
for i = 1:numel(ol)
    T = readtable(sprintf('../../tools/log/rot_step_v700_x/rot_step_v700_duty_%s.csv', ol{i}), 'VariableNamingRule', 'modify');
    t = T.Global_time - T.Global_time(1);
    nz = T.duty_diff ~= 0; d = diff([0; nz; 0]); rs = find(d == 1); re = find(d == -1) - 1; [~, k] = max(re - rs); on = rs(k); off = re(k);
    tt = t(on:off) - t(on); gg = T.gyro_z(on:off); st = tt >= 0.4;
    gh = hp(gg);
    lvl = abs(mean(gg(st)));
    fprintf('%-12s %6.0f | %8.1f %7.1f %6.1f | %8s\n', ['OL ' ol{i}], lvl, std(gh(st)), 100 * std(gh(st)) / lvl, domf(gh(st)), '-');
end

function f0 = dominant_freq(x, flo, fhi)
    x = x - mean(x); N = numel(x);
    w = 0.5 - 0.5 * cos(2 * pi * (0:N - 1)' / (N - 1));
    Y = abs(fft(x .* w)); f = (0:N - 1)' / (N * 1e-3);
    b = f >= flo & f <= fhi; fb = f(b); [~, i] = max(Y(b)); f0 = fb(i);
end
