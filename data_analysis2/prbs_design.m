%% PRBS入力設計（並進方向）
%
% step_analyze.m で得た duty 10%/15%/20% の同定結果(Kp, Tp1, 不感帯電圧u0)から，
% PRBS本同定実験のクロック周期Tc・duty範囲を設計し，距離制約（system_identification_flow.md
% に基づく実機制約：1試行最大3秒・使用可能空間4〜5m）を事前シミュレーションで検証する。
%
% 設計方針（system_identification_flow.md §1 [3] PRBS設計）:
%  - クロック周期Tc：最速点（最小Tp1）の時定数から Tc ≲ τ_fast/2.8 を目安に決定
%  - 段数n：ハードウェアLFSR実装済み値 n=8 を使用。最遅点（最大Tp1）に対し
%    最長パルス幅 nTc ≳ 2.5τ_slow を満たすことを確認
%  - duty範囲：不感帯電圧u0から十分マージンを確保（目安 u_min ≳ 3〜5×u0）
%  - 上記パラメータでの走行距離を，最大ゲイン点のモデルで保守的にシミュレーション検証

clear; clc;

results_dir = 'results';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% step_analyze.m の同定結果（data_analysis2/results/step_identification_summary.csv）
summary = readtable(fullfile(results_dir, 'step_identification_summary.csv'));
final   = readtable(fullfile(results_dir, 'step_identification_final.csv'));

duty_pct = summary.duty_pct;
Kp       = summary.Kp;
Tp1      = summary.Tp1_s;
u0       = final.u0_mc_mean;

[~, i_fast] = min(Tp1);   % 最速点（最小Tp1）：Tc決定に使用
[~, i_slow] = max(Tp1);   % 最遅点（最大Tp1）：カバレッジ確認に使用
[~, i_maxK] = max(Kp);    % 最大ゲイン点：距離の保守的な上限評価に使用

fprintf('最速点: duty%d%% (Tp1=%.4fs)\n', duty_pct(i_fast), Tp1(i_fast));
fprintf('最遅点: duty%d%% (Tp1=%.4fs)\n', duty_pct(i_slow), Tp1(i_slow));
fprintf('最大ゲイン点: duty%d%% (Kp=%.1f) ※距離の保守的評価に使用\n', duty_pct(i_maxK), Kp(i_maxK));
fprintf('不感帯電圧 u0 = %.4f V\n', u0);

%% クロック周期Tcの決定
Tc_upper = Tp1(i_fast) / 2.8;
Tc = 0.135;   % 採用値

fprintf('\nTc上限 (τ_fast/2.8) = %.4f s\n', Tc_upper);
fprintf('採用 Tc = %.3f s\n', Tc);
assert(Tc <= Tc_upper, 'Tcが速い極に対して粗すぎます（Tc <= τ_fast/2.8を満たしません）');

%% LFSR段数(n=8, 実装済み)でのカバレッジ確認
n = 8;
pulse_max = n * Tc;
coverage_target = 2.5 * Tp1(i_slow);

fprintf('\n最長パルス幅 nTc = %.3f s (n=%d)\n', pulse_max, n);
fprintf('目安 (2.5×τ_slow) = %.3f s\n', coverage_target);
if pulse_max >= coverage_target
    fprintf('→ 満たす（最遅点の定常応答に十分近づける）\n');
else
    fprintf('→ 不足（Tcを大きくするか、段数nの拡張を検討）\n');
end

%% duty範囲の決定（不感帯マージン）
% u_step = duty×Vbatt の実測平均から公称バッテリ電圧を逆算
Vbatt_nom = mean(summary.u_step_V ./ (duty_pct / 100));
fprintf('\n実測から逆算した公称バッテリ電圧 Vbatt_nom = %.3f V\n', Vbatt_nom);

duty_min = 0.08;
duty_max = 0.16;
u_min = duty_min * Vbatt_nom;
u_max = duty_max * Vbatt_nom;

fprintf('duty_min=%.2f → u_min=%.3fV (u0比 %.1fx)\n', duty_min, u_min, u_min / u0);
fprintf('duty_max=%.2f → u_max=%.3fV\n', duty_max, u_max);

%% 事前シミュレーション：距離制約の検証（最大ゲイン点のモデルで保守的に評価）
Ts_sim  = 0.001;
T_trial = 3.0;   % 1試行の最大走行時間 [s]

Nclock = round(Tc / Ts_sim);
Nbits  = round(T_trial / Tc);
prbs   = idinput(Nbits * Nclock, 'prbs', [0 1 / Nclock], [-1 1]);
duty_seq = duty_min + (prbs + 1) / 2 * (duty_max - duty_min);
u_seq    = duty_seq * Vbatt_nom;

G = tf(Kp(i_maxK), [Tp1(i_maxK) 1]);

t_sim    = (0:numel(u_seq) - 1)' * Ts_sim;
v_sim    = lsim(G, u_seq, t_sim);
dist_cum = cumtrapz(t_sim, v_sim);

fprintf('\n--- 距離シミュレーション結果（duty%d%%モデルで保守的に評価） ---\n', duty_pct(i_maxK));
fprintf('最大速度: %.2f m/s\n', max(v_sim) / 1000);
fprintf('%.1fs後の総走行距離: %.2f m\n', T_trial, dist_cum(end) / 1000);

fig = figure;
subplot(2, 1, 1);
plot(t_sim, u_seq);
ylabel('u [V]'); title(sprintf('PRBS入力設計案 (Tc=%.3fs, duty%.0f-%.0f%%)', Tc, duty_min * 100, duty_max * 100));
grid on;

subplot(2, 1, 2);
yyaxis left;
plot(t_sim, v_sim / 1000);
ylabel('v [m/s]');
yyaxis right;
plot(t_sim, dist_cum / 1000);
ylabel('距離 [m]');
xlabel('t [s]');
grid on;

savefig(fig, fullfile(results_dir, 'prbs_design_check.fig'));
exportgraphics(fig, fullfile(results_dir, 'prbs_design_check.png'));

%% 設計パラメータの保存
design = table(Tc, n, duty_min, duty_max, Vbatt_nom, T_trial, ...
    max(v_sim) / 1000, dist_cum(end) / 1000, ...
    'VariableNames', {'Tc_s', 'n_stages', 'duty_min', 'duty_max', 'Vbatt_nom_V', 'T_trial_s', ...
    'v_max_m_s', 'dist_3s_m'});
writetable(design, fullfile(results_dir, 'prbs_design_params.csv'));
