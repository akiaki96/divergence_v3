%% 回転角速度PI制御の閉ループシミュレーション（動作点依存プラント上）
%
% rot_step_v700_p1fit.m の平滑化モデル（results/rot_step_v700_p1fit_model_params.csv）から
% プラントを構成する：
%     d(omega)/dt = ( Kmap(u) - omega ) / Tmap(u)
% 0からのstepではrot_step_v700の実測step応答を再現し，静特性は Kmap(u) に一致する。
% 定常点まわりの局所ゲインは接線ゲインKtan(u)，局所時定数はTmap(u)となる。
%
% 比較する制御器（firmwareのPIDControllerと同一構造：back-calculation，FFなし，1kHz）:
%   A) 現行  : Kc=Tp1/(Kp*lambda)=3.7e-4, Ti=Tp1=23.1ms 固定, duty_diff上限0.20
%   B) 現行ゲインのまま上限を0.28に拡大
%   C) スケジュール: 目標角速度から動作点u*を逆算し Ti=T(u*), Kc=1/(lambda*Ktan(u*)/T(u*))
% 外乱・不確かさ: ゲイン倍率(1.0/1.5/2.0)，測定ノイズ(sigma=15dps, 1サンプル遅れ)

clear; clc;
results_dir = 'results';

pr = readtable(fullfile(results_dir, 'rot_step_v700_p1fit_model_params.csv'));
mapA = pr.K_a; mapB = pr.K_b; mapN = pr.K_n; mapT0 = pr.T_T0; mapC = pr.T_c; mapM = pr.T_m;
Kmap  = @(x) sign(x) .* (mapA * abs(x) + mapB * abs(x) .^ mapN);
Ktanf = @(x) mapA + mapN * mapB * abs(x) .^ (mapN - 1);
Tmap  = @(x) mapT0 + mapC * abs(x) .^ mapM;

ug = linspace(0, 0.30, 3001);
Kg = Kmap(ug);
u_of_omega = @(w) interp1(Kg, ug, abs(w), 'pchip', ug(end)) .* sign(w);   % 目標角速度→必要duty_diff

% 現行firmware値（config::pid_omega）
KP_ROT = 1250; TP1 = 0.0231; LAMBDA = 0.05;
Kc0 = TP1 / (KP_ROT * LAMBDA);

Ts = 1e-3; Tend = 1.5; N = round(Tend / Ts);
sigma = 15;              % 測定ノイズ [dps]
rng(1);

targets = [100 200 250 400 430 -430];
gainmult = [1.0 1.5 2.0];    % プラント静ゲインの倍率（局所ゲイン増の不確かさ試験）

LAMBDA_FAST = 0.03;
ctrl_names = {'A 現行(上限0.20)', 'B 現行ゲイン(上限0.28)', 'C スケジュール(lam=0.05)', 'D スケジュール(lam=0.03,上限0.26)', 'E ファーム案(Kc一定,Ti表,上限0.28)'};
nC = 5;
% ファーム案E（現行firmware）：Kc一定，Tiは|目標角速度|の区分線形表（rot_gain_scheduling_plan.md §6）
KC_E = 5.0e-4;
TI_BP_W = [0 100 200 250 400 430];
TI_BP_T = [0.0090 0.01725 0.0345 0.0420 0.0615 0.06525];   % firmware(config::pid_omega::TI_S_BP)と同一

rows = {};
fig = figure('Position', [50 50 1300 900]);
for it = 1:numel(targets)
    wref = targets(it);
    for ic = 1:nC
        for ig = 1:numel(gainmult)
            gm = gainmult(ig);
            switch ic
                case 1, ulim = 0.20; Kc = Kc0; Ti = TP1;
                case 2, ulim = 0.28; Kc = Kc0; Ti = TP1;
                case 5
                    ulim = 0.28;
                    Kc = KC_E;
                    Ti = interp1(TI_BP_W, TI_BP_T, min(abs(wref), 430));
                case {3, 4}
                    ulim = 0.28;
                    lam = LAMBDA;
                    if ic == 4, lam = LAMBDA_FAST; ulim = 0.26; end
                    ustar = abs(u_of_omega(wref));
                    Ti = Tmap(ustar);
                    Kc = 1 / (lam * Ktanf(ustar) / Tmap(ustar));
            end
            Tt = Ti;

            w = 0; I = 0; wmeas_prev = 0;
            wlog = zeros(N, 1); ulog = zeros(N, 1); satlog = false(N, 1);
            noise = sigma * randn(N, 1);
            for k = 1:N
                e = wref - wmeas_prev;
                u_unsat = Kc * e + I;
                u = max(min(u_unsat, ulim), -ulim);
                I = I + (Kc / Ti * e + (u - u_unsat) / Tt) * Ts;

                Tp = Tmap(u);
                w = w + Ts * (gm * Kmap(u) - w) / Tp;
                wmeas_prev = w + noise(k);
                wlog(k) = w; ulog(k) = u; satlog(k) = abs(u_unsat) > ulim;
            end
            t = (0:N - 1)' * Ts;

            % 指標
            wf = mean(wlog(round(0.8 * N):end));
            ov = max(0, (max(sign(wref) * wlog) - abs(wref)) / abs(wref)) * 100;
            tol = 0.05 * abs(wref);
            idx_out = find(abs(wlog - wref) > tol, 1, 'last');
            if isempty(idx_out), ts = 0; elseif idx_out >= N, ts = NaN; else, ts = t(idx_out + 1) * 1000; end
            reached = find(sign(wref) * wlog >= 0.9 * abs(wref), 1, 'first');
            if isempty(reached), tr = NaN; else, tr = t(reached) * 1000; end
            ripple = std(ulog(round(0.8 * N):end));
            rows(end + 1, :) = {wref, ic, gm, wf, ov, tr, ts, max(abs(ulog)), 100 * mean(satlog), ripple}; %#ok<SAGROW>

            if gm == 1.0 && (wref == 250 || wref == 430)
                sp = (wref == 430) * 2 + 1;
                subplot(2, 2, sp); plot(t * 1000, wlog, 'DisplayName', ctrl_names{ic}); hold on;
                subplot(2, 2, sp + 1); plot(t * 1000, ulog, 'DisplayName', ctrl_names{ic}); hold on;
            end
        end
    end
end
subplot(2, 2, 1); yline(250, 'k:'); ylabel('\omega [dps]'); title('目標+250dps'); legend('Location', 'southeast'); grid on; xlabel('t [ms]');
subplot(2, 2, 2); ylabel('duty\_diff'); title('制御出力（+250dps）'); grid on; xlabel('t [ms]');
subplot(2, 2, 3); yline(430, 'k:'); ylabel('\omega [dps]'); title('目標+430dps'); legend('Location', 'southeast'); grid on; xlabel('t [ms]');
subplot(2, 2, 4); ylabel('duty\_diff'); title('制御出力（+430dps）'); grid on; xlabel('t [ms]');
exportgraphics(fig, fullfile(results_dir, 'rot_omega_pi_sim.png'));
savefig(fig, fullfile(results_dir, 'rot_omega_pi_sim.fig'));

R = cell2table(rows, 'VariableNames', {'target_dps', 'ctrl', 'gain_mult', 'final_dps', 'overshoot_pct', ...
    'rise90_ms', 'settle5_ms', 'u_peak', 'sat_time_pct', 'u_ripple_std'});
writetable(R, fullfile(results_dir, 'rot_omega_pi_sim_summary.csv'));

fprintf('Kc0(現行)=%.3g, Ki0=%.3g, Ti0=%.1fms\n\n', Kc0, Kc0 / TP1, TP1 * 1000);
fprintf('%-8s %-26s %5s %9s %7s %8s %9s %7s %7s\n', 'target', 'ctrl', 'gain', 'final', 'OS%', 'rise90', 'settle5%', 'u_pk', 'sat%');
for r = 1:height(R)
    fprintf('%-8d %-26s %5.1f %9.1f %7.1f %8.0f %9.0f %7.3f %7.1f\n', R.target_dps(r), ctrl_names{R.ctrl(r)}, ...
        R.gain_mult(r), R.final_dps(r), R.overshoot_pct(r), R.rise90_ms(r), R.settle5_ms(r), R.u_peak(r), R.sat_time_pct(r));
end
fprintf('\n--- スケジュール後の各動作点のゲイン ---\n');
for wr = [100 200 250 400 430]
    us_ = abs(u_of_omega(wr));
    fprintf('target %3d dps: u*=%.3f, T=%.1fms, Ktan=%.0f, Kc=%.3g, Ti=%.1fms, Ki=%.3g\n', wr, us_, Tmap(us_) * 1000, Ktanf(us_), ...
        1 / (LAMBDA * Ktanf(us_) / Tmap(us_)), Tmap(us_) * 1000, 1 / (LAMBDA * Ktanf(us_) / Tmap(us_)) / Tmap(us_));
end
