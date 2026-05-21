clear; clc; close all;

%% ============================================================
% SELF-BALANCING ROBOT: LQR vs H-infinity
% One-input formulation for balancing:
% u_b = v_l + v_r
% v_l = u_b/2, v_r = u_b/2
%
% State:
% x = [theta; psi; theta_dot; psi_dot]
% ============================================================

%% ============================================================
% 1. Nominal self-balancing subsystem
% ============================================================

A = [0 0 1 0;
    0 0 0 1;
    0 55.540 -0.610 0.610;
    0 62.794 -0.316 0.316];

% Original paper has two identical input columns:
B2 = [0      0;
    0      0;
    9.385  9.385;
    -4.857 -4.857];

% Effective one-input balancing model:
% Since v_l = v_r = u_b/2, the net effect is same as one column times u_b.
B = B2(:,1);

C = eye(4);
D = zeros(4,1);

G = ss(A,B,C,D);

nx = size(A,1);

disp('================ OPEN LOOP ================')
disp('Open-loop poles:')
disp(eig(A))

disp('Controllability rank:')
disp(rank(ctrb(A,B)))

disp('Observability rank:')
disp(rank(obsv(A,C)))

%% ============================================================
% 2. LQR controller
% ============================================================

Q_lqr = diag([10000 1 10000 1]);
R_lqr = 200;

K_lqr = lqr(A,B,Q_lqr,R_lqr);

Acl_lqr = A - B*K_lqr;

disp('================ LQR ================')
disp('LQR gain K_lqr:')
disp(K_lqr)

disp('LQR closed-loop poles:')
disp(eig(Acl_lqr))

sys_lqr_ic = ss(Acl_lqr,[],eye(4),[]);

%% ============================================================
% 3. H-infinity mixed-sensitivity design
%
% mixsyn solves:
% min || [Ws*S; Wu*K*S; Wt*T] ||_inf
%
% S  = sensitivity
% KS = control effort channel
% T  = complementary sensitivity
% ============================================================

s = tf('s');

% -----------------------------
% Sensitivity weight Ws
% -----------------------------
Ms  = 3;       % maximum allowed sensitivity peak
As  = 0.15;      % steady-state error target
wbs = 4;         % desired bandwidth rad/s

Ws_base = (s/Ms + wbs)/(s + wbs*As);

% Penalize pitch angle most strongly.
% State order: theta, psi, theta_dot, psi_dot
Ws = blkdiag(0.01*Ws_base, ...
    0.1*Ws_base, ...
    0.005*Ws_base, ...
    0.08*Ws_base);

% -----------------------------
% Control effort weight Wu
% -----------------------------
umax = 24;       % voltage limit from paper-style comparison
Wu = tf(1/(umax * 4));

% -----------------------------
% Complementary sensitivity weight Wt
% Penalizes high-frequency response/noise amplification
Mt = 3;
At = 5e-2;
wbt = 50;

Wt_scalar = (s + wbt/Mt)/(At*s + wbt);

Wt = 0.01*blkdiag(Wt_scalar, ...
    Wt_scalar, ...
    Wt_scalar, ...
    Wt_scalar);

%% ============================================================
% 4. H-infinity synthesis using mixsyn
% ============================================================
%% TODO: EXPLAIN THE CHOICE OF WEIGHTS.
[K_hinf, CL_hinf_weighted, gamma_hinf] = mixsyn(G,Ws,Wu,Wt); 
disp('================ H-INFINITY ================')
disp('H-infinity gamma:')
disp(gamma_hinf)

disp('H-infinity controller order:')
disp(order(K_hinf))

%% Physical closed-loop for simulation
% Negative feedback: u = -K*y
CL_hinf_ic = feedback(G,K_hinf);

disp('H-infinity physical closed-loop poles:')
disp(pole(CL_hinf_ic))

if any(real(pole(CL_hinf_ic)) >= 0)
    warning('H-infinity closed-loop is unstable. Tune weights before trusting plots.');
end

%% ============================================================
% 5. Experiment 1 simulation: 10 seconds
% Initial condition from paper-style balancing experiment
% x0 = [theta; psi; theta_dot; psi_dot]
% ============================================================

t = 0:0.001:10;

x0_robot = [0;
    0.1;
    0.3;
    0.3];

%% LQR response
[~,t_lqr,x_lqr] = initial(sys_lqr_ic,x0_robot,t);

% Control input u_b = -Kx
u_lqr = -x_lqr*K_lqr';

% Motor voltages
vl_lqr = u_lqr/2;
vr_lqr = u_lqr/2;

%% H-infinity response
x0_ctrl = zeros(order(K_hinf),1);
x0_aug  = [x0_robot; x0_ctrl];

[y_hinf,t_hinf,x_aug_hinf] = initial(CL_hinf_ic,x0_aug,t);

% Controller output from y = x
u_hinf_raw = lsim(K_hinf,y_hinf,t_hinf);

% Negative feedback convention: physical control is -K*y
u_hinf = -u_hinf_raw;

% Motor voltages
vl_hinf = u_hinf/2;
vr_hinf = u_hinf/2;

%% ============================================================
% 6. Figure-style comparison: LQR vs Hinf
% ============================================================

figure('Name','Experiment 1: LQR vs Hinf','Color','w');

subplot(3,2,1)
plot(t_lqr,x_lqr(:,1),'LineWidth',1.4); hold on;
plot(t_hinf,y_hinf(:,1),'LineWidth',1.4);
grid on;
title('(a) Average wheel angle')
xlabel('Time (s)')
ylabel('\theta (rad)')
legend('LQR','H_\infty','Location','best')

subplot(3,2,2)
plot(t_lqr,x_lqr(:,3),'LineWidth',1.4); hold on;
plot(t_hinf,y_hinf(:,3),'LineWidth',1.4);
grid on;
title('(b) Average wheel angular velocity')
xlabel('Time (s)')
ylabel('\dot{\theta} (rad/s)')
legend('LQR','H_\infty','Location','best')

subplot(3,2,3)
plot(t_lqr,x_lqr(:,2),'LineWidth',1.4); hold on;
plot(t_hinf,y_hinf(:,2),'LineWidth',1.4);
grid on;
title('(c) Body pitch angle')
xlabel('Time (s)')
ylabel('\psi (rad)')
legend('LQR','H_\infty','Location','best')

subplot(3,2,4)
plot(t_lqr,x_lqr(:,4),'LineWidth',1.4); hold on;
plot(t_hinf,y_hinf(:,4),'LineWidth',1.4);
grid on;
title('(d) Body pitch angular velocity')
xlabel('Time (s)')
ylabel('\dot{\psi} (rad/s)')
legend('LQR','H_\infty','Location','best')

subplot(3,2,5)
plot(t_lqr,vl_lqr,'LineWidth',1.4); hold on;
plot(t_hinf,vl_hinf,'LineWidth',1.4);
yline(24,'r--','+24 V');
yline(-24,'r--','-24 V');
grid on;
title('(e) Left motor input')
xlabel('Time (s)')
ylabel('v_l (V)')
legend('LQR','H_\infty','Location','best')

subplot(3,2,6)
plot(t_lqr,vr_lqr,'LineWidth',1.4); hold on;
plot(t_hinf,vr_hinf,'LineWidth',1.4);
yline(24,'r--','+24 V');
yline(-24,'r--','-24 V');
grid on;
title('(f) Right motor input')
xlabel('Time (s)')
ylabel('v_r (V)')
legend('LQR','H_\infty','Location','best')

%% ============================================================
% 7. Sensitivity analysis for Hinf design
% ============================================================

L_hinf = G*K_hinf;
S_hinf = feedback(eye(4),L_hinf);
T_hinf = feedback(L_hinf,eye(4));

figure('Name','Hinf Sensitivity','Color','w');
sigma(S_hinf,T_hinf,{1e-2,1e3});
grid on;
legend('S','T');
title('H_\infty Sensitivity and Complementary Sensitivity');

figure('Name','Weighted Channels','Color','w');
bodemag(Ws*S_hinf,{1e-2,1e3}); hold on;
bodemag(Wu*K_hinf*S_hinf,{1e-2,1e3});
bodemag(Wt*T_hinf,{1e-2,1e3});
grid on;
legend('W_s S','W_u K S','W_t T');
title('Weighted Mixed-Sensitivity Channels');


%% ============================================================
% 8. Summary metrics
% ============================================================

fprintf('\n================ SUMMARY METRICS ================\n');

fprintf('LQR max |psi|      = %.4f rad\n', max(abs(x_lqr(:,2))));
fprintf('Hinf max |psi|     = %.4f rad\n', max(abs(y_hinf(:,2))));

fprintf('LQR max |u_b|      = %.4f V\n', max(abs(u_lqr)));
fprintf('Hinf max |u_b|     = %.4f V\n', max(abs(u_hinf)));

fprintf('LQR max |v_l|      = %.4f V\n', max(abs(vl_lqr)));
fprintf('Hinf max |v_l|     = %.4f V\n', max(abs(vl_hinf)));

fprintf('Hinf gamma         = %.4f\n', gamma_hinf);

%% ============================================================
% 9. Optional: save figures
% ============================================================

% saveas(figure(1),'experiment1_lqr_vs_hinf.png');