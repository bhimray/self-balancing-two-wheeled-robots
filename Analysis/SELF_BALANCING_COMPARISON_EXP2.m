clear; clc; close all;

%% ============================================================
% EXPERIMENT 2: DISTURBANCE REJECTION
% SELF-BALANCING ROBOT: LQR vs H-infinity
%
% One-input formulation for balancing:
% u_b = v_l + v_r
% v_l = u_b/2, v_r = u_b/2
%
% State:
% x = [theta; psi; theta_dot; psi_dot]
%
% Disturbance:
% xdot = A*x + B*u + Bd*f(t)
% Bd = [0;0;1;1]
% f(t) = 0.3*sin(t)
% ============================================================

%% ============================================================
% 1. Nominal self-balancing subsystem
% ============================================================

A = [0 0 1 0;
    0 0 0 1;
    0 55.540 -0.610 0.610;
    0 62.794 -0.316 0.316];

B2 = [0      0;
    0      0;
    9.385  9.385;
    -4.857 -4.857];

% Effective one-input balancing model
B = B2(:,1);

% Disturbance input matrix
Bd = [0;
    0;
    1;
    1];

C = eye(4);
D = zeros(4,1);

G = ss(A,B,C,D);

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
R_lqr = 500;

K_lqr = lqr(A,B,Q_lqr,R_lqr);
Acl_lqr = A - B*K_lqr;

disp('================ LQR ================')
disp('LQR gain K_lqr:')
disp(K_lqr)

disp('LQR closed-loop poles:')
disp(eig(Acl_lqr))

%% ============================================================
% 3. H-infinity mixed-sensitivity design
% ============================================================

s = tf('s');

% Sensitivity weight Ws
Ms  = 3;
As  = 0.15;
wbs = 4;

Ws_base = (s/Ms + wbs)/(s + wbs*As);

% State order: theta, psi, theta_dot, psi_dot
Ws = blkdiag(0.01*Ws_base, ...
    0.1*Ws_base, ...
    0.005*Ws_base, ...
    0.08*Ws_base);

% Control effort weight Wu
umax = 24;
Wu = tf(1/(umax*4));

% Complementary sensitivity weight Wt
Mt  = 3;
At  = 5e-2;
wbt = 50;

Wt_scalar = (s + wbt/Mt)/(At*s + wbt);

Wt = 0.01*blkdiag(Wt_scalar, ...
    Wt_scalar, ...
    Wt_scalar, ...
    Wt_scalar);

%% ============================================================
% 4. H-infinity synthesis using mixsyn
% ============================================================

[K_hinf, CL_hinf_weighted, gamma_hinf] = mixsyn(G,Ws,Wu,Wt);

disp('================ H-INFINITY ================')
disp('H-infinity gamma:')
disp(gamma_hinf)

disp('H-infinity controller order:')
disp(order(K_hinf))

% Physical closed-loop from plant output y=x to controller
CL_hinf_ic = feedback(G,K_hinf);

disp('H-infinity physical closed-loop poles:')
disp(pole(CL_hinf_ic))

if any(real(pole(CL_hinf_ic)) >= 0)
    warning('H-infinity closed-loop is unstable. Tune weights before trusting plots.');
end

%% ============================================================
% 5. Experiment 2 simulation setup
% ============================================================

t = 0:0.001:10;

% Same initial condition used in Experiment 1
x0_robot =[0;
    0.1;
    0.3;
    0.3];

% Disturbance from paper-style Experiment 2
f = 0.3*sin(t);

%% ============================================================
% 6. LQR disturbance response
%
% xdot = (A-BK)x + Bd*f(t)
% ============================================================

sys_lqr_dist = ss(Acl_lqr, Bd, eye(4), zeros(4,1));

[y_lqr,t_lqr,x_lqr] = lsim(sys_lqr_dist, f, t, x0_robot);

% LQR control input u_b = -Kx
u_lqr = -x_lqr*K_lqr';

% Split motor voltages
vl_lqr = u_lqr/2;
vr_lqr = u_lqr/2;

%% ============================================================
% 7. H-infinity disturbance response
%
% The Hinf controller is dynamic, so we create an augmented closed-loop
% system and inject Bd*f(t) only into the physical plant states.
% ============================================================

Acl_hinf = CL_hinf_ic.A;
Ccl_hinf = CL_hinf_ic.C;
Dcl_hinf = CL_hinf_ic.D;

ncl = size(Acl_hinf,1);

% Disturbance affects only the first 4 physical robot states
Bd_aug = [Bd;
    zeros(ncl-4,1)];

% Initial augmented state = robot initial condition + controller states
x0_ctrl = zeros(ncl-4,1);
x0_aug  = [x0_robot;
    x0_ctrl];

sys_hinf_dist = ss(Acl_hinf, Bd_aug, Ccl_hinf, Dcl_hinf);

[y_hinf,t_hinf,x_aug_hinf] = lsim(sys_hinf_dist, f, t, x0_aug);

% Compute Hinf controller output from y = x
u_hinf_raw = lsim(K_hinf, y_hinf, t_hinf);

% Negative feedback convention: physical control is -K*y
u_hinf = -u_hinf_raw;

% Split motor voltages
vl_hinf = u_hinf/2;
vr_hinf = u_hinf/2;

%% ============================================================
% 8. Plot external disturbance
% ============================================================

figure('Name','Experiment 2 Disturbance','Color','w');
plot(t,f,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('f(t)');
title('External Disturbance: f(t)=0.3sin(t)');

%% ============================================================
% 9. Experiment 2 comparison plots
% ============================================================

figure('Name','Experiment 2: Disturbance Rejection LQR vs Hinf','Color','w');

subplot(3,2,1)
plot(t_lqr,y_lqr(:,1),'LineWidth',1.4); hold on;
plot(t_hinf,y_hinf(:,1),'LineWidth',1.4);
grid on;
title('(a) Average wheel angle')
xlabel('Time (s)')
ylabel('\theta (rad)')
legend('LQR','H_\infty','Location','best')

subplot(3,2,2)
plot(t_lqr,y_lqr(:,3),'LineWidth',1.4); hold on;
plot(t_hinf,y_hinf(:,3),'LineWidth',1.4);
grid on;
title('(b) Average wheel angular velocity')
xlabel('Time (s)')
ylabel('\dot{\theta} (rad/s)')
legend('LQR','H_\infty','Location','best')

subplot(3,2,3)
plot(t_lqr,y_lqr(:,2),'LineWidth',1.4); hold on;
plot(t_hinf,y_hinf(:,2),'LineWidth',1.4);
grid on;
title('(c) Body pitch angle')
xlabel('Time (s)')
ylabel('\psi (rad)')
legend('LQR','H_\infty','Location','best')

subplot(3,2,4)
plot(t_lqr,y_lqr(:,4),'LineWidth',1.4); hold on;
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
% 10. Sensitivity analysis for Hinf design
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
% 11. Summary metrics
% ============================================================

fprintf('\n================ EXPERIMENT 2 SUMMARY METRICS ================\n');

fprintf('Hinf gamma         = %.4f\n', gamma_hinf);

fprintf('\n----- LQR -----\n');
fprintf('LQR max |theta|    = %.4f rad\n', max(abs(y_lqr(:,1))));
fprintf('LQR max |psi|      = %.4f rad\n', max(abs(y_lqr(:,2))));
fprintf('LQR max |u_b|      = %.4f V\n', max(abs(u_lqr)));
fprintf('LQR max |v_l|      = %.4f V\n', max(abs(vl_lqr)));

fprintf('\n----- Hinf -----\n');
fprintf('Hinf max |theta|   = %.4f rad\n', max(abs(y_hinf(:,1))));
fprintf('Hinf max |psi|     = %.4f rad\n', max(abs(y_hinf(:,2))));
fprintf('Hinf max |u_b|     = %.4f V\n', max(abs(u_hinf)));
fprintf('Hinf max |v_l|     = %.4f V\n', max(abs(vl_hinf)));