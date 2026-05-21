clear; clc; close all;

%% ============================================================
% 1. Nominal self-balancing subsystem
% x = [theta; psi; theta_dot; psi_dot]
% u = [v_l; v_r]
% ============================================================

A = [0 0 1 0;
    0 0 0 1;
    0 55.540 -0.610 0.610;
    0 62.794 -0.316 0.316];

B = [0      0;
    0      0;
    9.385  9.385;
    -4.857 -4.857];

C = eye(4);
D = zeros(4,2);

G = ss(A,B,C,D);

%% ============================================================
% 2. Nominal LQR baseline
% ============================================================

Q_lqr = diag([100 1000 10 10]);
R_lqr = eye(2);

K_lqr = lqr(A,B,Q_lqr,R_lqr);
Acl_lqr = A - B*K_lqr;

disp('LQR gain:')
disp(K_lqr)

disp('LQR poles:')
disp(eig(Acl_lqr))

%% ============================================================
% 3. Mixed-sensitivity H-infinity weights
%
% mixsyn minimizes:
% || [ Ws*S ; Wu*K*S ; Wt*T ] ||_inf
%
% S = sensitivity
% KS = control effort
% T = complementary sensitivity
% ============================================================

s = tf('s');

% Sensitivity weight Ws
% High gain at low frequency -> force small tracking/balancing error
% Low gain at high frequency -> relax high-frequency tracking
Ms = 2.0;        % max sensitivity peak
As = 1e-3;       % steady-state error requirement
wbs = 5;         % desired balancing bandwidth rad/s

Ws_scalar = (s/Ms + wbs)/(s + wbs*As);

% Since output has 4 states, make Ws 4x4
% Stronger penalty on pitch psi than other states
Ws = blkdiag(0.1*Ws_scalar, ...
    5.0*Ws_scalar, ...
    0.1*Ws_scalar, ...
    1.0*Ws_scalar);

% Control weight Wu
% Penalizes motor voltage command
% The paper mentions voltage limits around +/-24 V in the controller comparison.
% Use 1/24 as a basic control penalty.
umax = 24;
Wu = (1/umax)*eye(2);

% Complementary sensitivity weight Wt
% Penalizes high-frequency response/noise amplification
Mt = 2.0;
At = 1e-2;
wbt = 30;

Wt_scalar = (s + wbt/Mt)/(At*s + wbt);

Wt = 0.05*blkdiag(Wt_scalar, ...
    Wt_scalar, ...
    Wt_scalar, ...
    Wt_scalar);

%% ============================================================
% 4. H-infinity mixed-sensitivity synthesis
% ============================================================

[K_hinf, CL_hinf, gamma_hinf] = mixsyn(G, Ws, Wu, Wt);

disp('H-infinity gamma:')
disp(gamma_hinf)

disp('H-infinity controller order:')
disp(order(K_hinf))

%% ============================================================
% 5. Closed-loop system for initial-condition simulation
%
% u = -K*y, y = x
% feedback(G,K,+1) gives negative feedback convention for u = -K*y
% If your response is unstable, switch +1 to -1.
% ============================================================

CL_hinf_ic = feedback(G,K_hinf,+1);

disp('H-infinity closed-loop poles:')
disp(pole(CL_hinf_ic))

%% ============================================================
% 6. LQR closed-loop system for comparison
% ============================================================

sys_lqr_ic = ss(Acl_lqr, [], eye(4), []);

%% ============================================================
% 7. Experiment 1 simulation: 10 seconds
% Initial pitch disturbance, no external disturbance
% ============================================================

t = 0:0.001:10;

%% Hinf closed-loop initial condition
x0_robot = [0; 0.3; 0.1; 0.3];

x0_controller = zeros(order(K_hinf),1);

x0_aug = [x0_robot;
    x0_controller];

% LQR response
[~,t_lqr,x_lqr] = initial(sys_lqr_ic,x0_robot,t);
u_lqr = -x_lqr*K_lqr';

% Hinf response
[y_hinf,t_hinf,x_aug_hinf] = initial(CL_hinf_ic,x0_aug,t);

% Hinf control input: controller driven by y = x
theta_hinf     = y_hinf(:,1);
psi_hinf       = y_hinf(:,2);
thetaDot_hinf  = y_hinf(:,3);
psiDot_hinf    = y_hinf(:,4);

u_hinf = lsim(K_hinf, y_hinf, t_hinf);

%% ============================================================
% 8. comparison: LQR vs Hinf
% ============================================================

figure;

subplot(3,2,1)
plot(t_lqr,x_lqr(:,1),'LineWidth',1.4); hold on;
plot(t_hinf,y_hinf(:,1),'LineWidth',1.4);
grid on;
title('(a) Average wheel angle')
xlabel('Time (s)')
ylabel('\theta (rad)')
legend('LQR','H_\infty')

subplot(3,2,2)
plot(t_lqr,x_lqr(:,3),'LineWidth',1.4); hold on;
plot(t_hinf,y_hinf(:,3),'LineWidth',1.4);
grid on;
title('(b) Average wheel angular velocity')
xlabel('Time (s)')
ylabel('\dot{\theta} (rad/s)')

subplot(3,2,3)
plot(t_lqr,x_lqr(:,2),'LineWidth',1.4); hold on;
plot(t_hinf,y_hinf(:,2),'LineWidth',1.4);
grid on;
title('(c) Body pitch angle')
xlabel('Time (s)')
ylabel('\psi (rad)')

subplot(3,2,4)
plot(t_lqr,x_lqr(:,4),'LineWidth',1.4); hold on;
plot(t_hinf,y_hinf(:,4),'LineWidth',1.4);
grid on;
title('(d) Body pitch angular velocity')
xlabel('Time (s)')
ylabel('\dot{\psi} (rad/s)')

subplot(3,2,5)
plot(t_lqr,u_lqr(:,1),'LineWidth',1.4); hold on;
plot(t_hinf,u_hinf(:,1),'LineWidth',1.4);
grid on;
title('(e) Left motor input')
xlabel('Time (s)')
ylabel('v_l (V)')
legend('LQR','H_\infty')

subplot(3,2,6)
plot(t_lqr,u_lqr(:,2),'LineWidth',1.4); hold on;
plot(t_hinf,u_hinf(:,2),'LineWidth',1.4);
grid on;
title('(f) Right motor input')
xlabel('Time (s)')
ylabel('v_r (V)')
legend('LQR','H_\infty')