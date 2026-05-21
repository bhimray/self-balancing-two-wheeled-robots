clear; clc; close all;

%% =====================================================
% NOMINAL SELF-BALANCING MODEL
% =====================================================

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

%% =====================================================
% LQR CONTROLLER
% =====================================================

Q = diag([10000 1 10000 1]);
R = diag([1 1]);

K_lqr = lqr(A,B,Q,R);

disp('LQR gain:')
disp(K_lqr)

%% =====================================================
% CLOSED-LOOP SYSTEM
% =====================================================

Acl = A - B*K_lqr;

sys_cl = ss(Acl,B,C,D);

%% =====================================================
% SIMULATION SETTINGS
% =====================================================

t = 0:0.001:10;

% Initial condition
% small pitch disturbance
x0 = [0;
    0.3;
    0.1;
    0.3];

%% =====================================================
% CLOSED-LOOP RESPONSE
% =====================================================

[y,t,x] = initial(sys_cl,x0,t);

%% =====================================================
% CONTROL INPUT COMPUTATION
% u = -Kx
% =====================================================

u = zeros(length(t),2);

for k = 1:length(t)
    u(k,:) = -K_lqr*x(k,:)';
end

%% =====================================================
% FIGURE 3 STYLE PLOTS
% =====================================================

figure;

% -----------------------------------------------------
% (a) Average wheel angle
% -----------------------------------------------------
subplot(3,2,1)
plot(t,x(:,1),'LineWidth',1.5)
grid on
title('(a) Average wheel angle')
xlabel('Time (s)')
ylabel('\theta (rad)')

% -----------------------------------------------------
% (b) Average wheel angular velocity
% -----------------------------------------------------
subplot(3,2,2)
plot(t,x(:,3),'LineWidth',1.5)
grid on
title('(b) Average wheel angular velocity')
xlabel('Time (s)')
ylabel('\theta dot (rad/s)')

% -----------------------------------------------------
% (c) Body pitch angle
% -----------------------------------------------------
subplot(3,2,3)
plot(t,x(:,2),'LineWidth',1.5)
grid on
title('(c) Body pitch angle')
xlabel('Time (s)')
ylabel('\psi (rad)')

% -----------------------------------------------------
% (d) Body pitch angular velocity
% -----------------------------------------------------
subplot(3,2,4)
plot(t,x(:,4),'LineWidth',1.5)
grid on
title('(d) Body pitch angular velocity')
xlabel('Time (s)')
ylabel('\psi dot (rad/s)')

% -----------------------------------------------------
% (e) Control inputs
% -----------------------------------------------------
subplot(3,2,[5 6])
plot(t,u(:,1),'LineWidth',1.5)
hold on
plot(t,u(:,2),'--','LineWidth',1.5)

grid on

title('(e) Control inputs')
xlabel('Time (s)')
ylabel('Voltage')

legend('v_l','v_r')