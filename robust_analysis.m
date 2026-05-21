clear; clc; close all;

%% =========================================================
%  1. NOMINAL PARAMETERS
%% ==========================================================
M0 = 0.5;      % wheel/base mass
m0 = 0.2;      % body mass
b0 = 0.1;      % friction
I0 = 0.006;    % body inertia
g  = 9.81;     % gravity
l0 = 0.3;      % COM height

q0 = (M0 + m0)*(I0 + m0*l0^2) - (m0*l0)^2;

A0 = [0 1 0 0;
      0 -(I0 + m0*l0^2)*b0/q0   (m0^2*g*l0^2)/q0   0;
      0 0 0 1;
      0 -(m0*l0*b0)/q0          m0*g*l0*(M0 + m0)/q0  0];

B0 = [0;
      (I0 + m0*l0^2)/q0;
      0;
      m0*l0/q0];

C0 = eye(4);
D0 = zeros(4,1);

sys_nom = ss(A0,B0,C0,D0); % nominal state-space model

%% =========================================================
%  2. OPEN-LOOP ANALYSIS
%% ==========================================================
disp('Open-loop poles:')
disp(eig(A0))

%% =========================================================
%  3. NOMINAL LQR CONTROLLER
%% ==========================================================
Q = diag([10 1 100 1]);
R = 0.1;

K_lqr = lqr(A0,B0,Q,R);

Acl_nom = A0 - B0*K_lqr;
sys_cl_nom = ss(Acl_nom,B0,C0,D0);

disp('Closed-loop nominal poles (LQR):')
disp(eig(Acl_nom))

figure;
step(sys_cl_nom,5);
title('Nominal Closed-Loop Step Response (LQR)');
grid on;

%% =========================================================
%  4. UNCERTAIN PARAMETERS
% ==========================================================
m = ureal('m',m0,'Percentage',15);
I = ureal('I',I0,'Percentage',15);
l = ureal('l',l0,'Percentage',10);
b = ureal('b',b0,'Percentage',25);

% M is kept fixed here. You can also make it uncertain if you want:
% M = ureal('M',M0,'Percentage',10);
M = M0;

%% =========================================================
%  5. UNCERTAIN STATE-SPACE MODEL
% ==========================================================
q = (M + m)*(I + m*l^2) - (m*l)^2;

A = [0 1 0 0;
     0 -(I + m*l^2)*b/q   (m^2*g*l^2)/q   0;
     0 0 0 1;
     0 -(m*l*b)/q         m*g*l*(M + m)/q  0];

B = [0;
     (I + m*l^2)/q;
     0;
     m*l/q];

C = eye(4);
D = zeros(4,1);

sys_unc = ss(A,B,C,D);

%% =========================================================
%  6. UNCERTAIN CLOSED-LOOP SYSTEM WITH NOMINAL LQR GAIN
% ==========================================================
Acl_unc = A - B*K_lqr;
sys_cl_unc = ss(Acl_unc,B,C,D);

%% =========================================================
%  7. ROBUST STABILITY ANALYSIS
% ==========================================================
% robstab returns a robustness margin:
% margin > 1  -> robustly stable
% margin < 1  -> not robustly stable

[stabmarg, destabunc, report] = robstab(sys_cl_unc);

disp('================ ROBUST STABILITY RESULT ================')
disp(stabmarg)
disp(report)

%% =========================================================
%  8. INTERPRET THE MARGIN
% ==========================================================
% For most workflows:
% LowerBound > 1  => definitely robustly stable
% UpperBound < 1  => definitely not robustly stable
% If interval straddles 1, result is inconclusive/tighten analysis

fprintf('\nRobust stability margin lower bound = %.4f\n', stabmarg.LowerBound);
fprintf('Robust stability margin upper bound = %.4f\n', stabmarg.UpperBound);

if stabmarg.LowerBound > 1
    fprintf('Conclusion: system is robustly stable for all modeled uncertainties.\n');
elseif stabmarg.UpperBound < 1
    fprintf('Conclusion: system is NOT robustly stable for the modeled uncertainties.\n');
else
    fprintf('Conclusion: result is inconclusive; margin interval crosses 1.\n');
end

%% =========================================================
%  9. WORST-CASE DESTABILIZING UNCERTAINTY
% ==========================================================
% Substitute the worst-case perturbation found by robstab
sys_worst = usubs(sys_cl_unc, destabunc);

disp('Worst-case closed-loop poles:')
disp(pole(sys_worst))

figure;
step(sys_worst,5);
title('Worst-Case Closed-Loop Step Response');
grid on;

%% =========================================================
%  10. COMPARE NOMINAL VS WORST-CASE
% ==========================================================
figure;
step(sys_cl_nom,'b',sys_worst,'r--',5);
legend('Nominal closed-loop','Worst-case closed-loop');
title('Nominal vs Worst-Case Closed-Loop Response');
grid on;