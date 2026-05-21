%% Robust analysis of TWSBR with M,H parametric uncertainty
% Requires: Robust Control Toolbox

clear; clc; close all;

%% -----------------------------
% 1. Nominal physical parameters
% ------------------------------
g  = 9.81;

m  = 0.051;       % wheel mass [kg]
R  = 0.0325;      % wheel radius [m]
W  = 0.192;       % body width [m]
D  = 0.082;       % body depth [m]

Jw = m*R^2/2;     % wheel inertia
Jm = 1e-5;        % motor inertia

n  = 30;          % gear ratio
Rm = 2.9;         % motor resistance
Kb = 0.024;       % back EMF constant
Kt = 0.025;       % torque constant

fm = 0.0022;      % motor/body friction
fw = 0;           % wheel/floor friction

%% -----------------------------
% 2. Uncertain parameters
% ------------------------------
% Paper nominal values:
M0 = 0.703;       % body mass [kg]
H0 = 0.112;       % body height [m]

% assumed uncertainty levels
M = ureal('M', M0, 'Percentage', 20);
H = ureal('H', H0, 'Percentage', 40);

%% -----------------------------
% 3. Build uncertain A(M,H), B(M,H)
% ------------------------------
[A1_unc, B1_unc, A2_unc, B2_unc] = build_TWSBR_uncertain(M,H,m,R,W,D,Jw,Jm,n,Rm,Kb,Kt,fm,fw,g);

% Full 6-state model:
% x = [theta; psi; theta_dot; psi_dot; phi; phi_dot]
A_unc = blkdiag(A1_unc, A2_unc);
B_unc = [B1_unc;
         B2_unc];

C_unc = eye(6);
D_unc = zeros(6,2);

P_unc = ss(A_unc, B_unc, C_unc, D_unc);

disp('Nominal uncertain plant:')
P_nom = P_unc.NominalValue

%% -----------------------------
% 4. Check open-loop eigenvalues
% ------------------------------
disp('Open-loop nominal eigenvalues:')
eig(P_nom.A)

%% -----------------------------
% 5. Design nominal LQR controller
% ------------------------------
% Full-state feedback: u = -Kx
Q = diag([10, 500, 1, 10, 20, 1]);
R_lqr = 0.1*eye(2);

K_lqr = lqr(P_nom.A, P_nom.B, Q, R_lqr);

disp('LQR gain K:')
disp(K_lqr)

%% -----------------------------
% 6. Closed-loop uncertain system
% ------------------------------
% xdot = (A - B*K)x + B*d
% d is input disturbance entering motor voltage channel
Acl_unc = A_unc - B_unc*K_lqr;

Bdist_unc = B_unc;        % disturbance entering same channel as voltage
Cperf_unc = eye(6);       % performance output = all states
Dperf_unc = zeros(6,2);

CL_unc = ss(Acl_unc, Bdist_unc, Cperf_unc, Dperf_unc);

disp('Closed-loop nominal eigenvalues:')
eig(CL_unc.NominalValue.A)

%% -----------------------------
% 7. Robust stability analysis
% ------------------------------
disp('Running robstab...')
[stabmarg, destabunc, report] = robstab(CL_unc);

disp(stabmarg)
disp(report)

%% -----------------------------
% 8. Worst-case gain / robust performance
% ------------------------------
% This checks worst-case amplification from voltage disturbance d
% to state response x.
disp('Running wcgain...')
[wcg, wcunc, wcinfo] = wcgain(CL_unc);

disp('Worst-case gain:')
disp(wcg)

%% -----------------------------
% 9. Sample uncertain closed-loop responses
% ------------------------------
figure;
usample_CL = usample(CL_unc, 30);
step(usample_CL, 3);
grid on;
title('Sampled uncertain closed-loop response from voltage disturbance to states');

%% -----------------------------
% 10. Compare nominal and worst-case model
% ------------------------------
CL_nom = CL_unc.NominalValue;
CL_wc  = usubs(CL_unc, wcunc);

figure;
bodemag(CL_nom, CL_wc);
grid on;
legend('Nominal','Worst-case');
title('Nominal vs worst-case closed-loop magnitude');

%% ============================================================
% Local function: parameter-dependent TWSBR model
% ============================================================
function [A1,B1,A2,B2] = build_TWSBR_uncertain(M,H,m,R,W,D,Jw,Jm,n,Rm,Kb,Kt,fm,fw,g)

    L = H/2;

    Jpsi = M*L^2/3;
    Jphi = M*(W^2 + D^2)/12;

    alpha = n*Kt/Rm;
    beta  = n*Kt*Kb/Rm + fm;

    %% Self-balancing subsystem
    E11 = (2*m + M)*R^2 + 2*Jw + 2*n^2*Jm;
    E12 = M*L*R - 2*n^2*Jm;
    E21 = -M*L*R + 2*n^2*Jm;
    E22 = M*L^2 + Jpsi + 2*n^2*Jm;

    detE = E11*E22 - E12*E21;

    A32 = -MgL_term(M,g,L)*E12/detE;
    A42 =  MgL_term(M,g,L)*E11/detE;

    A33 = -(2*(beta + fw)*E22 + 2*beta*E12)/detE;
    A43 =  (2*(beta + fw)*E21 + 2*beta*E11)/detE;

    A34 =  2*beta*(E22 + E12)/detE;
    A44 = -2*beta*(E11 + E21)/detE;

    B3 =  alpha*(E22 + E12)/detE;
    B4 = -alpha*(E11 + E21)/detE;

    A1 = [0 0 1 0;
          0 0 0 1;
          0 A32 A33 A34;
          0 A42 A43 A44];

    B1 = [0 0;
          0 0;
          B3 B3;
          B4 B4];

    %% Steering subsystem
    I = 0.5*m*W^2 + Jphi + (W^2/(2*R^2))*(Jw + n^2*Jm);
    J = (W^2/(2*R^2))*(beta + fw);
    K = (W/R)*alpha;

    A2 = [0 1;
          0 -J/I];

    B2 = [0 0;
          K/I -K/I];
end

function val = MgL_term(M,g,L)
    val = M*g*L;
end