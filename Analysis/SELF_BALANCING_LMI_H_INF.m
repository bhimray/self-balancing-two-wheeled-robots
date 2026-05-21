clear; clc; close all;


sdpt3Path = 'C:\Users\rayyu\OneDrive\Desktop\UTD\sdpt3';
if isfolder(sdpt3Path)
    addpath(genpath(sdpt3Path));
end

if exist('yalmip', 'file') == 2
    yalmip('clear');
end

if exist('sdpvar', 'file') ~= 2 || exist('optimize', 'file') ~= 2
    error(['YALMIP is not on the MATLAB path. Add YALMIP first, then run ' ...
        'this script again. Example: addpath(genpath(''C:\path\to\YALMIP''))']);
end

%% ============================================================
% EXPERIMENT 3: PARAMETRIC UNCERTAINTY USING LMI-H_INFINITY ONLY
%
% Self-balancing TWSBR subsystem:
% x = [theta; psi; theta_dot; psi_dot]
%
% Uncertainty:
% M in [0.2, 0.3] kg
% H in [0.14, 0.20] m
%
% Controller:
% u_b = -K_lmi_hinf x
% v_l = v_r = u_b/2
% ============================================================

%% ============================================================
% 1. Nominal midpoint model
% ============================================================

M0 = 0.25;
H0 = 0.112;

[A1_nom,B1_nom,~,~,par_nom] = twsbr_symbolic_ss(M0,H0);

A = A1_nom;
B = B1_nom(:,1);

C = eye(4);
D = zeros(4,1);

G = ss(A,B,C,D);

nx = size(A,1);
nu = size(B,2);

disp('================ NOMINAL OPEN LOOP ================')
disp('Open-loop poles:')
disp(eig(A))

disp('Controllability rank:')
disp(rank(ctrb(A,B)))

disp('Observability rank:')
disp(rank(obsv(A,C)))

%% ============================================================
% 2. Nominal LMI-H_INFINITY synthesis
%
% Plant:
% xdot = A x + B1w w + B u
%
% Controller:
% u = -Kx
%
% Performance:
% z = [Qz^(1/2)x;
%      Rz^(1/2)u]
% ============================================================

B1w = [0;
       0;
       1;
       1];

nw = size(B1w,2);

Qz = diag([0.1 1.0 0.01 0.1]);  % pitch weighted most
Rz = 1e3;                        % control penalty

Cz = [sqrt(Qz);
      zeros(nu,nx)];

Dz = [zeros(nx,nu);
      sqrt(Rz)*eye(nu)];

nz = size(Cz,1);
D11 = zeros(nz,nw);

X = sdpvar(nx,nx,'symmetric');
Y = sdpvar(nu,nx,'full');
gamma = sdpvar(1,1);

AclX = A*X - B*Y;
CclX = Cz*X - Dz*Y;

LMI = [AclX + AclX',  B1w,              CclX';
       B1w',          -gamma*eye(nw),   D11';
       CclX,           D11,            -gamma*eye(nz)];

umax = 48;   % bound on u_b, or use 48 if u_b = vl + vr
%% Actuator constraint LMI:
ActuatorLMI = [umax^2, Y;
            Y',  X] >= 1e-6*eye(nx+1);

Constraints = [];
Constraints = [Constraints, X >= 1e-6*eye(nx)];
Constraints = [Constraints, gamma >= 1e-6];
Constraints = [Constraints, LMI <= -1e-6*eye(size(LMI))];
Constraints = [Constraints, ActuatorLMI]; %% Add actuator constraint LMI

Objective = gamma;

opts = sdpsettings('solver','sdpt3','verbose',1);
sol = optimize(Constraints,Objective,opts);

if sol.problem ~= 0
    error('LMI-Hinf synthesis failed: %s', sol.info);
end

Xv = value(X);
Yv = value(Y);

gamma_lmi_hinf = value(gamma);
K_lmi_hinf = Yv / Xv;

Acl_lmi_hinf = A - B*K_lmi_hinf;

disp('================ NOMINAL LMI-H_INFINITY ================')
disp('K_lmi_hinf:')
disp(K_lmi_hinf)

disp('LMI-Hinf gamma:')
disp(gamma_lmi_hinf)

disp('Closed-loop poles:')
disp(eig(Acl_lmi_hinf))

%% ============================================================
% 3. Uncertain parameter model
% ============================================================

M_unc = ureal('M',M0,'Range',[0.2 0.3]);
H_unc = ureal('H',H0,'Range',[0.14 0.20]);

[A1_unc,B1_unc,~,~,~] = twsbr_symbolic_ss(M_unc,H_unc);

A_unc = A1_unc;
B_unc = B1_unc(:,1);

G_unc = ss(A_unc,B_unc,eye(4),zeros(4,1));

%% ============================================================
% 4. Robust stability analysis
% ============================================================

Acl_unc_lmi = A_unc - B_unc*K_lmi_hinf;

CL_lmi_unc = ss(Acl_unc_lmi,zeros(4,1),eye(4),zeros(4,1));

disp('================ ROBSTAB: LMI-H_INFINITY ================')

[stab_lmi,destab_lmi,report_lmi] = robstab(CL_lmi_unc);

disp(stab_lmi)
disp(report_lmi)

fprintf('\nLMI-Hinf robust stability lower bound = %.4f\n',stab_lmi.LowerBound);
fprintf('LMI-Hinf robust stability upper bound = %.4f\n',stab_lmi.UpperBound);

if stab_lmi.LowerBound > 1
    fprintf('Conclusion: LMI-Hinf is robustly stable for full modeled uncertainty.\n');
elseif stab_lmi.UpperBound < 1
    fprintf('Conclusion: LMI-Hinf is NOT robustly stable for full modeled uncertainty.\n');
else
    fprintf('Conclusion: inconclusive robust stability result.\n');
end

%% ============================================================
% 5. Worst-case pole check
% ============================================================

disp('================ WORST-CASE POLE CHECK ================')

try
    CL_lmi_worst = usubs(CL_lmi_unc,destab_lmi);
    disp('Worst-case LMI-Hinf poles:')
    disp(pole(CL_lmi_worst))
catch
    disp('No destabilizing substitution returned or substitution failed.')
end

%% ============================================================
% 6. Endpoint simulations
% ============================================================

t = 0:0.001:10;

x0 = [0;
      0.1;
      0.3;
      0.3];

cases = {
    'Nominal',       0.25, 0.17;
    'Low mass/height', 0.2, 0.14;
    'Mass change',   0.3, 0.17;
    'Height change', 0.25, 0.20;
    'Both max',      0.3, 0.20
};

results = struct();

for i = 1:size(cases,1)

    caseName = cases{i,1};
    Mval = cases{i,2};
    Hval = cases{i,3};

    [A1_i,B1_i,~,~,~] = twsbr_symbolic_ss(Mval,Hval);

    Ai = A1_i;
    Bi = B1_i(:,1);

    Acl_i = Ai - Bi*K_lmi_hinf;
    sys_i = ss(Acl_i,[],eye(4),[]);

    [~,ti,xi] = initial(sys_i,x0,t);

    u_i = -xi*K_lmi_hinf';
    vl_i = u_i/2;
    vr_i = u_i/2;

    field = matlab.lang.makeValidName(caseName);

    results.(field).t = ti;
    results.(field).x = xi;
    results.(field).u = u_i;
    results.(field).vl = vl_i;
    results.(field).vr = vr_i;

    fprintf('\n================ CASE: %s ================\n',caseName)
    fprintf('M = %.3f kg, H = %.3f m\n',Mval,Hval)
    fprintf('max |theta| = %.4f rad\n',max(abs(xi(:,1))));
    fprintf('max |psi|   = %.4f rad\n',max(abs(xi(:,2))));
    fprintf('max |u_b|   = %.4f V\n',max(abs(u_i)));
    fprintf('max |v_l|   = %.4f V\n',max(abs(vl_i)));

end

%% ============================================================
% 7. Endpoint plots
% ============================================================

figure('Name','Experiment 3 LMI-Hinf: Pitch Angle','Color','w');

for i = 1:size(cases,1)

    caseName = cases{i,1};
    field = matlab.lang.makeValidName(caseName);

    plot(results.(field).t,results.(field).x(:,2),'LineWidth',1.4); hold on;

end

grid on;
xlabel('Time (s)')
ylabel('\psi (rad)')
title('LMI-H_\infty Pitch Response Under Parameter Changes')
legend(cases(:,1),'Location','best')

figure('Name','Experiment 3 LMI-Hinf: Wheel Angle','Color','w');

for i = 1:size(cases,1)
    
    caseName = cases{i,1};
    field = matlab.lang.makeValidName(caseName);
    
    plot(results.(field).t,results.(field).x(:,1),'LineWidth',1.4); hold on;
    
end

grid on;
xlabel('Time (s)')
ylabel('\theta (rad)')
title('LMI-H_\infty Wheel Angle Response Under Parameter Changes')
legend(cases(:,1),'Location','best')

figure('Name','Experiment 3 LMI-Hinf: Motor Voltage','Color','w');

for i = 1:size(cases,1)

    caseName = cases{i,1};
    field = matlab.lang.makeValidName(caseName);

    plot(results.(field).t,results.(field).vl,'LineWidth',1.4); hold on;

end

yline(24,'r--','+24 V');
yline(-24,'r--','-24 V');
grid on;
xlabel('Time (s)')
ylabel('v_l (V)')
title('LMI-H_\infty Motor Voltage Under Parameter Changes')
legend(cases(:,1),'Location','best')

%% ============================================================
% 8. Worst-case gain analysis
% ============================================================

disp('================ WCGAIN: LMI-H_INFINITY ================')

CL_lmi_perf_unc = ss(Acl_unc_lmi,B1w,eye(4),zeros(4,1));

[wcg_lmi,wcu_lmi,info_lmi] = wcgain(CL_lmi_perf_unc);

disp('LMI-Hinf worst-case gain:')
disp(wcg_lmi)

fprintf('\nWorst-case gain lower bound = %.4f\n',wcg_lmi.LowerBound);
fprintf('Worst-case gain upper bound = %.4f\n',wcg_lmi.UpperBound);

%% ============================================================
% 9. Monte Carlo simulation
% ============================================================

Nmc = 100;
rng(1);

M_samples = 0.2 + (0.3-0.2)*rand(Nmc,1);
H_samples = 0.14 + (0.20-0.14)*rand(Nmc,1);

maxPsi = zeros(Nmc,1);
maxTheta = zeros(Nmc,1);
maxVl = zeros(Nmc,1);
stable = true(Nmc,1);

figure('Name','Monte Carlo LMI-Hinf Pitch','Color','w');
hold on; grid on;
xlabel('Time (s)')
ylabel('\psi (rad)')
title('Monte Carlo Pitch Response: LMI-H_\infty')

for k = 1:Nmc

    Mval = M_samples(k);
    Hval = H_samples(k);

    [A1_k,B1_k,~,~,~] = twsbr_symbolic_ss(Mval,Hval);

    Ak = A1_k;
    Bk = B1_k(:,1);

    Acl_k = Ak - Bk*K_lmi_hinf;

    stable(k) = all(real(eig(Acl_k)) < 0);

    sys_k = ss(Acl_k,[],eye(4),[]);

    try
        [~,tk,xk] = initial(sys_k,x0,t);

        uk = -xk*K_lmi_hinf';
        vlk = uk/2;

        maxTheta(k) = max(abs(xk(:,1)));
        maxPsi(k)   = max(abs(xk(:,2)));
        maxVl(k)    = max(abs(vlk));

        plot(tk,xk(:,2),'Color',[0.75 0.75 1]);

    catch
        stable(k) = false;
        maxTheta(k) = Inf;
        maxPsi(k) = Inf;
        maxVl(k) = Inf;
    end

end

fprintf('\n================ MONTE CARLO SUMMARY ================\n')
fprintf('Stable samples      = %d / %d\n',sum(stable),Nmc)
fprintf('Max MC |psi|        = %.4f rad\n',max(maxPsi(isfinite(maxPsi))))
fprintf('Mean MC max |psi|   = %.4f rad\n',mean(maxPsi(isfinite(maxPsi))))
fprintf('Max MC |theta|      = %.4f rad\n',max(maxTheta(isfinite(maxTheta))))
fprintf('Max MC |v_l|        = %.4f V\n',max(maxVl(isfinite(maxVl))))

%% ============================================================
% 10. Monte Carlo histograms
% ============================================================

figure('Name','Monte Carlo Metrics: LMI-Hinf','Color','w');

subplot(1,3,1)
histogram(maxPsi(isfinite(maxPsi)),20)
grid on
xlabel('max |\psi| (rad)')
ylabel('Count')
title('Pitch Peak')

subplot(1,3,2)
histogram(maxTheta(isfinite(maxTheta)),20)
grid on
xlabel('max |\theta| (rad)')
ylabel('Count')
title('Wheel Angle Peak')

subplot(1,3,3)
histogram(maxVl(isfinite(maxVl)),20)
grid on
xlabel('max |v_l| (V)')
ylabel('Count')
title('Voltage Peak')

%% ============================================================
% LOCAL FUNCTION
% ============================================================

function [A1,B1,A2,B2,par] = twsbr_symbolic_ss(M,H)

g  = 9.81;

m  = 0.051;
R  = 0.0325;
W  = 0.192;
D  = 0.082;

Jm = 1e-5;
Rm = 2.9;
Kb = 0.024;
Kt = 0.025;
n  = 30;

fm = 0.0022;
fw = 0;

L = H/2;
Jw = m*R^2/2;
Jpsi = M*L^2/3;
Jphi = M*(W^2 + D^2)/12;

alpha = n*Kt/Rm;
beta  = n*Kt*Kb/Rm + fm;

E11 = (2*m + M)*R^2 + 2*Jw + 2*n^2*Jm;
E12 = M*L*R - 2*n^2*Jm;
E22 = M*L^2 + Jpsi + 2*n^2*Jm;

detE = E11*E22 - E12^2;

A32 = -g*M*L*E12/detE;
A42 =  g*M*L*E11/detE;

A33 = -((2*beta + fw)*E22 + 2*beta*E12)/detE;
A43 =  ((2*beta + fw)*E12 + 2*beta*E11)/detE;

A34 =  2*beta*(E22 + E12)/detE;
A44 = -2*beta*(E11 + E12)/detE;

A1 = [0 0 1 0;
      0 0 0 1;
      0 A32 A33 A34;
      0 A42 A43 A44];

B3 =  alpha*(E22 + E12)/detE;
B4 = -alpha*(E11 + E12)/detE;

B1 = [0  0;
      0  0;
      B3 B3;
      B4 B4];

I = m*W^2/2 + Jphi + (W^2/(2*R^2))*(Jw + n^2*Jm);
J = fw*W^2/(2*R^2) + beta*W^2/(2*R^2);
K = alpha*W/(2*R);

A2 = [0 1;
      0 -J/I];

B2 = [0    0;
     -K/I  K/I];

par.g = g;
par.m = m;
par.R = R;
par.W = W;
par.D = D;
par.H = H;
par.M = M;
par.L = L;
par.Jw = Jw;
par.Jpsi = Jpsi;
par.Jphi = Jphi;
par.Jm = Jm;
par.Rm = Rm;
par.Kb = Kb;
par.Kt = Kt;
par.n = n;
par.fm = fm;
par.fw = fw;
par.alpha = alpha;
par.beta = beta;
par.E11 = E11;
par.E12 = E12;
par.E22 = E22;
par.detE = detE;
par.I = I;
par.J = J;
par.K = K;

end