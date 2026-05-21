clear; clc; close all;

%% ============================================================
% EXPERIMENT 3: PARAMETRIC UNCERTAINTY / ROBUST STABILITY
% Self-balancing robot: LQR vs H-infinity
%
% Paper Experiment 3:
% 1) Payload mass: M = 0.2 kg -> 0.3 kg
% 2) Robot height: H = 0.14 m -> 0.20 m
%
% Controllers are designed at nominal plant and kept fixed.
% ============================================================

%% ============================================================
% 1. Nominal self-balancing subsystem
% ============================================================

M0 = 0.25;
H0 = 0.17;

[A1_nom,B1_nom,A2_nom,B2_nom,par_nom] = twsbr_symbolic_ss(M0,H0);

A = A1_nom;
B = B1_nom(:,1);       % one-input common-mode balancing input

C = eye(4);
D = zeros(4,1);

G = ss(A,B,C,D);

disp('================ NOMINAL OPEN LOOP ================')
disp('Open-loop poles:')
disp(eig(A))

disp('Controllability rank:')
disp(rank(ctrb(A,B)))

disp('Observability rank:')
disp(rank(obsv(A,C)))

%% ============================================================
% 2. Nominal LQR controller
% ============================================================

Q_lqr = diag([10000 1 10000 1]);
R_lqr = 1000;

K_lqr = lqr(A,B,Q_lqr,R_lqr);
Acl_lqr = A - B*K_lqr;

disp('================ NOMINAL LQR ================')
disp('K_lqr:')
disp(K_lqr)

disp('LQR nominal closed-loop poles:')
disp(eig(Acl_lqr))

%% ============================================================
% 3. Nominal H-infinity mixed-sensitivity controller
% ============================================================

s = tf('s');

Ms  = 3;
As  = 0.15;
wbs = 4;

Ws_base = (s/Ms + wbs)/(s + wbs*As);

Ws = blkdiag(0.01*Ws_base, ...
    0.1*Ws_base, ...
    0.005*Ws_base, ...
    0.08*Ws_base);

umax = 24;
Wu = tf(1/(umax*2));

Mt  = 3;
At  = 5e-2;
wbt = 50;

Wt_scalar = (s + wbt/Mt)/(At*s + wbt);

Wt = 0.01*blkdiag(Wt_scalar, ...
    Wt_scalar, ...
    Wt_scalar, ...
    Wt_scalar);

[K_hinf, CL_hinf_weighted, gamma_hinf] = mixsyn(G,Ws,Wu,Wt);

disp('================ NOMINAL H-INFINITY ================')
disp('Hinf gamma:')
disp(gamma_hinf)

disp('Hinf controller order:')
disp(order(K_hinf))

CL_hinf_nom = feedback(G,K_hinf);

disp('Hinf nominal physical closed-loop poles:')
disp(pole(CL_hinf_nom))

%% ============================================================
% 4. Structured uncertainty model using physical M,H
% ============================================================

M_unc = ureal('M',M0,'Range',[0.2 0.3]);
H_unc = ureal('H',H0,'Range',[0.14 0.20]);

[A1_unc,B1_unc,A2_unc,B2_unc,par_unc] = twsbr_symbolic_ss(M_unc,H_unc);

A_unc = A1_unc;
B_unc = B1_unc(:,1);

G_unc = ss(A_unc,B_unc,eye(4),zeros(4,1));

L_hinf = G_unc*K_hinf;
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
% 5. Robust stability analysis: LQR
% ============================================================

Acl_lqr_unc = A_unc - B_unc*K_lqr;
CL_lqr_unc = ss(Acl_lqr_unc,zeros(4,1),eye(4),zeros(4,1));

disp('================ ROBSTAB: LQR ================')

[stab_lqr,destab_lqr,report_lqr] = robstab(CL_lqr_unc);

disp(stab_lqr)
disp("Destabilizing substitution for LQR:")
disp(destab_lqr)
disp(report_lqr)

fprintf('\nLQR robust stability lower bound = %.4f\n',stab_lqr.LowerBound);
fprintf('LQR robust stability upper bound = %.4f\n',stab_lqr.UpperBound);

if stab_lqr.LowerBound > 1
    fprintf('LQR conclusion: robustly stable for full modeled uncertainty.\n');
elseif stab_lqr.UpperBound < 1
    fprintf('LQR conclusion: NOT robustly stable for full modeled uncertainty.\n');
else
    fprintf('LQR conclusion: inconclusive margin interval crosses 1.\n');
end

%% ============================================================
% 6. Robust stability analysis: H-infinity
% ============================================================

CL_hinf_unc = feedback(G_unc,K_hinf);

disp('================ ROBSTAB: H-INFINITY ================')

[stab_hinf,destab_hinf,report_hinf] = robstab(CL_hinf_unc);

disp(stab_hinf)

disp("Destabilizing substitution for Hinf:")
disp(destab_hinf)

disp(report_hinf)

fprintf('\nHinf robust stability lower bound = %.4f\n',stab_hinf.LowerBound);
fprintf('Hinf robust stability upper bound = %.4f\n',stab_hinf.UpperBound);

if stab_hinf.LowerBound > 1
    fprintf('Hinf conclusion: robustly stable for full modeled uncertainty.\n');
elseif stab_hinf.UpperBound < 1
    fprintf('Hinf conclusion: NOT robustly stable for full modeled uncertainty.\n');
else
    fprintf('Hinf conclusion: inconclusive margin interval crosses 1.\n');
end

%% ============================================================
% 7. Worst-case pole analysis
% ============================================================

disp('================ WORST-CASE POLE CHECK ================')

try
    CL_lqr_worst = usubs(CL_lqr_unc,destab_lqr);
    disp('LQR worst-case poles:')
    disp(pole(CL_lqr_worst))
catch
    disp('No LQR destabilizing substitution returned or substitution failed.')
end

try
    CL_hinf_worst = usubs(CL_hinf_unc,destab_hinf);
    disp('Hinf worst-case poles:')
    disp(pole(CL_hinf_worst))
catch
    disp('No Hinf destabilizing substitution returned or substitution failed.')
end

%% ============================================================
% 8. Endpoint simulations
% ============================================================

t = 0:0.001:10;

x0_robot = [0;
    0.1;
    0.3;
    0.3];

cases = {
    'Nominal',       0.25,  0.17;
    'Mass change',   0.3,  0.17;
    'Height change', 0.25,  0.20;
    'Both changes',  0.3,  0.20
    };

results = struct();

for i = 1:size(cases,1)
    
    caseName = cases{i,1};
    Mval = cases{i,2};
    Hval = cases{i,3};
    
    [A1_i,B1_i,~,~,~] = twsbr_symbolic_ss(Mval,Hval);
    
    Ai = A1_i;
    Bi = B1_i(:,1);
    Gi = ss(Ai,Bi,C,D);
    
    %% ----- LQR endpoint response -----
    Acl_i_lqr = Ai - Bi*K_lqr;
    sys_i_lqr = ss(Acl_i_lqr,[],eye(4),[]);
    
    [~,t_i_lqr,x_i_lqr] = initial(sys_i_lqr,x0_robot,t);
    
    u_i_lqr = -x_i_lqr*K_lqr';
    vl_i_lqr = u_i_lqr/2;
    vr_i_lqr = u_i_lqr/2;
    
    %% ----- Hinf endpoint response -----
    CL_i_hinf = feedback(Gi,K_hinf);
    
    x0_aug = [x0_robot;
        zeros(order(K_hinf),1)];
    
    [y_i_hinf,t_i_hinf,~] = initial(CL_i_hinf,x0_aug,t);
    
    u_i_hinf_raw = lsim(K_hinf,y_i_hinf,t_i_hinf);
    u_i_hinf = -u_i_hinf_raw;
    
    vl_i_hinf = u_i_hinf/2;
    vr_i_hinf = u_i_hinf/2;
    
    %% Save
    field = matlab.lang.makeValidName(caseName);
    
    results.(field).t_lqr = t_i_lqr;
    results.(field).x_lqr = x_i_lqr;
    results.(field).u_lqr = u_i_lqr;
    results.(field).vl_lqr = vl_i_lqr;
    results.(field).vr_lqr = vr_i_lqr;
    
    results.(field).t_hinf = t_i_hinf;
    results.(field).x_hinf = y_i_hinf;
    results.(field).u_hinf = u_i_hinf;
    results.(field).vl_hinf = vl_i_hinf;
    results.(field).vr_hinf = vr_i_hinf;
    
    %% Print metrics
    fprintf('\n================ CASE: %s ================\n',caseName)
    fprintf('M = %.3f kg, H = %.3f m\n',Mval,Hval)
    
    fprintf('LQR max |psi|      = %.4f rad\n',max(abs(x_i_lqr(:,2))));
    fprintf('LQR max |theta|    = %.4f rad\n',max(abs(x_i_lqr(:,1))));
    fprintf('LQR max |v_l|      = %.4f V\n',max(abs(vl_i_lqr)));
    
    fprintf('Hinf max |psi|     = %.4f rad\n',max(abs(y_i_hinf(:,2))));
    fprintf('Hinf max |theta|   = %.4f rad\n',max(abs(y_i_hinf(:,1))));
    fprintf('Hinf max |v_l|     = %.4f V\n',max(abs(vl_i_hinf)));
    
end

%% ============================================================
% 9. Plot Experiment 3 endpoint comparison
% ============================================================

figure('Name','Experiment 3: Pitch Angle Under Parameter Changes','Color','w');

for i = 1:size(cases,1)
    
    caseName = cases{i,1};
    field = matlab.lang.makeValidName(caseName);
    
    subplot(2,2,i)
    plot(results.(field).t_lqr,results.(field).x_lqr(:,2),'LineWidth',1.4); hold on;
    plot(results.(field).t_hinf,results.(field).x_hinf(:,2),'LineWidth',1.4);
    grid on;
    title(caseName)
    xlabel('Time (s)')
    ylabel('\psi (rad)')
    legend('LQR','H_\infty','Location','best')
    
end

figure('Name','Experiment 3: Wheel Angle Under Parameter Changes','Color','w');

for i = 1:size(cases,1)
    
    caseName = cases{i,1};
    field = matlab.lang.makeValidName(caseName);
    
    subplot(2,2,i)
    plot(results.(field).t_lqr,results.(field).x_lqr(:,1),'LineWidth',1.4); hold on;
    plot(results.(field).t_hinf,results.(field).x_hinf(:,1),'LineWidth',1.4);
    grid on;
    title(caseName)
    xlabel('Time (s)')
    ylabel('\theta (rad)')
    legend('LQR','H_\infty','Location','best')
    
end

figure('Name','Experiment 3: Motor Voltage Under Parameter Changes','Color','w');

for i = 1:size(cases,1)
    
    caseName = cases{i,1};
    field = matlab.lang.makeValidName(caseName);
    
    subplot(2,2,i)
    plot(results.(field).t_lqr,results.(field).vl_lqr,'LineWidth',1.4); hold on;
    plot(results.(field).t_hinf,results.(field).vl_hinf,'LineWidth',1.4);
    yline(24,'r--','+24 V');
    yline(-24,'r--','-24 V');
    grid on;
    title(caseName)
    xlabel('Time (s)')
    ylabel('v_l (V)')
    legend('LQR','H_\infty','Location','best')
    
end

%% ============================================================
% 10. Robustness margin comparison bar plot
% ============================================================

figure('Name','Robust Stability Margin Comparison','Color','w');

bar([stab_lqr.LowerBound, stab_hinf.LowerBound])
grid on;
set(gca,'XTickLabel',{'LQR','H_\infty'})
ylabel('Robust Stability Margin Lower Bound')
title('Robust Stability Margin Comparison')


%% ============================================================
% 11. WORST-CASE PERFORMANCE / GAIN ANALYSIS
% ============================================================

disp('================ WCGAIN: LQR ================')

% Performance output = physical states
% Input = zero dummy input is not useful, so define disturbance channel Bd
Bd = [0; 0; 1; 1];

% LQR uncertain disturbance-to-state system
CL_lqr_perf_unc = ss(Acl_lqr_unc, Bd, eye(4), zeros(4,1));

[wcg_lqr,wcu_lqr,info_lqr] = wcgain(CL_lqr_perf_unc);

disp('LQR worst-case gain:')
disp(wcg_lqr)

disp('LQR worst-case uncertainty:')
disp(wcu_lqr)

disp('================ WCGAIN: H-INFINITY ================')

% For Hinf, augment disturbance into physical plant states only
CL_hinf_unc_full = feedback(G_unc,K_hinf);

ncl = order(CL_hinf_unc_full);
Bd_aug = [Bd;
    zeros(ncl-4,1)];

CL_hinf_perf_unc = ss(CL_hinf_unc_full.A, ...
    Bd_aug, ...
    CL_hinf_unc_full.C, ...
    zeros(size(CL_hinf_unc_full.C,1),1));

[wcg_hinf,wcu_hinf,info_hinf] = wcgain(CL_hinf_perf_unc);

disp('Hinf worst-case gain:')
disp(wcg_hinf)

disp('Hinf worst-case uncertainty:')
disp(wcu_hinf)

%% ===============
%analyzing worst-case uncertainty
%===========================

CL_hinf_worst_perf = usubs(CL_hinf_perf_unc,wcu_hinf);
pole(CL_hinf_worst_perf)
norm(CL_hinf_worst_perf,inf)
G_worst = usubs(G_unc,wcu_hinf);

CL_hinf_worst = feedback(G_worst,K_hinf);


x0_aug = [x0_robot;
    zeros(order(K_hinf),1)];

[y_worst,t_worst] = initial(CL_hinf_worst,x0_aug,t);

u_worst_raw = lsim(K_hinf,y_worst,t_worst);
u_worst = -u_worst_raw;
vl_worst = u_worst/2;



figure;
subplot(2,1,1)
plot(t_worst,y_worst(:,2),'LineWidth',1.5)
grid on
ylabel('\psi (rad)')
title('Worst-Case H_\infty Pitch Response')

subplot(2,1,2)
plot(t_worst,vl_worst,'LineWidth',1.5); hold on;
yline(24,'r--','+24 V');
yline(-24,'r--','-24 V');
grid on
xlabel('Time (s)')
ylabel('v_l (V)')
title('Worst-Case H_\infty Motor Voltage')



%% ============================================================
% 12. COMPARE WORST-CASE GAINS
% ============================================================

fprintf('\n================ ROBUST PERFORMANCE SUMMARY ================\n');

fprintf('LQR worst-case gain lower bound  = %.4f\n',wcg_lqr.LowerBound);
fprintf('LQR worst-case gain upper bound  = %.4f\n',wcg_lqr.UpperBound);

fprintf('Hinf worst-case gain lower bound = %.4f\n',wcg_hinf.LowerBound);
fprintf('Hinf worst-case gain upper bound = %.4f\n',wcg_hinf.UpperBound);


if wcg_hinf.UpperBound < wcg_lqr.UpperBound
    fprintf('Conclusion: Hinf has better worst-case disturbance attenuation.\n');
else
    fprintf('Conclusion: LQR has better worst-case disturbance attenuation for this output choice.\n');
end

%% ============================================================
% 13. BAR PLOT: ROBUST STABILITY AND PERFORMANCE
% ============================================================

figure('Name','Robust Stability and Worst-Case Gain','Color','w');

subplot(1,2,1)
bar([stab_lqr.LowerBound, stab_hinf.LowerBound])
grid on
set(gca,'XTickLabel',{'LQR','H_\infty'})
ylabel('Robust Stability Margin')
title('Robust Stability Margin')

subplot(1,2,2)
bar([wcg_lqr.UpperBound, wcg_hinf.UpperBound])
grid on
set(gca,'XTickLabel',{'LQR','H_\infty'})
ylabel('Worst-Case Gain')
title('Worst-Case Disturbance-to-State Gain')

%% LQR worst-case from robstab
G_lqr_destab = usubs(G_unc,destab_lqr);
A_lqr_destab = G_lqr_destab.A;
B_lqr_destab = G_lqr_destab.B;

Acl_lqr_destab = A_lqr_destab - B_lqr_destab*K_lqr;
sys_lqr_destab = ss(Acl_lqr_destab,[],eye(4),[]);

[~,t_lqr_des,x_lqr_des] = initial(sys_lqr_destab,x0_robot,t);

u_lqr_des = -x_lqr_des*K_lqr';
vl_lqr_des = u_lqr_des/2;

%% Hinf worst-case from robstab
G_hinf_destab = usubs(G_unc,destab_hinf);
CL_hinf_destab = feedback(G_hinf_destab,K_hinf);

x0_aug = [x0_robot;
    zeros(order(K_hinf),1)];

[y_hinf_des,t_hinf_des] = initial(CL_hinf_destab,x0_aug,t);

u_hinf_des_raw = lsim(K_hinf,y_hinf_des,t_hinf_des);
u_hinf_des = -u_hinf_des_raw;
vl_hinf_des = u_hinf_des/2;

%% Plot
figure('Name','Robstab Worst-Case Simulation','Color','w');

subplot(2,1,1)
plot(t_lqr_des,x_lqr_des(:,2),'LineWidth',1.5); hold on;
plot(t_hinf_des,y_hinf_des(:,2),'LineWidth',1.5);
grid on;
ylabel('\psi (rad)');
title('Pitch Response at Robstab Worst-Case Substitution');
legend('LQR worst-case','H_\infty worst-case');

subplot(2,1,2)
plot(t_lqr_des,vl_lqr_des,'LineWidth',1.5); hold on;
plot(t_hinf_des,vl_hinf_des,'LineWidth',1.5);
yline(24,'r--','+24 V');
yline(-24,'r--','-24 V');
grid on;
xlabel('Time (s)');
ylabel('v_l (V)');
title('Motor Voltage at Robstab Worst-Case Substitution');
legend('LQR','H_\infty');

%% ============================================================
% 14. SAVE FIGURES
% ============================================================

figureDir = fullfile(fileparts(mfilename('fullpath')),'figures');

if ~exist(figureDir,'dir')
    mkdir(figureDir);
end

figHandles = findobj('Type','figure');
[~,sortIdx] = sort([figHandles.Number]);
figHandles = figHandles(sortIdx);

for k = 1:numel(figHandles)
    
    fig = figHandles(k);
    figName = get(fig,'Name');
    
    if isempty(figName)
        figName = sprintf('figure_%d',fig.Number);
    end
    
    fileName = lower(regexprep(figName,'[^A-Za-z0-9]+','_'));
    fileName = regexprep(fileName,'(^_+|_+$)','');
    fileBase = sprintf('%02d_%s',k,fileName);
    
    pngPath = fullfile(figureDir,[fileBase '.png']);
    figPath = fullfile(figureDir,[fileBase '.fig']);
    
    try
        exportgraphics(fig,pngPath,'Resolution',300);
    catch
        saveas(fig,pngPath);
    end
    
    savefig(fig,figPath);
    
    fprintf('Saved figure: %s\n',pngPath);
    fprintf('Saved MATLAB figure: %s\n',figPath);
    
end


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
