%% ============================================================
% MU-SYNTHESIS / D-K ITERATION FOR UNCERTAIN TWSBR
%% ============================================================

M0 = 0.25;
H0 = 0.17;

% Uncertain parameters
M_unc = ureal('M',M0,'Range',[0.2 0.3]);
H_unc = ureal('H',H0,'Range',[0.14 0.20]);

[A1_unc,B1_unc,~,~,~] = twsbr_symbolic_ss(M_unc,H_unc);

A_unc = A1_unc;
B_unc = B1_unc(:,1);

G_unc = ss(A_unc,B_unc,eye(4),zeros(4,1));

%% Mixed-sensitivity weights
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
Wu = tf(1/(umax*4));

Mt  = 3;
At  = 5e-2;
wbt = 50;

Wt_scalar = (s + wbt/Mt)/(At*s + wbt);

Wt = 0.01*blkdiag(Wt_scalar, ...
                  Wt_scalar, ...
                  Wt_scalar, ...
                  Wt_scalar);

%% Build uncertain generalized plant
P_unc = augw(G_unc,Ws,Wu,Wt);

nmeas = 4;   % measured outputs y = x
ncon  = 1;   % one balancing input u_b

%% D-K iteration / mu-synthesis
[K_mu,CL_mu,gamma_mu,info_mu] = dksyn(P_unc,nmeas,ncon);

disp('================ MU-SYNTHESIS RESULT ================')
disp('mu-synthesis gamma:')
disp(gamma_mu)

disp('controller order:')
disp(order(K_mu))

%% Physical uncertain closed-loop
CL_mu_phys_unc = feedback(G_unc,K_mu);

%% Robust stability
[stab_mu,destab_mu,report_mu] = robstab(CL_mu_phys_unc);

disp('================ ROBSTAB: MU-SYNTHESIS ================')
disp(stab_mu)
disp(report_mu)

%% Robust performance
[perf_mu,wcu_mu,report_perf_mu] = robustperf(CL_mu);

disp('================ ROBUSTPERF: MU-SYNTHESIS ================')
disp(perf_mu)
disp(report_perf_mu)

%% Worst-case gain
[wcg_mu,wcu_wcg_mu] = wcgain(CL_mu);

disp('================ WCGAIN: MU-SYNTHESIS ================')
disp(wcg_mu)
disp(wcu_wcg_mu)


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