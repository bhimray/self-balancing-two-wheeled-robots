%% ============================================================
% 14. MONTE CARLO SIMULATION OVER PARAMETER UNCERTAINTY
% ============================================================

Nmc = 10000;              % number of random samples
rng(1);                 % reproducibility

t = 0:0.001:10;
x0_robot = [0;
    0.1;
    0.3;
    0.3];

M_samples = 0.2 + (0.3-0.2)*rand(Nmc,1);
H_samples = 0.112 + (0.20-0.14)*rand(Nmc,1);

maxPsi_lqr  = zeros(Nmc,1);
maxPsi_hinf = zeros(Nmc,1);

maxVl_lqr   = zeros(Nmc,1);
maxVl_hinf  = zeros(Nmc,1);

stable_lqr  = true(Nmc,1);
stable_hinf = true(Nmc,1);

figure('Name','Monte Carlo Pitch Response','Color','w');
subplot(1,2,1); hold on; grid on;
title('LQR Monte Carlo: Pitch Angle')
xlabel('Time (s)')
ylabel('\psi (rad)')

subplot(1,2,2); hold on; grid on;
title('H_\infty Monte Carlo: Pitch Angle')
xlabel('Time (s)')
ylabel('\psi (rad)')

for k = 1:Nmc
    
    Mval = M_samples(k);
    Hval = H_samples(k);
    
    [A1_k,B1_k,~,~,~] = twsbr_symbolic_ss(Mval,Hval);
    
    Ak = A1_k;
    Bk = B1_k(:,1);
    Gk = ss(Ak,Bk,C,D);
    
    %% LQR
    Acl_lqr_k = Ak - Bk*K_lqr;
    stable_lqr(k) = all(real(eig(Acl_lqr_k)) < 0);
    
    sys_lqr_k = ss(Acl_lqr_k,[],eye(4),[]);
    
    try
        [~,t_lqr_k,x_lqr_k] = initial(sys_lqr_k,x0_robot,t);
        
        u_lqr_k = -x_lqr_k*K_lqr';
        vl_lqr_k = u_lqr_k/2;
        
        maxPsi_lqr(k) = max(abs(x_lqr_k(:,2)));
        maxVl_lqr(k)  = max(abs(vl_lqr_k));
        
        subplot(1,2,1)
        plot(t_lqr_k,x_lqr_k(:,2),'Color',[0.7 0.7 1]);
        
    catch
        stable_lqr(k) = false;
        maxPsi_lqr(k) = Inf;
        maxVl_lqr(k)  = Inf;
    end
    
    %% Hinf
    CL_hinf_k = feedback(Gk,K_hinf);
    stable_hinf(k) = all(real(pole(CL_hinf_k)) < 0);
    
    x0_aug = [x0_robot;
        zeros(order(K_hinf),1)];
    
    try
        [y_hinf_k,t_hinf_k,~] = initial(CL_hinf_k,x0_aug,t);
        
        u_hinf_raw_k = lsim(K_hinf,y_hinf_k,t_hinf_k);
        u_hinf_k = -u_hinf_raw_k;
        vl_hinf_k = u_hinf_k/2;
        
        maxPsi_hinf(k) = max(abs(y_hinf_k(:,2)));
        maxVl_hinf(k)  = max(abs(vl_hinf_k));
        
        subplot(1,2,2)
        plot(t_hinf_k,y_hinf_k(:,2),'Color',[1 0.75 0.75]);
        
    catch
        stable_hinf(k) = false;
        maxPsi_hinf(k) = Inf;
        maxVl_hinf(k)  = Inf;
    end
    
end

%% Overlay nominal responses
subplot(1,2,1)
plot(results.Nominal.t_lqr,results.Nominal.x_lqr(:,2),'b','LineWidth',2)
legend('MC samples','Nominal','Location','best')

subplot(1,2,2)
plot(results.Nominal.t_hinf,results.Nominal.x_hinf(:,2),'r','LineWidth',2)
legend('MC samples','Nominal','Location','best')

%% ============================================================
% 15. MONTE CARLO SUMMARY
% ============================================================

fprintf('\n================ MONTE CARLO SUMMARY ================\n');

fprintf('Number of samples = %d\n',Nmc);

fprintf('\n----- LQR -----\n');
fprintf('Stable samples      = %d / %d\n',sum(stable_lqr),Nmc);
fprintf('Max over MC |psi|   = %.4f rad\n',max(maxPsi_lqr));
fprintf('Mean max |psi|      = %.4f rad\n',mean(maxPsi_lqr(isfinite(maxPsi_lqr))));
fprintf('Max over MC |v_l|   = %.4f V\n',max(maxVl_lqr));
fprintf('Mean max |v_l|      = %.4f V\n',mean(maxVl_lqr(isfinite(maxVl_lqr))));

fprintf('\n----- Hinf -----\n');
fprintf('Stable samples      = %d / %d\n',sum(stable_hinf),Nmc);
fprintf('Max over MC |psi|   = %.4f rad\n',max(maxPsi_hinf));
fprintf('Mean max |psi|      = %.4f rad\n',mean(maxPsi_hinf(isfinite(maxPsi_hinf))));
fprintf('Max over MC |v_l|   = %.4f V\n',max(maxVl_hinf));
fprintf('Mean max |v_l|      = %.4f V\n',mean(maxVl_hinf(isfinite(maxVl_hinf))));

%% ============================================================
% 16. MONTE CARLO HISTOGRAMS
% ============================================================

figure('Name','Monte Carlo Metrics','Color','w');

subplot(2,2,1)
histogram(maxPsi_lqr(isfinite(maxPsi_lqr)),20); hold on;
histogram(maxPsi_hinf(isfinite(maxPsi_hinf)),20);
grid on;
xlabel('max |\psi| (rad)')
ylabel('Count')
title('Pitch Peak Distribution')
legend('LQR','H_\infty')

subplot(2,2,2)
histogram(maxVl_lqr(isfinite(maxVl_lqr)),20); hold on;
histogram(maxVl_hinf(isfinite(maxVl_hinf)),20);
grid on;
xlabel('max |v_l| (V)')
ylabel('Count')
title('Voltage Peak Distribution')
legend('LQR','H_\infty')

subplot(2,2,3)
scatter(M_samples,maxPsi_lqr,25,'filled'); hold on;
scatter(M_samples,maxPsi_hinf,25,'filled');
grid on;
xlabel('M (kg)')
ylabel('max |\psi| (rad)')
title('Pitch Peak vs Mass')
legend('LQR','H_\infty')

subplot(2,2,4)
scatter(H_samples,maxPsi_lqr,25,'filled'); hold on;
scatter(H_samples,maxPsi_hinf,25,'filled');
grid on;
xlabel('H (m)')
ylabel('max |\psi| (rad)')
title('Pitch Peak vs Height')
legend('LQR','H_\infty')


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