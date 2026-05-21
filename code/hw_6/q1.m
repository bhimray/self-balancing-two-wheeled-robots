clear; clc; close all;


%% 1.a
load("npresp.mat", "Gfr")
whos
order = 6;

G = fitfrd(Gfr, order);

G = minreal(tf(G));

disp('Fitted transfer function G(s):')
G

figure;
bode(Gfr,'b',G,'r--');
grid on;
legend('Experimental FRD','Fitted G(s)');
title(['Nano-positioning Stage: FRD Fit, Order = ', num2str(order)]);
saveCurrentFigure('2a_frd_fit');


%% 1.b
%% Transfer function from a
s = tf('s');

G = (0.08876*s^6 - 876.1*s^5 + 1.136e7*s^4 - 4.345e10*s^3 ...
    + 4.097e14*s^2 - 2.095e17*s + 3.082e21) / ...
    (s^6 + 1021*s^5 + 7.856e7*s^4 + 5.129e10*s^3 ...
    + 1.342e15*s^2 + 3.65e17*s + 5.421e21);

G = minreal(G);

%% Frequency grid
w = logspace(-4,6,8000);   % rad/s

%% Search ranges for PI controller
Kp_list = logspace(-4,4,200);
Ki_list = logspace(-2,8,250);

best_bw = -inf;
best_Kp = NaN;
best_Ki = NaN;
best_C  = [];

for i = 1:length(Kp_list)
    
    Kp = Kp_list(i);
    
    for j = 1:length(Ki_list)
        
        Ki = Ki_list(j);
        
        Cpi = Kp + Ki/s;
        L = minreal(G*Cpi);
        
        %% Check closed-loop stability
        CL = feedback(L,1);
        if ~isstable(CL)
            continue;
        end
        
        %% Gain and phase margins
        [GM, PM] = margin(L);
        
        if isempty(GM) || isempty(PM) || isnan(GM) || isnan(PM)
            continue;
        end
        
        if GM < 1.5 || PM < 75
            continue;
        end
        
        %% Sensitivity function
        S = feedback(1,L);
        
        [magS,~,wout] = bode(S,w);
        magS = squeeze(magS);
        
        %% Find bandwidth where |S| crosses -3 dB from below
        target = 10^(-3/20);
        
        idx = find(magS(1:end-1) < target & magS(2:end) >= target, 1, 'first');
        
        if isempty(idx)
            continue;
        end
        
        wbw = wout(idx);          % rad/s
        fbw = wbw/(2*pi);         % Hz
        
        %% Store best controller
        if fbw > best_bw
            best_bw = fbw;
            best_Kp = Kp;
            best_Ki = Ki;
            best_C  = Cpi;
            best_GM = GM;
            best_PM = PM;
        end
        
    end
end


fprintf('\nBest PI controller found:\n');
fprintf('Kp = %.6g\n', best_Kp);
fprintf('Ki = %.6g\n', best_Ki);
fprintf('Bandwidth = %.6f Hz\n', best_bw);
fprintf('Gain margin = %.6f\n', best_GM);
fprintf('Phase margin = %.6f deg\n', best_PM);

CPI = minreal(best_C)

Lpi = minreal(G*CPI);
Spi = feedback(1,Lpi);
Tpi = feedback(Lpi,1);

%% Plots
figure;
margin(Lpi);
grid on;
title('PI Loop Transfer Function Margin Plot');
saveCurrentFigure('pi_loop_margin');

figure;
bodemag(Spi,{1,1e6});
grid on;
yline(-3,'r--','-3 dB');
title('Sensitivity Function S for PI Controller');
saveCurrentFigure('pi_sensitivity');

figure;
step(feedback(G*CPI,1));
grid on;
title('Closed-loop Step Response with PI Controller');
saveCurrentFigure('pi_step_response');

%% 1.c mixed sensitivity weights

fbw_des = 250;                 % desired BW in Hz
ws = 2*pi*fbw_des;             % rad/s

%% Sensitivity bound Bs
As = 1e-4;                     % low-frequency bound, approx < -80 dB
Ms = 1.5;                      % peak sensitivity bound

Bs = (Ms*s + As*ws)/(s + ws);  % desired upper bound on |S|
W1 = 1/Bs;                     % mixsyn uses reciprocal weight

%% Control effort bound Bc
Bc = 10;                       % desired upper bound on |C*S|
W2 = 1/Bc;                     % = 0.1

%% Complementary sensitivity bound Bt
ft = 500;                      % Hz
wt = 2*pi*ft;

Mt = 1.5;                      % peak T bound
At = 0.01;                     % high-frequency T bound = -40 dB

Bt = (Mt*wt + At*s)/(s + wt);  % desired upper bound on |T|
W3 = 1/Bt;                     % mixsyn reciprocal weight

%% Plot design bounds
figure;
bodemag(Bs, Bt, Bc*tf(1), {1,1e6});
grid on;
legend('B_s: upper bound on |S|', ...
    'B_t: upper bound on |T|', ...
    'B_c: upper bound on |CS|');
title('Performance Bounds');
saveCurrentFigure('2c_performance_bounds');

%% H-infinity mixed sensitivity synthesis
[Cinf, CL, gamma] = mixsyn(G,W1,W2,W3);
Cinf = minreal(Cinf);

fprintf('\nMixed sensitivity H-infinity controller:\n');
Cinf
fprintf('\nAchieved gamma = %.6f\n', gamma);

%% Closed-loop functions
Linf = minreal(G*Cinf);
Sinf = feedback(1,Linf);
Tinf = feedback(Linf,1);
CSinf = minreal(Cinf*Sinf);

%% weighted plots
WS  = minreal(W1*Sinf);
WCS = minreal(W2*CSinf);
WT  = minreal(W3*Tinf);


fprintf('Peak |S| = %.4f\n', norm(Sinf,inf));
fprintf('Peak |T| = %.4f\n', norm(Tinf,inf));
fprintf('Peak |C*S| = %.4f\n', norm(CSinf,inf));

%% Check T at 500 Hz
T_500 = abs(freqresp(Tinf,2*pi*500));
T_500_db = 20*log10(T_500);
fprintf('|T(j2pi500)| = %.4f = %.2f dB\n', T_500, T_500_db);

%% Check DC gain of S
S0 = abs(evalfr(Sinf,0));
S0_db = 20*log10(S0);
fprintf('|S(0)| = %.4e = %.2f dB\n', S0, S0_db);

figure;
bodemag(WS, WCS, WT, {1,1e6});
grid on;
yline(0,'k--','0 dB bound');
legend('|W_1S|','|W_2CS|','|W_3T|','0 dB');
title('Weighted Mixed-Sensitivity Verification');
saveCurrentFigure('weighted_mixed_sensitivity');

%% Physical bound plots
figure;
bodemag(Sinf, Bs, {1,1e6});
grid on;
legend('|S|','B_s');
title('Sensitivity vs Bound');
saveCurrentFigure('sensitivity_vs_Bs');

figure;
bodemag(Tinf, Bt, {1,1e6});
grid on;
legend('|T|','B_t');
title('Complementary Sensitivity vs Bound');
saveCurrentFigure('T_vs_Bt');

figure;
bodemag(CSinf, Bc*tf(1), {1,1e6});
grid on;
legend('|C_\infty S|','B_c = 10');
title('Control Effort vs Bound');
saveCurrentFigure('CS_vs_Bc');

%% 1.d

% PI controller from 1(b)
CPI = (0.217112*s + 0.01)/s;

%% Loop transfer functions
Lpi   = minreal(G*CPI);
Linf  = minreal(G*Cinf);

%% Gain and phase margins

[GM_pi, PM_pi, Wcg_pi, Wcp_pi] = margin(Lpi);
[GM_inf, PM_inf, Wcg_inf, Wcp_inf] = margin(Linf);

fprintf('\n===== PI Controller Margins =====\n');
fprintf('GM_PI = %.4f\n', GM_pi);
fprintf('GM_PI = %.2f dB\n', 20*log10(GM_pi));
fprintf('PM_PI = %.2f deg\n', PM_pi);
fprintf('Gain crossover frequency = %.4f rad/s\n', Wcp_pi);
fprintf('Phase crossover frequency = %.4f rad/s\n', Wcg_pi);

fprintf('\n===== H-infinity Controller Margins =====\n');
fprintf('GM_Hinf = %.4f\n', GM_inf);
fprintf('GM_Hinf = %.2f dB\n', 20*log10(GM_inf));
fprintf('PM_Hinf = %.2f deg\n', PM_inf);
fprintf('Gain crossover frequency = %.4f rad/s\n', Wcp_inf);
fprintf('Phase crossover frequency = %.4f rad/s\n', Wcg_inf);

%% Bode plot of controllers

figure;
bode(CPI,'b',Cinf,'r--',{1,1e6});
grid on;
legend('C_{PI}','C_{\infty}');
title('Bode Plot of PI and H_{\infty} Controllers');
saveCurrentFigure('controller_bode');

%% Bode plot of loop transfer functions

figure;
bode(Lpi,'b',Linf,'r--',{1,1e6});
grid on;
legend('G C_{PI}','G C_{\infty}');
title('Bode Plot of Loop Transfer Functions');
saveCurrentFigure('loop_transfer_bode');
