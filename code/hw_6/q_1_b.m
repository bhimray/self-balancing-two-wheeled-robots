clear; clc; close all;

%% Transfer function from 1(a)
s = tf('s');

G = (0.08876*s^6 - 876.1*s^5 + 1.136e7*s^4 - 4.345e10*s^3 ...
    + 4.097e14*s^2 - 2.095e17*s + 3.082e21) / ...
    (s^6 + 1021*s^5 + 7.856e7*s^4 + 5.129e10*s^3 ...
    + 1.342e15*s^2 + 3.65e17*s + 5.421e21);

G = minreal(G);

%% Frequency grid
w = logspace(0,6,5000);   % rad/s

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

%% Display result
fprintf('\nBest PI controller found:\n');
fprintf('Kp = %.6g\n', best_Kp);
fprintf('Ki = %.6g\n', best_Ki);
fprintf('Bandwidth = %.6f Hz\n', best_bw);
fprintf('Gain margin = %.6f\n', best_GM);
fprintf('Phase margin = %.6f deg\n', best_PM);

CPI = minreal(best_C)

%% Final loop, sensitivity, complementary sensitivity
Lpi = minreal(G*CPI);
Spi = feedback(1,Lpi);
Tpi = feedback(Lpi,1);

%% Plots
figure;
margin(Lpi);
grid on;
title('PI Loop Transfer Function Margin Plot');
saveCurrentFigure('1b_pi_loop_margin');

figure;
bodemag(Spi,{1,1e6});
grid on;
yline(-3,'r--','-3 dB');
title('Sensitivity Function S for PI Controller');
saveCurrentFigure('1b_pi_sensitivity');

figure;
step(feedback(G*CPI,1));
grid on;
title('Closed-loop Step Response with PI Controller');
saveCurrentFigure('1b_pi_step_response');
