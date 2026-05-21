%% Problem 1(d)

% PI controller from 1(b)
CPI = (0.217112*s + 0.01)/s;

% H-infinity controller from 1(c)
% Cinf is already computed from mixsyn

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
saveCurrentFigure('1d_controller_bode');

%% Bode plot of loop transfer functions

figure;
bode(Lpi,'b',Linf,'r--',{1,1e6});
grid on;
legend('G C_{PI}','G C_{\infty}');
title('Bode Plot of Loop Transfer Functions');
saveCurrentFigure('1d_loop_transfer_bode');

%% Optional margin plots

figure;
margin(Lpi);
grid on;
title('Margins of PI Loop Transfer Function');
saveCurrentFigure('1d_pi_margin');

figure;
margin(Linf);
grid on;
title('Margins of H_{\infty} Loop Transfer Function');
saveCurrentFigure('1d_hinf_margin');
