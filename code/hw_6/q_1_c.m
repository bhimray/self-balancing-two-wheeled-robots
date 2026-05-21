clear; clc; close all;

%% Plant from 1(a)
s = tf('s');

G = (0.08876*s^6 - 876.1*s^5 + 1.136e7*s^4 - 4.345e10*s^3 ...
    + 4.097e14*s^2 - 2.095e17*s + 3.082e21) / ...
    (s^6 + 1021*s^5 + 7.856e7*s^4 + 5.129e10*s^3 ...
    + 1.342e15*s^2 + 3.65e17*s + 5.421e21);

G = minreal(G);

%% Desired bandwidth
fbw_des = 300;              % Hz
wbw = 2*pi*fbw_des;         % rad/s

%% Mixed sensitivity weights
%
% W1 shapes S.
% Requirement:
%   Ms <= 1.5
%   S(0) < -80 dB
%   20 dB/dec slope below bandwidth
%
% Standard form:
%   W1 = (s/Ms + wbw)/(s + wbw*A)
%
% Then:
%   1/W1(0) = A  -> desired low-frequency S
%   1/W1(inf) = Ms -> peak S bound

Ms = 1.5;
A  = 1e-4;          % gives S(0) approx -80 dB

W1 = (s/Ms + wbw)/(s + wbw*A);

%
% W2 shapes control effort CS.
% Requirement:
%   |C*S| <= 10
%
% So choose:
%   W2 = 1/10
%

W2 = 0.1;

%
% W3 shapes T.
% Requirements:
%   |T| < -3 dB at 500 Hz
%   max |T| <= 1.5
%   |T| < -40 dB as w -> infinity
%
% Choose inverse bound 1/W3:
%   low-frequency upper bound ~ 1.5
%   high-frequency upper bound ~ 0.01
%   transition near 500 Hz
%

Mt = 1.5;
At = 0.01;
wt = 2*pi*500;

W3 = (s + wt/Mt)/(At*s + wt);

%% Plot inverse weights as design targets
figure;
bodemag(1/W1, 1/W3, {1,1e6});
grid on;
legend('1/W1: desired upper bound on S', ...
       '1/W3: desired upper bound on T');
title('Mixed Sensitivity Design Targets');
saveCurrentFigure('1c_mixed_sensitivity_design_targets');

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

%% Compute sensitivity bandwidth from -3 dB crossing
w = logspace(0,6,10000);
[magS,~,wout] = bode(Sinf,w);
magS = squeeze(magS);

target = 10^(-3/20);
idx = find(magS(1:end-1) < target & magS(2:end) >= target, 1, 'first');

if ~isempty(idx)
    wbw_actual = wout(idx);
    fbw_actual = wbw_actual/(2*pi);
else
    wbw_actual = NaN;
    fbw_actual = NaN;
end

fprintf('\nActual sensitivity bandwidth = %.3f Hz\n', fbw_actual);
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

%% Plots
figure;
bodemag(Sinf,1/W1,{1,1e6});
grid on;
legend('|S|','1/|W1|');
title('Sensitivity Function vs Desired Bound');
saveCurrentFigure('1c_sensitivity_vs_bound');

figure;
bodemag(Tinf,1/W3,{1,1e6});
grid on;
legend('|T|','1/|W3|');
title('Complementary Sensitivity Function vs Desired Bound');
saveCurrentFigure('1c_complementary_sensitivity_vs_bound');

figure;
bodemag(CSinf,10*tf(1),{1,1e6});
grid on;
legend('|C_\infty S|','Control effort limit = 10');
title('Control Effort Transfer Function');
saveCurrentFigure('1c_control_effort');

figure;
margin(Linf);
grid on;
title('Loop Transfer Function with H-infinity Controller');
saveCurrentFigure('1c_hinf_loop_margin');
