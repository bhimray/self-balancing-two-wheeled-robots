clear; clc; close all;

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
saveCurrentFigure('1a_frd_fit');
