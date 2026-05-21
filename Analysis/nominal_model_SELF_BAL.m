clear; clc; close all;

%% =====================================================
% SELF-BALANCING SUBSYSTEM
% ======================================================

A = [0 0 1 0;
    0 0 0 1;
    0 55.540 -0.610 0.610;
    0 62.794 -0.316 0.316];

B = [0      0;
    0      0;
    9.385  9.385;
    -4.857 -4.857];

%% Outputs
C = eye(4);

D = zeros(4,2);

P_nom = ss(A,B,C,D);

%% =====================================================
% OPEN-LOOP ANALYSIS
% ======================================================

disp('Open-loop poles:')
eig(A)

disp('Controllability rank:')
rank(ctrb(A,B))

disp('Observability rank:')
rank(obsv(A,C))