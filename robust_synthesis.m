clear; clc; close all;

%% =========================================================
% 0) SOLVER SETUP
% ==========================================================
% Keep path changes local to this MATLAB session. Calling savepath here can
% make a broken solver/YALMIP path persistent across unrelated projects.
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

%% =========================================================
% 1) NOMINAL PARAMETERS
% ==========================================================
M0 = 0.5;      % wheel/base mass
m0 = 0.2;      % body mass
b0 = 0.1;      % viscous friction
I0 = 0.006;    % body inertia
g  = 9.81;     % gravity
l0 = 0.3;      % COM height

% Uncertainty bounds
m_range = [0.85*m0, 1.15*m0];
I_range = [0.85*I0, 1.15*I0];
l_range = [0.90*l0, 1.10*l0];
b_range = [0.75*b0, 1.25*b0];

%% =========================================================
% 2) HELPER FUNCTION TO BUILD A,B
% ==========================================================
buildAB = @(M,m,b,I,g,l) localBuildAB(M,m,b,I,g,l);

% Nominal system
[A0,B0] = buildAB(M0,m0,b0,I0,g,l0);

disp('Nominal open-loop poles:')
disp(eig(A0))

%% =========================================================
% 3) NOMINAL LQR FOR BASELINE COMPARISON
% ==========================================================
Q = diag([10 1 100 1]);
R = 0.1;
K_lqr = lqr(A0,B0,Q,R);

Acl_lqr = A0 - B0*K_lqr;
disp('Nominal LQR closed-loop poles:')
disp(eig(Acl_lqr))

%% =========================================================
% 4) BUILD POLYTOPIC VERTEX SET
% ==========================================================
m_vals = m_range;
I_vals = I_range;
l_vals = l_range;
b_vals = b_range;

A_vertices = {};
B_vertices = {};
idx = 1;

for im = 1:2
    for iI = 1:2
        for il = 1:2
            for ib = 1:2
                m = m_vals(im);
                I = I_vals(iI);
                l = l_vals(il);
                b = b_vals(ib);

                [A,B] = buildAB(M0,m,b,I,g,l);

                A_vertices{idx} = A;
                B_vertices{idx} = B;
                idx = idx + 1;
            end
        end
    end
end

nv = numel(A_vertices);
fprintf('Number of uncertainty vertices: %d\n', nv);

%% =========================================================
% 5) ROBUST LMI STATE-FEEDBACK SYNTHESIS
%    u = -Kx
%
% Find P > 0, Y such that for every vertex i:
% A_i P + P A_i' - B_i Y - Y' B_i' < 0
%
% Then K = Y / P
% ==========================================================
n = size(A0,1);
m_in = size(B0,2);

P = sdpvar(n,n,'symmetric');
Y = sdpvar(m_in,n,'full');

epsP = 1e-4;
epsL = 1e-4;

Constraints = [P >= epsP*eye(n)];

for i = 1:nv
    Ai = A_vertices{i};
    Bi = B_vertices{i};

    LMI_i = Ai*P + P*Ai' - Bi*Y - Y'*Bi';
    Constraints = [Constraints, LMI_i <= -epsL*eye(n)];
end

% Optional mild objective: keep the Lyapunov matrix scale moderate.
Objective = trace(P);

solverCandidates = {'sdpt3', 'sedumi', 'mosek'};
sol = [];
solverUsed = '';
solverReports = {};

for k = 1:numel(solverCandidates)
    solverName = solverCandidates{k};

    if ~localSolverOnPath(solverName)
        solverReports{end+1} = sprintf('%s: not found on MATLAB path', solverName); %#ok<SAGROW>
        continue
    end

    opts = sdpsettings('solver', solverName, 'verbose', 1);
    sol = optimize(Constraints, Objective, opts);
    solverReports{end+1} = sprintf('%s: %s', solverName, sol.info); %#ok<SAGROW>

    if sol.problem == 0
        solverUsed = solverName;
        break
    end
end

if isempty(sol) || sol.problem ~= 0
    fprintf('\nSolver attempts:\n');
    fprintf('  %s\n', solverReports{:});
    error(['LMI optimization failed. If SDPT3 reports "Undefined function ''mexmat''", ' ...
           'your SDPT3/YALMIP path is broken or incompatible. Reinstall/update YALMIP ' ...
           'and SDPT3, then restart MATLAB and run yalmip(''clear'').']);
end

fprintf('LMI optimization solved with %s.\n', solverUsed);

P_val = value(P);
Y_val = value(Y);
K_lmi = Y_val / P_val;

disp('LMI-based robust state-feedback gain K:')
disp(K_lmi)

%% =========================================================
% 6) CHECK CLOSED-LOOP POLES AT ALL VERTICES
% ==========================================================
fprintf('\nClosed-loop poles for all vertices using K_lmi:\n');
stable_all = true;

for i = 1:nv
    Ai = A_vertices{i};
    Bi = B_vertices{i};
    Acl_i = Ai - Bi*K_lmi;
    ev = eig(Acl_i);

    fprintf('Vertex %2d max real pole = %+8.5f\n', i, max(real(ev)));

    if any(real(ev) >= 0)
        stable_all = false;
    end
end

if stable_all
    fprintf('\nAll vertices are stable under the LMI controller.\n');
else
    fprintf('\nAt least one vertex is not stable under the LMI controller.\n');
end

%% =========================================================
% 7) COMPARE NOMINAL RESPONSES: LQR vs LMI
% ==========================================================
sys_lqr = ss(A0 - B0*K_lqr, B0, eye(4), zeros(4,1));
sys_lmi = ss(A0 - B0*K_lmi, B0, eye(4), zeros(4,1));

figure;
step(sys_lqr, 5);
hold on;
step(sys_lmi, 5);
grid on;
legend('LQR','LMI robust');
title('Nominal Closed-Loop Step Response: LQR vs LMI');

%% =========================================================
% 8) OPTIONAL: CHECK A WORST-LOOKING VERTEX RESPONSE
% ==========================================================
worst_idx = 1;
worst_real = -Inf;

for i = 1:nv
    Acl_i = A_vertices{i} - B_vertices{i}*K_lmi;
    mr = max(real(eig(Acl_i)));
    if mr > worst_real
        worst_real = mr;
        worst_idx = i;
    end
end

A_w = A_vertices{worst_idx};
B_w = B_vertices{worst_idx};

sys_worst_lmi = ss(A_w - B_w*K_lmi, B_w, eye(4), zeros(4,1));
sys_worst_lqr = ss(A_w - B_w*K_lqr, B_w, eye(4), zeros(4,1));

figure;
step(sys_worst_lqr,'r--',sys_worst_lmi,'b',5);
grid on;
legend('LQR at worst vertex','LMI at worst vertex');
title(sprintf('Worst-Vertex Comparison (vertex %d)', worst_idx));

%% =========================================================
% LOCAL FUNCTION
% ==========================================================
function [A,B] = localBuildAB(M,m,b,I,g,l)
    q = (M+m)*(I+m*l^2) - (m*l)^2;

    A = [0 1 0 0;
         0 -(I+m*l^2)*b/q   (m^2*g*l^2)/q   0;
         0 0 0 1;
         0 -(m*l*b)/q       m*g*l*(M+m)/q   0];

    B = [0;
         (I+m*l^2)/q;
         0;
         m*l/q];
end

function tf = localSolverOnPath(solverName)
    switch lower(solverName)
        case 'sdpt3'
            tf = exist('sqlp', 'file') == 2;
        case 'sedumi'
            tf = exist('sedumi', 'file') == 2;
        case 'mosek'
            tf = exist('mosekopt', 'file') == 3 || exist('mosekopt', 'file') == 2;
        otherwise
            tf = exist(solverName, 'file') == 2;
    end
end
