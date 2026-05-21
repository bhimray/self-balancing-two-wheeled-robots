%% Steering subsystem
A2 = [0 1;
    0 -11.742];

B2 = [0 0;
    61.146 -61.146];

C2 = [1 0];
D2 = 0;

disp('Steering open-loop poles:')
disp(eig(A2))

disp('Steering controllability rank:')
disp(rank(ctrb(A2,B2)))

%% LQR for steering
Q2 = diag([100 1]);
R2 = diag([1 1]);

K_steer = lqr(A2,B2,Q2,R2);

disp('Steering LQR gain:')
disp(K_steer)

Acl2 = A2 - B2*K_steer;

disp('Steering closed-loop poles:')
disp(eig(Acl2))