A2 = [0 1;
    0 -11.742];

B2 = [0 0;
    61.146 -61.146];

C2 = [1 0];

D2 = [0 0];

P_steer = ss(A2,B2,C2,D2);

disp('Steering subsystem poles:')
eig(A2)

disp('Controllability rank:')
rank(ctrb(A2,B2))

disp('Observability rank:')
rank(obsv(A2,C2))