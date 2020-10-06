clear all; clc;
%% Robot

L1 = Link('d', 0,  'a', 0,   'alpha', pi/2);
L2 = Link('d', 0,  'a', 1,   'alpha', 0);
L3 = Link('d', 0,  'a', 1,   'alpha', -pi/2);
L4 = Link('d', 0,  'a', 0,   'alpha', -pi/2);
L5 = Link('d', 0,  'a', 0,   'alpha', pi/2, 'offset', pi/2);
L6 = Link('d', 0,  'a', 0,   'alpha', 0);
qz = [0 0 0 0 0 0];
L = [L1 L2 L3 L4 L5 L6];
rob = SerialLink(L , 'name', 'WALLe');

%% Plotting robot in a default position

rob.plot(qz);

%path1 = [0.5*B.stroke; zeros(1,numcols(B.stroke))];
qn = [0.4 0 -0.4 0 0 0 ];

T0 = transl(0.4, 0.2, 0) * trotx(pi);
T1 = transl(-0.4, -0.2, 0.3) * troty(pi/2) * trotz(-pi/2);

% and a smooth sequence between them in 50 steps is

T = ctraj(T0, T1, 50);
q_traj = rob.ikine6s(T);
rob.plot(q_traj);
%k1 = find(isnan(path1(1,:)));

%path1(:,k1) = path1(:,2: k1-1);path1(3,k1) = 0.2;



%traj1 = mstraj(qz(:,2:end)',[0.8 0.8 0.8], [], qz(:,1)', 0.3, 0.5);


%plot3(traj(:,1), traj(:,2), traj(:,3));

%Tp1 = SE3(-0.6, 0.3, -0.4) * SE3(traj1) * SE3.oa( [0 1 0], [0 0 -1]);

%q = rob.ikine6s(qz);
q_end = [-2.6779   -0.7073    2.5963   -2.1300   -2.1265   -2.5067]
J = rob.jacob0(q_end);
det(J)
inv(J)

rob.vellipse(q_end,'fillcolor', 'b','edgecolor', 'w', 'alpha', 0.5);

