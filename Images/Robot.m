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

%hold on 
%rob.plot(qz)
%axis([-6 2 -3 3 -2 5])
%rob.teach()


path_start = [0.4 0 -0.4; -0.38 0.5 -0.4];
p_start = mstraj(path_start, [], [1, 1]', path_start(1,:), 0.1, 0);

load hershey

B = hershey{'D'}
C = hershey{'I'}
dot = hershey{'.'};

path1 = [0.5*B.stroke; zeros(1,numcols(B.stroke))];
path2 = [0.5*C.stroke; zeros(1,numcols(C.stroke))];

path3 = [zeros(1,numcols(B.stroke)); 0.5*B.stroke];
path4 = [zeros(1,numcols(C.stroke)); 0.5*C.stroke];

path5 = [0.1*dot.stroke; zeros(1,numcols(dot.stroke))];

k1 = find(isnan(path1(1,:)));
k2 = find(isnan(path2(1,:)));
k3 = find(isnan(path3(1,:)));
k4 = find(isnan(path4(1,:)));

path1(:,k1) = path1(:,2: k1-1);path1(3,k1) = 0.2;

path2(:,k2) = path2(:,2: k2-1);path2(3,k2) = 0.2;

path3(:,k3) = path3(:,2: k3-1);path3(3,k3) = 0.2;

path4(:,k4) = path4(:,2: k4-1);path4(3,k4) = 0.2;

traj1 = mstraj(path1(:,2:end)',[0.8 0.8 0.8], [], path1(:,1)', 0.3, 0.5);
traj2 = mstraj(path2(:,2:end)',[0.8 0.8 0.8], [], path2(:,1)', 0.3, 0.5);
traj3 = mstraj(path3(:,2:end)',[0.8 0.8 0.8], [], path3(:,1)', 0.3, 0.5);
traj4 = mstraj(path4(:,2:end)',[0.8 0.8 0.8], [], path4(:,1)', 0.3, 0.5);
traj5 = mstraj(path5(:,2:end)',[0.8 0.8 0.8], [], path5(:,1)', 0.02, 0.2);

%plot3(traj(:,1), traj(:,2), traj(:,3));

Tp1 = SE3(-0.6, 0.3, -0.4) * SE3(traj1) * SE3.oa( [0 1 0], [0 0 -1]);
Tp2 = SE3(-0.2, 0.3, -0.4) * SE3(traj2) * SE3.oa( [0 1 0], [0 0 -1]);

Tp3 = SE3(-0.5, -0.3, 0.1) * SE3(traj3) * SE3.oa( [0 1 0], [-1 0 0]);

Tp4 = SE3(-0.5, 0.4, 0.1) * SE3(traj4) * SE3.oa( [0 1 0], [0 0 -1]);
Tp5 = SE3(0, 0.6, -0.4) * SE3(traj5) * SE3.oa( [0 1 0], [0 0 -1]);

q1 = rob.ikine6s(Tp1);
q2 = rob.ikine6s(Tp2);
q3 = rob.ikine6s(Tp3);
q4 = rob.ikine6s(Tp4);
qend = rob.ikine6s(Tp5);


k0 = 1;
t = 5;
o = [0.2 0.13 (-0.4 - 0.02)];
qpartenza = transl(path_start(1,:))*trotx(pi);

q_opt = optimize(rob, k0, t, o, qpartenza,p_start );
qdef = [q_opt; q1; q2;q3; q4; qend];

option = {'k.', 'LineWidth', 1};
plot_sphere(o, 0.2, 'y');

rob.plot(qdef, 'trail', {'r.', 'LineWidth', 2});


function q_opt = optimize(robot, k0, t, o, qp, p)
    syms s1 s2 s3 s4 s5 s6;
    joints = [s1 s2 s3 s4 s5 s6];
    q_opt = [];
    
    omega_q = norm(double(robot.fkine(joints))*[0 0 0 1]' - [o 1]');
    omega_dot_q = gradient(omega_q, joints);
    
    for i= 1:t
        if i == 1
            q_prec = robot.ikine6s(qp);
            J = robot.jacob0(q_prec);
            ve = (p(1,:) - [0.4 0 -0.4])/(1/t);
        else
            J = robot.jacob0(q_opt(i-1,:));
            q_prec = q_opt(i-1, :);
            
            ve = (p(i,:) - p(i-1, :))/(1/t);
        end
        q0_dot = k0 .*double(subs(omega_dot_q, joints, q_prec));
        J_right = J' * inv(J*J');
        
        q_dot = J_right * [ve(1,:) zeros(1,3)]' + (eye(6) - J_right * J)*q0_dot;
        q_opt = [q_opt; q_prec + q_dot' + (1/t)];
    end
    
end

