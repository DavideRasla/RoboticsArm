clear all; clc;
%% Robot

L1 = Link('d', 0,  'a', 0,   'alpha', pi/2);
L2 = Link('d', 0,  'a', 2,   'alpha', 0);
L3 = Link('d', 0,  'a', 2,   'alpha', -pi/2);
L4 = Link('d', 0,  'a', 0,   'alpha', -pi/2);
L5 = Link('d', 0,  'a', 0,   'alpha', pi/2, 'offset', pi/2);
L6 = Link('d', 0,  'a', 0,   'alpha', 0);
%{
deg = pi/180;

L1 = Revolute('d', 0, ...   % link length (Dennavit-Hartenberg notation)
    'a', 0, ...               % link offset (Dennavit-Hartenberg notation)
    'alpha', pi/2, ...        % link twist (Dennavit-Hartenberg notation)
    'I', [0, 0.35, 0, 0, 0, 0], ... % inertia tensor of link with respect to center of mass I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
    'r', [0, 0, 0], ...       % distance of ith origin to center of mass [x,y,z] in link reference frame
    'm', 0, ...               % mass of link
    'Jm', 200e-6, ...         % actuator inertia 
    'G', -62.6111, ...        % gear ratio
    'B', 1.48e-3, ...         % actuator viscous friction coefficient (measured at the motor)
    'Tc', [0.395 -0.435], ... % actuator Coulomb friction coefficient for direction [-,+] (measured at the motor)
    'qlim', [-160 160]*deg ); % minimum and maximum joint angle

L2 = Revolute('d', 0, 'a', 2, 'alpha', 0, ...
    'I', [0.13, 0.524, 0.539, 0, 0, 0], ...
    'r', [-0.3638, 0.006, 0.2275], ...
    'm', 17.4, ...
    'Jm', 200e-6, ...
    'G', 107.815, ...
    'B', .817e-3, ...
    'Tc', [0.126 -0.071], ...
    'qlim', [-45 225]*deg );

L3 = Revolute('d', 0.15005, 'a', 2, 'alpha', -pi/2,  ...
    'I', [0.066, 0.086, 0.0125, 0, 0, 0], ...
    'r', [-0.0203, -0.0141, 0.070], ...
    'm', 4.8, ...
    'Jm', 200e-6, ...
    'G', -53.7063, ...
    'B', 1.38e-3, ...
    'Tc', [0.132, -0.105], ...
    'qlim', [-225 45]*deg );

L4 = Revolute('d', 0.4318, 'a', 0, 'alpha', pi/2,  ...
    'I', [1.8e-3, 1.3e-3, 1.8e-3, 0, 0, 0], ...
    'r', [0, 0.019, 0], ...
    'm', 0.82, ...
    'Jm', 33e-6, ...
    'G', 76.0364, ...
    'B', 71.2e-6, ...
    'Tc', [11.2e-3, -16.9e-3], ...
    'qlim', [-110 170]*deg);

L5 = Revolute('d', 0, 'a', 0, 'alpha', -pi/2,  ...
    'I', [0.3e-3, 0.4e-3, 0.3e-3, 0, 0, 0], ...
    'r', [0, 0, 0], ...
    'm', 0.34, ...
    'Jm', 33e-6, ...
    'G', 71.923, ...
    'B', 82.6e-6, ...
    'Tc', [9.26e-3, -14.5e-3], ...
    'qlim', [-100 100]*deg );


L6= Revolute('d', 0, 'a', 0, 'alpha', 0,  ...
    'I', [0.15e-3, 0.15e-3, 0.04e-3, 0, 0, 0], ...
    'r', [0, 0, 0.032], ...
    'm', 0.09, ...
    'Jm', 33e-6, ...
    'G', 76.686, ...
    'B', 36.7e-6, ...
    'Tc', [3.96e-3, -10.5e-3], ...
    'qlim', [-266 266]*deg );
%}
qz = [0 0 0 0 0 0];
qr = [0 pi/2 -pi/2 0 0 0]; % ready pose, arm up
qs = [0 0 -pi/2 0 0 0];

L = [L1 L2 L3 L4 L5 L6];
rob = SerialLink(L , 'name', 'Giotto');



%% Getting the Image
binaryImage = CreatingBinary();

imshow(binaryImage);

%% Finding the boundaries and rescaling manually the traj

hold on;
[trajectories, B,L,N, A]  = GettingTrajectories(binaryImage);


%% Creation of a single trajectory
figure
%filtering the points in order to reduce the #
trajectories = Filtering(trajectories);

%% Mstraj for each traj


q_def = [];
path_start = [0.4 0 -0.4; -0.38 0.5 -0.4];
%p_start = mstraj(path_start, [], [1, 1]', path_start(1,:), 0.1, 0);

qpartenza = transl(path_start(1,:))*trotx(pi);




k0 = 1;
t = 1;
o = [0.3 0.3 0];
figure
plot_sphere(o, 0.2, 'y');


path_start = [0.4 0 -0.4; -0.38 0.5 -0.4];
p_start = mstraj(path_start, [], [1, 1]', path_start(1,:), 0.1, 0);


%qpartenza =   SE3(p_start) * SE3.oa([0 1 0], [0, 0, -1]);
Tp =   SE3(p_start) * SE3.oa([0 1 0], [0, 0, -1]);
qpartenza = transl(path_start(1,:))*trotx(pi);
%q_opt = AvoidanceStart(rob, k0, t, o, qpartenza,Tp );
q_opt = AvoidanceStart(rob, k0,t, o, qpartenza,p_start); 
q_def = [q_def; q_opt];

hold on;
 for k=1:length(trajectories)
    InitialPath = trajectories{k}';
    ExtendedPath = [InitialPath; zeros(1,numcols(InitialPath));];
    traj = mstraj([0 0 -2 ] + ExtendedPath(:,2:end)',[0.8 0.8 0.8], [], [0 0 -2 ] + ExtendedPath(:,1)', 0.3, 0.2);
    Tp =   SE3(traj) * SE3.oa([0 1 0], [0, 0, -1]);
    q_traj = rob.ikine6s(Tp);
    m = rob.maniplty(q_opt);
    m;
    q_def = [q_def; q_traj]; 
 end
q_def = [q_def; qz]; 
rob.plot(q_def, 'trail', {'r.', 'LineWidth', 2});

%% plotting the result (This can take a while...)

rob.plot(q_def, 'xyz','noraise', 'trail', {'r.', 'LineWidth', 2});








function q_opt = AvoidanceStart(robot, k0,t,  o, qp, p)
 syms s1 s2 s3 s4 s5 s6 ;
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


function val = objFunCA(q0dot,robot, q_old, Ve, J, Jc, obs)
    qdot = Jc*Ve + (eye(size(Jc*J))-Jc*J)*q0dot;
   % q_new = q_old + qdot'; % new q in function of q0dot 
	% compute for each joint the minimum distance from obstacles
    functional = minDistancesObstacles(robot, qdot, obs); 
    % max of minimum distance joint-obstacle
	val = -min(functional);
end



function [q0dot_optimum] = optCollisionAvoidance(robot, Ve, J, Jc, q_old, obs)  
    x0 = zeros(robot.n, 1);
    options = optimoptions('fmincon','Display','iter',...
         'Algorithm', 'active-set', 'FunctionTolerance', 1e-3);
    q0dot_optimum = fmincon(@(q0dot)objFunCA(q0dot, robot,q_old, Ve, J, Jc, obs),...
        x0,[],[],[],[],[],[],@(q0dot)nonLinearConstr(q0dot, robot, Ve, J, Jc, q_old ), options);
end







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%{

q_end = [-2.6779   -0.7073    2.5963   -2.1300   -2.1265   -2.5067]
J = rob.jacob0(q_end);
det(J)
inv(J)

rob.vellipse(q_end,'fillcolor', 'b','edgecolor', 'w', 'alpha', 0.5);
%}


%% FUNCTIONS


function binaryImage = CreatingBinary()


format long g;
format compact;
fontSize = 36;
rgbImage = imread('Images/Image4.png');

% Get the dimensions of the image.  numberOfColorBands should be = 3.
[rows, columns, numberOfColorBands] = size(rgbImage);

%imshow(rgbImage);
greenChannel = rgbImage(:, :, 2);

% Get the binaryImagepathAll = [path; zeros(1,numcols(path))];
binaryImage = greenChannel < 200;


%imshow(binaryImage);


end

function[trajectories, B,L,N, A] =  GettingTrajectories(binaryImage)
figure
hold on;
[B,L,N, A] = bwboundaries(binaryImage,4,'holes');%use 'noholes'
                                                 %to simplify
trajectories = cell(0);

%% Getting the max value for each traj
max_Arrays = cell(0);
max_X = 0;
max_Y = 0;
 
    for k =1:length(B)
       boundary = B{k};
       if length(B{k}) > 10 % Not empty traj
        %getting the max
           for i = 1:length(B{k})
               if boundary(i,1) > max_X
                   max_X = boundary(i,1);
               end
               if boundary(i,2) > max_Y
                    max_Y = boundary(i,2);
               end  
           end
           
          max_Arrays = [max_Arrays; [max_X, max_Y]]; %adding to the cell array
          max_X = 0;
          max_Y = 0;
       %getting the traj
           xy = [B{k}(:,1), B{k}(:,2)];
           trajectories = [trajectories; xy];        
       end
    end


    
%Manual rescaling: Getting the largest values of them all
    maxX = 0;
    maxY = 0;
    for i=1:length(max_Arrays)
        if max_Arrays{i}(1) > maxX
            maxX = max_Arrays{i}(1);
        end
         if max_Arrays{i}(2) > maxY
            maxY = max_Arrays{i}(2);
        end
    end
%Rescaling each traj by the max values of all, for each axis, multipling
%by two in order to obtain [0;2]

      for k =1:length(trajectories)
          for i = 1:length(trajectories{k})
           trajectories{k}(i,1) = trajectories{k}(i,1) / maxX * 2;%dividing maxX
           trajectories{k}(i,2) = trajectories{k}(i,2) / maxY * 2;%dividing maxY
          end
          plot(trajectories{k}(:,1), trajectories{k}(:,2), 'r', 'LineWidth', 2);
      end
      
  end

    %{
%rescaling into  [0:3]
figure
hold on

    for k=1:length(trajectories)
        trajectories{k} = rescale(trajectories{k}, 0,3);
        plot( trajectories{k}(:,2),  trajectories{k}(:,1), 'g', 'LineWidth', 2);
       
    end
   %}


function trajectories = Filtering(trajectories)
    V = [];
    for i = 1:length(trajectories)
         if length(trajectories{i}) > 3500 %only if necessary
            for j = 1:30:length(trajectories{i})% 7 it's the step size. A large step size  it is less precise. Anyway 7 is a good compromise
              V = [V; trajectories{i}(j,:)];   
            end
            trajectories{i} = V;    
         elseif length(trajectories{i}) > 1500 %only if necessary
            for j = 1:5:length(trajectories{i})% 7 it's the step size. A large step size  it is less precise. Anyway 7 is a good compromise
              V = [V; trajectories{i}(j,:)];   
            end
            trajectories{i} = V;    
         elseif length(trajectories{i}) > 400 %only if necessary
            for j = 1:3:length(trajectories{i})% 7 it's the step size. A large step size  it is less precise. Anyway 7 is a good compromise
              V = [V; trajectories{i}(j,:)];   
            end
            trajectories{i} = V;    
        end
        V = [];
    end
end

function [qtraj, traj, ee_traj] = TrajOpt_CollisionAvoid(robot,qi,Ti,Tf,pointsCloud, Goal)  

    dampingFactor = 0.03;
    qtraj = qi;
    ee_traj = Ti;
    for i = 2:1:size(traj,3)
        Told = ee_traj(:,:,end);
        Tnew = traj(:,:,i);
        qOld = qtraj(end,:);
        J = robot.jacob0(qOld); 
        Jc = J'*(J*J'+dampingFactor^2*eye(size(J*J')))^-1;
        Ve = tr2delta(Told, Tnew);
        qDot = Jc*Ve;
        qNew = qOld + qDot';
        for j = 1:1:robot.n
            if  qNew(j) < robot.qlim(j,1)
                qNew(j) = robot.qlim(j,1);
            end
            if  qNew(j) > robot.qlim(j,2)
                qNew(j) = robot.qlim(j,2);
            end
        end
        
        dist = minDistancesObstacles(robot, qNew, pointsCloud, Goal);
        if(min(dist) < 0.25)
            q0dot = optCollisionAvoidance(robot, Ve, J, Jc, qOld, Tnew, pointsCloud, Goal);
            qDot = Jc*Ve + (eye(size(Jc*J))-Jc*J)*q0dot;
            qNew = qOld + qDot';
        end
        qtraj=cat(1,qtraj,qNew);
        
        eeT = robot.fkine(qNew);
        ee_traj= cat(3,ee_traj,eeT);

    end 
end