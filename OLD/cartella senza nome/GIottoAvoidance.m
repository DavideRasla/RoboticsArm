clear all; clc;
%% Robot


global obs r p_curr
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


qz = [0 0 0 0 0 0];
qr = [0 pi/2 -pi/2 0 0 0]; % ready pose, arm up
qs = [0 0 -pi/2 0 0 0];

L = [L1 L2 L3 L4 L5 L6];
rob = SerialLink(L , 'name', 'Giotto');
%rob.plot(qr);


%% Getting the Image
binaryImage = CreatingBinary();

imshow(binaryImage);

%% Finding the boundaries and rescaling manually the traj

hold on;
[trajectories, B,L,N, A]  = GettingTrajectories(binaryImage);


%% Creation of a single trajectory
%filtering the points in order to reduce the #
figure
trajectories = Filtering(trajectories);

%% Mstraj for each traj


q_def = [];

q_def = [q_def; qr];

hold on;
 for k=1:length(trajectories)
    InitialPath = trajectories{k}';
    ExtendedPath = [InitialPath; zeros(1,numcols(InitialPath));];
    traj = mstraj([0 0 -2 ] + ExtendedPath(:,2:end)',[0.8 0.8 0.8], [], [0 0 -2 ] + ExtendedPath(:,1)', 0.3, 0.2);
    Tp =   SE3(traj) * SE3.oa([0 1 0], [0, 0, -1]);
    q_traj = rob.ikine6s(Tp);
    %plotting manipulability
   % figure
   % hold on;
    m = rob.maniplty(q_traj);
    x = 0:pi/10:2*pi;
   % plot(m,'g');
    q_def = [q_def; q_traj]; 
 end
tmax = 5;
t=[0:100]'/100*tmax;
[q,qd,qdd] = jtraj(q_traj(end,:), qr, t);
q_rest = [q_def(end,:); q]; 

q_def = [q_def; q_rest]; 
%% plotting the result (This can take a while...)
axis([-3 3 -3 3 -2 5]);
%figure

%rob.plot(q_def, 'xyz','noraise', 'trail', {'r.', 'LineWidth', 2});


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
r = 0.2;
time = 0:0.1:5;
[tj] = jtraj(qz, q_def(1,:), time);
idx = round(length(time)/2);
TO = rob.fkine(q_def(idx, :));
obs = transl(TO);
a = plot_sphere(obs', r, 'blue');
rob.plot(tj, 'trail' , '.k');


H = rob.fkine(q_def);
pavoid = AvoidObs(H);

th = rob.ikine6s(pavoid);
rob.plot(th,'xyz','noraise', 'trail', {'r.', 'LineWidth', 2} );


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{

q_end = [-2.6779   -0.7073    2.5963   -2.1300   -2.1265   -2.5067]
J = rob.jacob0(q_end);
det(J)
inv(J)

rob.vellipse(q_end,'fillcolor', 'b','edgecolor', 'w', 'alpha', 0.5);
%}


%% FUNCTIONS
function pjt = AvoidObs(h)

global obs r
about(h)
time = 0:0.1:5;
pj = transl(h);
pj_opt = zeros(length(time), 3);
opt_flag = false;
for i = 2:1:length(time)-1
    p_curr = pj(i, :);
    p_curr
    p = pj(i+1, :); % Sto un passo avanti
    % Vedo se devo ottimizzare
    if (norm(p - obs) < r+0.1)
        opt_flag = true;
        
    elseif ( norm(p_curr - obs) > r + 0.3) & ( opt_flag == true)
          fprintf( '%s\n', "Fine optimizzaz")
          opt_flag = false;
    end
    
    if (opt_flag == true)
            dis = @(x) norm(x-p);
                if pj_opt(i-1, :) == [0,0, 0]
                    p0 = pj(i, :);
                else
                    p0 = pj_opt(i-1, :);
                end

            y = fmincon(dis, p0, [], [], [], [],[],[],  @nonlin)
            pj(i+1, :)
            pj_opt(i+1, :) = y;
            pj(i+1, :) = y;
    end 
end


pjt = pj;
end
