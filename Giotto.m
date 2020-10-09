clear all; clc;
%% Robot

L1 = Link('d', 0,  'a', 0,   'alpha', pi/2);
L2 = Link('d', 0,  'a', 2,   'alpha', 0);
L3 = Link('d', 0,  'a', 2,   'alpha', -pi/2);
L4 = Link('d', 0,  'a', 0,   'alpha', -pi/2);
L5 = Link('d', 0,  'a', 0,   'alpha', pi/2, 'offset', pi/2);
L6 = Link('d', 0,  'a', 0,   'alpha', 0);
qz = [0 0 0 0 0 0];
L = [L1 L2 L3 L4 L5 L6];
rob = SerialLink(L , 'name', 'Giotto');

%% Getting the Image
binaryImage = CreatingBinary();

imshow(binaryImage);

%% Finding the boundarie

hold on;
[trajectories, B,L,N, A]  = GettingTrajectories(binaryImage);


%% Creation of a single trajectory

%filtering the points in order to reduce the #
trajectories = Filtering(trajectories);

%% Mstraj

%q_def = cell(0)
q_def = [];

 for k=1:length(trajectories)
    path = trajectories{k}';
    pathAll = [path; zeros(1,numcols(path))];
    path1=pathAll;
    traj1 = mstraj(path1(:,2:end)',[0.8 0.8 0.8], [], path1(:,1)', 0.3, 0.2);
    Tp1 =  SE3(-2, -2.5, -0.8) * SE3(traj1) * SE3.oa( [0 1 0], [0 0 -1]);
    q_traj = rob.ikine6s(Tp1);
    q_def = [q_def; q_traj]; 
 end

q_def = [q_def; qz];

%% plotting the result (This can take a while...)
hold on;   
figure
   
rob.plot(q_def, 'xyz','noraise', 'trail', {'r.', 'LineWidth', 2});

for i = 1:length(q_def)
     
    %rob.plot(q_def{i}, 'wrist', 'trail', {'r.', 'LineWidth', 2});
end

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
rgbImage = imread('Images/Bart.png');

% Get the dimensions of the image.  numberOfColorBands should be = 3.
[rows, columns, numberOfColorBands] = size(rgbImage);

%imshow(rgbImage);
greenChannel = rgbImage(:, :, 2);

% Get the binaryImagepathAll = [path; zeros(1,numcols(path))];
binaryImage = greenChannel < 200;


%imshow(binaryImage);


end

function[trajectories, B,L,N, A] =  GettingTrajectories(binaryImage)


[B,L,N, A] = bwboundaries(binaryImage,4,'holes');%use 'noholes'
                                                 %to simplify
trajectories = cell(0);

    for k =1:length(B)
       boundary = B{k};
      % plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2);
      if length(B{k}) > 10 % Not empty traj
           xy = [B{k}(:,2), B{k}(:,1)];
           trajectories = [trajectories; xy]
          plot(B{k}(:,2), B{k}(:,1), 'r', 'LineWidth', 2);
      end

    end

%rescaling into  [0:3]
figure
hold on

    for k=1:length(trajectories)
        trajectories{k} = rescale(trajectories{k}, 0,3);
        plot( trajectories{k}(:,2),  trajectories{k}(:,1), 'g', 'LineWidth', 2);
       
    end
end


function trajectories = Filtering(trajectories)
    V = [];
    for i = 1:length(trajectories)
         if length(trajectories{i}) > 3500 %only if necessary
            for j = 1:20:length(trajectories{i})% 7 it's the step size. A large step size  it is less precise. Anyway 7 is a good compromise
              V = [V; trajectories{i}(j,:)];   
            end
            trajectories{i} = V;    
         elseif length(trajectories{i}) > 1500 %only if necessary
            for j = 1:7:length(trajectories{i})% 7 it's the step size. A large step size  it is less precise. Anyway 7 is a good compromise
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
