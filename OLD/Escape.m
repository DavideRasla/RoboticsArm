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

%% Getting Image


format long g;
format compact;
fontSize = 36;
rgbImage = imread('Image2.png');
figure
%imshow('image4.png')
%[xi,yi] = getpts



% Get the dimensions of the image.  numberOfColorBands should be = 3.
[rows, columns, numberOfColorBands] = size(rgbImage);
% Display the original color image.
%subplot(2, 2, 1);
%imshow(rgbImage);

% Enlarge figure to full screen.
%set(gcf, 'Units', 'Normalized', 'Outerposition', [0, 0, 1, 1]);
% Extract the individual red, green, and blue color channels.
% redChannel = rgbImage(:, :, 1);
greenChannel = rgbImage(:, :, 2);
% blueChannel = rgbImage(:, :, 3);
% Get the binaryImagepathAll = [path; zeros(1,numcols(path))];
binaryImage = greenChannel < 200;
% Display the original color image.
%subplot(2, 2, 2);
%imshow(binaryImage);

% Find the baseline
verticalProfile  = sum(binaryImage, 2);
lastLine = find(verticalProfile, 1, 'last')
% Scan across columns finding where the top of the hump is
for col = 1 : columns
  yy = lastLine - find(binaryImage(:, col), 1, 'first');
  if isempty(yy)
    y(col) = 0;
  else
    y(col) = yy;
  end
end
%subplot(2, 2, 3);
%plot(1 : columns, y, 'b-', 'LineWidth', 3);

hold on; [B,L,N] = bwboundaries(binaryImage,'noholes');%use 'noholes' to simplify

trajectories = cell(0);
    for k=1:length(B),
    	boundary = B{k};
        xy = [B{k}(:,1), B{k}(:,2)];
        trajectories = [trajectories; xy]
    	%plot(boundary(:,2), boundary(:,1),'.r')
    end

xy = [B{1}(:,1), B{1}(:,2)]; % 2 1-D vectors.

    for k=1:length(trajectories),
        trajectories{k} = rescale(trajectories{k})
    end

%xy = [columns, rows]; % Stitch together to form N by 2 (x,y) array

%plot(xy(:,1), xy(:,2))




%% Creating a map

%map = LandmarkMap(10, 10);

%load map1
%goal = [5,3];
%start=[2,1];
%ds = Dstar(map);
%ds.plan(goal)%add animate
%path = ds.query(start)
%path = path';
%%%path = xy';
q_def = [];
 for k=1:length(trajectories)
    path = trajectories{k}';
    pathAll = [path; zeros(1,numcols(path))];
    path1=pathAll;
    %path1 = ceil(path1);
    %path1 = unique(path1.','rows')
    %path1 = path1';
    traj1 = mstraj(path1(:,2:end)',[0.8 0.8 0.8], [], path1(:,1)', 2, 0.4);

    Tp1 =  SE3(0, 0.6, -0.4) * SE3(traj1) * SE3.oa( [0 1 0], [0 0 -1]);

    q_traj = rob.ikine6s(Tp1);
    q_def = [q_def; q_traj];
    
 end
rob.plot(q_def, 'trail', {'r.', 'LineWidth', 2});
%rob.plot(q_traj, 'trail', {'r.', 'LineWidth', 2});

%rob.plot(q_traj);
%{
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
%}
