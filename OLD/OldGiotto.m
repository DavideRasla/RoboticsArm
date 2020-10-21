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

%% Finding the boundaries and rescaling manually the traj

hold on;
[trajectories, B,L,N, A]  = GettingTrajectories(binaryImage);


%% Creation of a single trajectory
figure
%filtering the points in order to reduce the #
trajectories = Filtering(trajectories);

%% Mstraj for each traj


q_def = [];


q_def = [q_def; qz];

hold on;
 for k=1:length(trajectories)
    InitialPath = trajectories{k}';
    ExtendedPath = [InitialPath; zeros(1,numcols(InitialPath));];
    ExtendedPath(3,:) = ExtendedPath(3,:) -2;
    traj = mstraj(ExtendedPath(:,2:end)',[0.8 0.8 0.8], [], ExtendedPath(:,1)', 0.3, 0.2);
    Tp =   SE3(traj) * SE3.oa([0 1 0], [0, 0, -1]);
    q_traj = rob.ikine6s(Tp);
    m = rob.maniplty(q_traj);
    m;
    q_def = [q_def; q_traj]; 
 end
q_def = [q_def; qz]; 


%% plotting the result (This can take a while...)

   
rob.plot(q_def, 'xyz','noraise', 'trail', {'r.', 'LineWidth', 2});




















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
rgbImage = imread('Images/Unipi.png');

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
            for j = 1:10:length(trajectories{i})% 7 it's the step size. A large step size  it is less precise. Anyway 7 is a good compromise
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


