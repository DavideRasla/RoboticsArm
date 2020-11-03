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