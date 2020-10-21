

function trajectories = Filtering(trajectories)
    V = [];
    for i = 1:length(trajectories)
         if length(trajectories{i}) > 3500 
            for j = 1:30:length(trajectories{i})
              V = [V; trajectories{i}(j,:)];   
            end
            trajectories{i} = V;    
         elseif length(trajectories{i}) > 1500 
            for j = 1:5:length(trajectories{i})
              V = [V; trajectories{i}(j,:)];   
            end
            trajectories{i} = V;    
         elseif length(trajectories{i}) > 400 
            for j = 1:3:length(trajectories{i})
              V = [V; trajectories{i}(j,:)];   
            end
            trajectories{i} = V;    
        end
        V = [];
    end
end