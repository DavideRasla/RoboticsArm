%% Path End Effector computed by Artificial Potential Field Algorithm 
function [Points_Fit, Points_AP, PathEE] = PathCollisionAvoidance(Tini, TGoal, Goal, pointsCloud)
    stop = 0;
    n_obstacles = size(pointsCloud,1);
    hold on;
    d0 = 0.25;
    Points_AP(:,:,1) = transl(Tini);
    AttrFactor = 0.05;
    RepFactor = 0.001;
    while (norm(Points_AP(:,:,end) - transl(TGoal)) > 0.01 && stop == 0)
        F = zeros(3,1);
        dgoal = transl(TGoal)- Points_AP(:,:,end); % distance EE - Goal    
        if(norm(dgoal) < d0)
            Fatt = dgoal*AttrFactor;  % quadratic
        else
            Fatt = dgoal*AttrFactor/norm(dgoal); % conical
        end
        F = Fatt;
        for i = 1:1:n_obstacles
           if(strcmp(pointsCloud{i}{1,1}, Goal)==0 ||...
               strcmp(pointsCloud{i}{1,1}, strcat(Goal, '_u'))==0 ||...
                strcmp(pointsCloud{k}{1,1}, 'GlassFinal'))
               [dObs,dist] = distEEobstacles(Points_AP(:,:,end),  pointsCloud{i});
               if(dist < d0)
                  Frep = RepFactor*((1/dist-1/d0)*1/dist^2)*(dObs/dist);
               else
                  Frep = zeros(3,1);
               end
           end
           F = F + Frep;
        end
        if(norm(F) > 0.1)
            F = 0.1*F/norm(F);
        end
        newstep = Points_AP(:,:,end) + F;
        if(norm(newstep - Points_AP(:,:,end)) > 0.01)
            Points_AP = cat(3, Points_AP, newstep);
        else
            stop = 1;
        end
    end
    Points_AP = cat(3, Points_AP, transl(TGoal)); 
    
    if(size(Points_AP,3) == 2)  %% if EE position init and final are equal
        PathEE = ctraj(Tini,TGoal,20);
        Points_Fit = [];
    else
        data=[];
        for i=1:1:size(Points_AP,3)
            data = cat(2, data,Points_AP(:,:,i));
        end
        data = data';
        Points_Fit = fitData(data);
        
        PathEE = ctraj(Tini,TGoal,size(Points_Fit,2));
        plotData(PathEE, data, Points_Fit);
        for i =1:1:size(PathEE,3)
            PathEE(1:3,4,i) = Points_Fit(:,i);
            if strcmp(Goal, 'PourDown') == 0 
               PathEE(1:3,2,i) = [0;0;1];
            end
        end 
    end
end
