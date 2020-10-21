function [multi_qtraj,trajPlot] = multi_trajectory(robot,qi,Ti,segments,Goal,pointsCloud)  
    mqtraj = [];
    ee_traj = [];
    trajPlot = {};
    for i = 1:1:size(segments,3) 
        if i ~= 1 
            q = mqtraj(numrows(mqtraj),:);
            T = ee_traj(:,:,end); 
        else 
            q = qi;
            T = Ti;
        end
        Tf = segments(:,:,i);
        [qT,~,eeT] = TrajOpt_CollisionAvoid(robot,q,T,Tf,pointsCloud,Goal{i});
        if(strcmp(Goal{i}, 'PourDown') == 1)
            qT = cat(1, qT, repmat(qT(end,:),15,1));
            eeT = cat(3, eeT, repmat(eeT(:,:,end),1,1,15));
        end
        trajPlot = cat(1, trajPlot, {qT,Goal{i},eeT});
        mqtraj = cat(1,mqtraj,qT);
        ee_traj= cat(3,ee_traj,eeT);
    end
    multi_qtraj = mqtraj;
end
