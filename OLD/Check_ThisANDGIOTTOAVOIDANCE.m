
TrajOpt_CollisionAvoid(robot,q,T,Tf,pointsCloud,Goal{i});
function [qtraj, traj, ee_traj] = TrajOpt_CollisionAvoid(robot,qi,Ti,Tf,pointsCloud, Goal)  
    [~, ~,traj] = PathCollisionAvoidance(Ti, Tf, Goal, pointsCloud);
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
        
        dist = (robot, qNew, pointsCloud, Goal);
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
   
q0dot = optCollisionAvoidance(robot, Ve, J, Jc, qOld, Tnew, pointsCloud, Goal);

function [q0dot_optimum] = optCollisionAvoidance(robot, Ve, J, Jc, q_old, Tnew, pointsCloud, Goal)  
    x0 = zeros(robot.n, 1);
    options = optimoptions('fmincon','Display','iter',...
         'Algorithm', 'active-set', 'FunctionTolerance', 1e-3);
    q0dot_optimum = fmincon(@(q0dot)objFunCA(q0dot, robot,q_old, Ve, J, Jc, pointsCloud, Goal),...
        x0,[],[],[],[],[],[],@(q0dot)nonLinearConstr(q0dot, robot, Ve, J, Jc, q_old, Tnew), options);
end

function val = objFunCA(q0dot,robot, q_old, Ve, J, Jc, pointsCloud, Goal)
    qdot = Jc*Ve + (eye(size(Jc*J))-Jc*J)*q0dot;
    q_new = q_old + qdot'; % new q in function of q0dot 
	% compute for each joint the minimum distance from obstacles
    functional = minDistancesObstacles(robot, q_new, pointsCloud, Goal); 
    % max of minimum distance joint-obstacle
	val = -min(functional);
end
