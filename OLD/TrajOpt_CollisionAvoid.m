function [qtraj, traj, ee_traj] = TrajOpt_CollisionAvoid(robot,qi,obs)  

    dampingFactor = 0.03;
    qtraj = qi;

    for i = 2:1:size(qtraj,3)
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
        
        dist = (robot, qNew, obs);
        if(min(dist) < 0.25)
            q0dot = optCollisionAvoidance(robot, Ve, J, Jc, qOld, Tnew, pointsCloud,);
            qDot = Jc*Ve + (eye(size(Jc*J))-Jc*J)*q0dot;
            qNew = qOld + qDot';
        end
        qtraj=cat(1,qtraj,qNew);
        
        eeT = robot.fkine(qNew);
        ee_traj= cat(3,ee_traj,eeT);

    end 
end