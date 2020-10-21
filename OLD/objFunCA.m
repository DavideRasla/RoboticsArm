function val = objFunCA(q0dot,robot, q_old, Ve, J, Jc, pointsCloud, Goal)
    qdot = Jc*Ve + (eye(size(Jc*J))-Jc*J)*q0dot;
    q_new = q_old + qdot'; % new q in function of q0dot 
	% compute for each joint the minimum distance from obstacles
    functional = minDistancesObstacles(robot, q_new, pointsCloud, Goal); 
    % max of minimum distance joint-obstacle
	val = -min(functional);
end