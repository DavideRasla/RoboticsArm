function [q0dot_optimum] = optCollisionAvoidance(robot, Ve, J, Jc, q_old, Tnew, pointsCloud, Goal)  
    x0 = zeros(robot.n, 1);
    options = optimoptions('fmincon','Display','iter',...
         'Algorithm', 'active-set', 'FunctionTolerance', 1e-3);
    q0dot_optimum = fmincon(@(q0dot)objFunCA(q0dot, robot,q_old, Ve, J, Jc, pointsCloud, Goal),...
        x0,[],[],[],[],[],[],@(q0dot)nonLinearConstr(q0dot, robot, Ve, J, Jc, q_old, Tnew), options);
end