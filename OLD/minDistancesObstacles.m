function [W] = minDistancesObstacles(robot, q, obs)
    pointsJoints = getPositionJoints(robot, q);
    W = zeros(robot.n,1);
    for i = 1:1:robot.n %points of each joint
       distances = [];
       for j= 1:1:size(pointsJoints{i}) %points of each joint

                       d = norm(pointsJoints{i}(j,:) - obs);
                       distances = [distances;d];
       end
       W(i,1) = min(distances);
    end
end