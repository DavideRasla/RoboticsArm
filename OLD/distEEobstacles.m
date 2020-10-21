function [diff, dist] = distEEobstacles(EEpos, pointsObs)
   dist = 100;
   for x = 1:1:size(pointsObs{1,2})
       d = norm(EEpos - pointsObs{1,2}(x,:)');
       if(min(d, dist) == d)  
           dist = d;
           diff = EEpos - pointsObs{1,2}(x,:)';
       end
   end
end