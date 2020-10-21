function [pointsJoints] = getPositionJoints(robot, q)
    base = robot.base;
    posJoints = [];
    Tjoints = {};
    hold on; 
   for i=1:1:robot.n
        T = base;
        for j = 1:1:i-1
            T = T*robot.links(j).A(q(1,j));
        end
        Tjoints{i,1} = T;
        positionJoints(i,:) = transl(T);
        if robot.links(i).isprismatic 
            c = cylinder(Tjoints{i,1}, [0.1 0.1 q(1,i)]);
            R = t2r(Tjoints{i,1});
        else
            if(robot.links(i).a ~= 0)
                Tc = Tjoints{i,1}*(rt2tr(rpy2r([pi/2 pi/2+q(1,i) 0]), [0 0 0]'));
                R = t2r(Tc);
                c = cylinder(Tc, [0.1 0.1 robot.links(i).a]);
            else 
                c = cylinder(Tjoints{i,1}, [0.1 0.1 robot.links(i).d]); 
                R = t2r(Tjoints{i,1});
            end
        end
    
        %plot(c);
        h = c.scale(1,3);
        centre = positionJoints(i,:);
        if(h<0)
            centre = centre +(R*[0 0 h]')';
        end
        pointsJoints{i,:} = pointCylinder(centre, 0.1, h, R);
    end   
    %robot.plot(q);
end