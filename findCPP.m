%%find Closest Path Point

function [x,y,theta] = findCPP(robotX, robotY, CenterX, CenterY, Radius)
  warning('findCPP has been depreciated by newFindCPP, please use it instead and note the argument order is different');
  %%nearest point on path is point on path that is on the line through the center of the circle and the robots pose that is closest to the robots pose.
  %%let theta be the angle the robot should be at this point on the path
  %%%calculating path compensation term
  %%care! need to be careful about which solution is grabbed, dont want the other side of the circle
  %%this could all be done in world coordinates, which means waypointsolver center needs to be transformed through robots initial pose to the world
  %thisLineEq: y - center1(2) = (center1(2) - robotState(2))/(center1(1) - robotState(1)) * (x - center1(1));
  %pathEq: Radius^2 = (x - center1(1))^2 + (y - center1(2))^2;
  %will be the solution that is closest to robot pose - this is the care!
  center = [CenterX, CenterY];
  robotPhysicalPose = [robotX, robotY];
    if (center(1) - robotPhysicalPose(1) ==0)
      %M = (center(2) - robotPhysicalPose(2))/.001;
      %X is the same and then its just the closest y Coord (Y+R or Y-R)
      Xp = center(1);
      Ypa = center(2)+Radius;
      Ypb = center(2)-Radius;
      if (abs(Ypa - robotPhysicalPose(2)) <abs(Ypb -robotPhysicalPose(2)))
          Yp = Ypa;
      else
          Yp = Ypb;
      end
    else
        M = (center(2) - robotPhysicalPose(2))/(center(1) - robotPhysicalPose(1));
        potXpa = (-(-2*center(1) - 2*M^2*center(1)) + sqrt((-2*center(1) - 2*M^2*center(1))^2 - 4*(1+M^2)*(center(1)^2 - Radius^2 + M^2*center(1)^2)))/(2*(1 + M^2));
        potXpb = (-(-2*center(1) - 2*M^2*center(1)) - sqrt((-2*center(1) - 2*M^2*center(1))^2 - 4*(1+M^2)*(center(1)^2 - Radius^2 + M^2*center(1)^2)))/(2*(1 + M^2));
        if (abs(robotPhysicalPose(1) - potXpa) < abs(robotPhysicalPose(1) - potXpb))
         Xp = potXpa;
        else
         Xp = potXpb;
        end
        Yp = M*(Xp - center(1)) + center(2);
    end
    
    x = Xp;
    y = Yp;
    
    theta = angleDiff(atan2(Yp - center(2), Xp - center(1)), pi/2);
end

