%twoTurnPlotter

function [] = twoTurnPlotter(xi, yi, thi, wpx, wpy, wpth)
%lazy renaming of variables to appease copy pasta
initialPose = [xi, yi, thi];
waypoint = [wpx, wpy, wpth];
beenflipped = false;

% if ((tan(thi)*(wpx - xi) + yi > wpy) && abs(thi) < pi/2  || ...)% && angleDiff(thi, wpth) > 0) %waypoint is below robot (negative y)
%      (tan(thi)*(wpx - xi) + yi < wpy) && abs(thi) >=pi/2 - .1) %need to handle changing definition of above for rotating robot
%     %need to put waypoint above robot for calculations and then invert the
%     %maneuvers radii?
%     
if (abs(thi) <= pi/2)
    if (tan(thi)*(wpx - xi) + yi > wpy)
        [wpx, wpy, wpth] = reflectWaypointAroundRobot(wpx, wpy, wpth, xi, yi ,thi);
        disp('flip1');
        beenflipped = true;
    end
else %want to make sure angle is either > pi/2 or <=, nothing else
    if (tan(thi)*(wpx - xi) + yi < wpy)
        [wpx, wpy, wpth] = reflectWaypointAroundRobot(wpx, wpy, wpth, xi, yi ,thi);
        disp('flip1');
        beenflipped = true;
    end
end

cla;

[distance1, radius1, xc1, yc1, distance2, radius2, xc2, yc2] = twoTurnSolver(xi, yi, thi, wpx, wpy, wpth)
if (beenflipped)
    disp('flip2');

    radius1 = -radius1
    radius2 = -radius2
    [wpx, wpy, wpth] = reflectWaypointAroundRobot(wpx, wpy, wpth, xi, yi ,thi);
    [xc1, yc1] = reflectWaypointAroundRobot(xc1, yc1, 0, xi, yi, thi)
    [xc2, yc2] = reflectWaypointAroundRobot(xc2, yc2, 0, xi, yi, thi)

end

maneuverPlot(xi, yi, thi, distance1, radius1, xc1, yc1);

[interX, interY, interTh] = maneuverEndFinder(initialPose(1),initialPose(2),initialPose(3),distance1, radius1, xc1, yc1)

maneuverPlot(interX, interY, interTh, distance2, radius2, xc2, yc2);

hold on;
plot(wpx+.25*cos(wpth),wpy+.25*sin(wpth), 'rd');
plot(wpx-.25*cos(wpth),wpy-.25*sin(wpth), 'bx');
hold off;

title(sprintf('From [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]',initialPose(1),initialPose(2),initialPose(3),  waypoint(1),waypoint(2), waypoint(3)));

end
