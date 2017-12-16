%twoTurnPlotter

function [] = twoTurnPlotter(xi, yi, thi, wpx, wpy, wpth)
%lazy renaming of variables to appease copy pasta
initialPose = [xi, yi, thi];
waypoint = [wpx, wpy, wpth];

% if (tan(thi)*(wpx - xi) + yi > wpy) %waypoint is below robot (negative y)
%     %need to put waypoint above robot for calculations and then invert the
%     %maneuvers radii
%     xtemp = (tan(thi) * xi - yi - 1/tan(thi) * wpx + wpy)/(tan(thi) - 1/tan(thi));
%     ytemp = tan(thi)*(xtemp - xi) +yi;
%     %temp is the point on the robots line that is closest to line
%     %perpindicular to waypoint
%     %reflected point is on other side
%     delx = wpx - xi;
%     dely = wpy - yi;
%     newwpx = xtemp - delx;
%     newwpy = ytemp - dely;
%     newth = 2*thi - wpth;
%     
%     waypoint= [newwpx, newwpy, newth];
% end

cla;

[distance1, radius1, xc1, yc1, distance2, radius2, xc2, yc2] = twoTurnSolver(xi, yi, thi, wpx, wpy, wpth)
% if (tan(thi)*(wpx - xi) + yi > wpy)
%     radius1 = -radius1
%     radius2 = -radius2
% end

maneuverPlot(xi, yi, thi, distance1, radius1, xc1, yc1);

[interX, interY, interTh] = maneuverEndFinder(initialPose(1),initialPose(2),initialPose(3),distance1, radius1, xc1, yc1)

maneuverPlot(interX, interY, interTh, distance2, radius2, xc2, yc2);

hold on;
plot(wpx+.25*cos(wpth),wpy+.25*sin(wpth), 'rd');
plot(wpx-.25*cos(wpth),wpy-.25*sin(wpth), 'bx');
hold off;

title(sprintf('From [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]',initialPose(1),initialPose(2),initialPose(3),  waypoint(1),waypoint(2), waypoint(3)));

end