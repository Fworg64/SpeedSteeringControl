%oneTurnPlotter
%matlab is a piece of shit


function [] = oneTurnPlotter(xi, yi, thi, wpx, wpy, wpth)
initialPose = [xi, yi, thi];
waypoint = [wpx, wpy, wpth];

cla;

[distance1, radius1, xc1, yc1, distance2, radius2, xc2, yc2] = oneTurnSolver(initialPose(1),initialPose(2),initialPose(3), waypoint(1),waypoint(2), waypoint(3) )

maneuverPlot(initialPose(1),initialPose(2),initialPose(3),distance1, radius1, xc1, yc1);

[interX, interY, interTh] = maneuverEndFinder(initialPose(1),initialPose(2),initialPose(3),distance1, radius1, xc1, yc1)

maneuverPlot(interX, interY, interTh, distance2, radius2, xc2, yc2);

hold on;
plot([wpx-.25*cos(wpth),wpx+.25*cos(wpth)],[wpy-.25*sin(wpth),wpy+.25*sin(wpth)], 'bx');
hold off;

title(sprintf('From [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]',initialPose(1),initialPose(2),initialPose(3),  waypoint(1),waypoint(2), waypoint(3)));

end