%manueverEndFinder
%finds the end of a maneuver given the start pose and the maneuver
%parameters

function [x, y, th] = maneuverEndFinder(xi, yi, thi, distance, radius, xc, yc)
NAD = distance/radius; %net angluar distance travelled through maneuver

UTx = radius*cos(NAD - pi/2);
UTy = radius*sin(NAD - pi/2) + radius;

x = cos(thi) * UTx - sin(thi) * UTy + xi;
y = sin(thi) * UTx + cos(thi) * UTy + yi;
th = thi + NAD;

end