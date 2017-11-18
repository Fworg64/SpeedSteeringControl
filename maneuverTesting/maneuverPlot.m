%maneuverplot
%this function plots a mauever of the form distance, radius, xc, yc and the
%path it would take from a pose given by x, y ,th

function [] = maneuverPlot(x, y, theta, distance, radius, xc, yc)
fieldlength = 7.38;
fieldwidth = 3.78;
startinglength = 1.5;
obstaclelength = 2.94;
%clf
hold on;
plot([startinglength, startinglength,0],[-fieldwidth/2,0,0]);
plot([startinglength, startinglength,0],[fieldwidth/2,0,0]);
plot([startinglength + obstaclelength,startinglength + obstaclelength],[-fieldwidth/2,fieldwidth/2]);



%plot initial robot
plot([x-.25*cos(theta)],[y-.25*sin(theta)], 'ro');
plot([x+.25*cos(theta)],[y+.25*sin(theta)], 'rd');
%get end robot
[xe, ye, the] = maneuverEndFinder(x, y, theta, distance, radius,xc, yc);
%plot end robot
plot([xe-.25*cos(the)],[ye-.25*sin(the)], 'go');
plot([xe+.25*cos(the)],[ye+.25*sin(the)], 'gd');
%get intermediate robot
dD = distance/20;
xPoints = zeros(1,20);
yPoints = zeros(1,20);
for k = 1:20
    [xPoints(1,k),yPoints(1,k)] = maneuverEndFinder(x, y, theta, dD*k, radius, xc, yc);
end
%plot intermediate points
plot(xPoints(1,:), yPoints(1,:),'r--');

%plot center of turn as well
plot(xc, yc, 'm*');

xlim([0,fieldlength]);
ylim([-fieldwidth/2,fieldwidth/2]);
pbaspect([fieldlength,fieldwidth,1]);
hold off;

end