%newfindcpp test

function [] = newFindCppTestPlot(xc, yc, radius, rx, ry, rth)

fieldlength = 7.38;
fieldwidth = 3.78;
startinglength = 1.5;
obstaclelength = 2.94;
clf
hold on;
plot([startinglength, startinglength,0],[-fieldwidth/2,0,0]);
plot([startinglength, startinglength,0],[fieldwidth/2,0,0]);
plot([startinglength + obstaclelength,startinglength + obstaclelength],[-fieldwidth/2,fieldwidth/2]);



%plot robot
plot([rx-.25*cos(rth)],[ry-.25*sin(rth)], 'ro');
plot([rx+.25*cos(rth)],[ry+.25*sin(rth)], 'rd');

%plot manuever
xPoints = zeros(1,20);
yPoints = zeros(1,20);
for k = 1:20
    xPoints(k) = xc + radius * cos(2*pi / 20 * k);
    yPoints(k) = yc + radius * sin(2*pi /20 * k);
end
%plot intermediate points
plot(xPoints(1,:), yPoints(1,:),'r--');

%plot center of turn as well
plot(xc, yc, 'm*');

%plot cpp
[x,y, theta] = newFindCPP(xc, yc, radius, rx, ry, rth)
plot([x-.25*cos(theta)],[y-.25*sin(theta)], 'bo');
plot([x+.25*cos(theta)],[y+.25*sin(theta)], 'bd');

xlim([0,fieldlength]);
ylim([-fieldwidth/2,fieldwidth/2]);
pbaspect([fieldlength,fieldwidth,1]);
hold off;


end