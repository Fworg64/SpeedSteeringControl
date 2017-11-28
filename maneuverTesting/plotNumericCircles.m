%plot numeric circle

function [] = plotNumericCircles(r,wpx, wpy, wpth)
    hold on;
    angle = 0:.1:2*pi+.1;
    waypoint = [wpx,wpy,wpth];

    fieldlength = 7.38;
fieldwidth = 3.78;
startinglength = 1.5;
obstaclelength = 2.94;
cla
hold on;
plot([startinglength, startinglength,0],[-fieldwidth/2,0,0]);
plot([startinglength, startinglength,0],[fieldwidth/2,0,0]);
plot([startinglength + obstaclelength,startinglength + obstaclelength],[-fieldwidth/2,fieldwidth/2]);


plot([0,.3],[0,0],'rd');
plot([waypoint(1),waypoint(1)+.3*cos(waypoint(3))],[waypoint(2),waypoint(2)+.3*sin(waypoint(3))],'gd');
    circle1x = r*cos(angle);
    circle1y = r*sin(angle) + r;
    plot(circle1x,circle1y);
    circle2x = r*cos(angle) - r*cos(-wpth - pi/2) + wpx ;%+ r;
    circle2y = r*sin(angle) - r*sin(-wpth + pi/2) + wpy ;%- r;
    plot(circle2x, circle2y);
 
    title(sprintf('R = %.4f', r));

end