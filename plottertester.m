%waypointtesterv2
x = 1;
y = 2;
theta = 4*pi/6;

clf;
[firstlinear, firstdistance, firstradius, seconddistance, secondradius, secondlinear, xcenter, ycenter, xcenter2, ycenter2, mode] = waypoint2setpointsv4SAMOct25(x, y, theta)

if (mode ==1) %straight then turn
    hold on;
    %plot straight line
    plot([0,firstlinear],[0,0]);
    pathangles = linspace(0,firstdistance/firstradius,50);
    
    plot([firstlinear + firstradius*cos(pathangles -pi/2)], [firstradius*sin(pathangles -pi/2) + firstradius]);
end
if (mode ==2) %double arc
    hold on;
    pathangles = linspace(0,firstdistance/firstradius,50);
    plot([firstradius*cos(pathangles - pi/2)], [firstradius*sin(pathangles - pi/2) + firstradius]);
    pathangles2 = linspace(0,seconddistance/secondradius,50);
    plot([secondradius*cos(pathangles2 - pi/2 + pathangles(50)) + firstradius*cos(pathangles(50) - pi/2)], ...
         [secondradius*sin(pathangles2 - pi/2 + pathangles(50)) + firstradius*sin(pathangles(50) - pi/2) + firstradius]);
    legend('First', 'Second');
end
if (mode ==3) %double arc with line
    hold on;
    plot([0,firstlinear],[0,0]);
    pathangles = linspace(0,firstdistance/firstradius,50);
    plot([firstradius*cos(pathangles - pi/2) + firstlinear], [firstradius*sin(pathangles - pi/2) + firstradius]);
    pathangles2 = linspace(0,seconddistance/secondradius,50);
    plot([secondradius*cos(pathangles2 - pi/2 + pathangles(50)) + firstradius*cos(pathangles(50) - pi/2) + firstlinear], ...
         [secondradius*sin(pathangles2 - pi/2 + pathangles(50)) + firstradius*sin(pathangles(50) - pi/2) + firstradius]);
    legend('First', 'Second');
end