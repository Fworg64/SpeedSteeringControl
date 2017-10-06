%%waypoint2setpointtester
%this script runs the waypoint2setpoints script through a few tests to validate the output.
function waypoint2setpointtester

  figure();
  fieldlength = 5;
  fieldwidth = 4;


  global waypointRobotCoord = [2,1,pi/3];
  %works for intersection closer to start, circle as starting part, first quadrant?
  %implies intersection distance to start < intersection distance to end

  [Distance, Radius, Distance2, Radius2, xcenterUT, ycenterUT, xcenter2UT, ycenter2UT] = waypoint2setpointsv3(waypointRobotCoord(1), waypointRobotCoord(2), waypointRobotCoord(3));

  angle1 = Distance/Radius;

  angle1space = linspace(0,angle1,40);
  jaunt1plotx = Radius * cos(angle1space - pi/2);
  jaunt1ploty = Radius * sin(angle1space - pi/2) + Radius;
  plot(jaunt1plotx, jaunt1ploty, 'b');
  xlim([0,fieldlength]);
  ylim([-fieldwidth/2,fieldwidth/2]);
  axes('position', [.1,.3,.8,.6]);
  pbaspect([fieldlength,fieldwidth,1]);
  hold on;

  xintermediate = Radius*cos(angle1 - pi/2);
  yintermediate = Radius*sin(angle1 - pi/2) + Radius;

  angle2 = Distance2/Radius2;

  angle2space = linspace(0,angle2,40);
  jaunt2plotx = Radius2 * cos(angle2space -pi/2);
  jaunt2ploty = Radius2 * sin(angle2space -pi/2) + Radius2;
  %plot(jaunt2plotx, jaunt2ploty, 'r');

  xmore = Radius2*cos(angle2 - pi/2);
  ymore = Radius2*sin(angle2 - pi/2) + Radius2;

  %transform xmore to coordinates with origin at intermediate position
  %translate to intermediate point, rotate to angle1

  transformMat = [cos(angle1), -sin(angle1), 0; %%ccw rotation of 'more' vector by angle1
                  sin(angle1), cos(angle1), 0;
                  0,0,1];
               
  moreRot = transformMat * [xmore;ymore;1];

  transformed2jaunt = transformMat * [jaunt2plotx;jaunt2ploty;ones(1,40)];
  transformed2jaunt(1:2,:) = transformed2jaunt(1:2,:) + [xintermediate;yintermediate];
  plot(transformed2jaunt(1,:), transformed2jaunt(2,:), 'r');

  xFinal = moreRot(1) + xintermediate;
  yFinal = moreRot(2) + yintermediate;
  angleFinal = angle2 + angle1;

  plot([waypointRobotCoord(1), .1*cos(waypointRobotCoord(3)) + waypointRobotCoord(1)], [waypointRobotCoord(2), .1*sin(waypointRobotCoord(3)) + waypointRobotCoord(2)], 'g*');

  %add slider
  hsliderx = uicontrol (                    ...
         'style', 'slider',                ...
         'Units', 'normalized',            ...
         'position', [0.1, 0.1, 0.8, 0.1], ...
         'min', .01,                         ...
         'max', 5,                        ...
         'value', 2,                      ...
         'callback', {@xplotstuff}          ...
       );
  hslidery = uicontrol (                    ...
         'style', 'slider',                ...
         'Units', 'normalized',            ...
         'position', [0.1, 0.2, 0.8, 0.1], ...
         'min', -2,                         ...
         'max', 2,                        ...
         'value', 1.5,                      ...
         'callback', {@yplotstuff}          ...
       );
  hsliderth = uicontrol (                    ...
         'style', 'slider',                ...
         'Units', 'normalized',            ...
         'position', [0.1, 0.3, 0.8, 0.1], ...
         'min', -pi,                         ...
         'max', pi,                        ...
         'value', pi/4,                      ...
         'callback', {@thplotstuff}          ...
       );
  
end

function xplotstuff(h,event)
  
  newx = get(h, 'value');
  global waypointRobotCoord;
  fieldlength = 5;
  fieldwidth = 4;
  waypointRobotCoord(1) = newx;
  [Distance, Radius, Distance2, Radius2, xcenterUT, ycenterUT, xcenter2UT, ycenter2UT] = waypoint2setpointsv3(waypointRobotCoord(1), waypointRobotCoord(2), waypointRobotCoord(3));

  angle1 = Distance/Radius;

  angle1space = linspace(0,angle1,40);
  jaunt1plotx = Radius * cos(angle1space - pi/2);
  jaunt1ploty = Radius * sin(angle1space - pi/2) + Radius;
  plot(jaunt1plotx, jaunt1ploty, 'b');
  xlim([0,fieldlength]);
  ylim([-fieldwidth/2,fieldwidth/2]);
  %axes('xposition', [.1,.3,.8,.6]);
  pbaspect([fieldlength,fieldwidth,1]);
  hold on;
  xintermediate = Radius*cos(angle1 - pi/2);
  yintermediate = Radius*sin(angle1 - pi/2) + Radius;

  angle2 = Distance2/Radius2;

  angle2space = linspace(0,angle2,40);
  jaunt2plotx = Radius2 * cos(angle2space -pi/2);
  jaunt2ploty = Radius2 * sin(angle2space -pi/2) + Radius2;
  %plot(jaunt2plotx, jaunt2ploty, 'r');

  xmore = Radius2*cos(angle2 - pi/2);
  ymore = Radius2*sin(angle2 - pi/2) + Radius2;

  %transform xmore to coordinates with origin at intermediate position
  %translate to intermediate point, rotate to angle1

  transformMat = [cos(angle1), -sin(angle1), 0; %%ccw rotation of 'more' vector by angle1
                  sin(angle1), cos(angle1), 0;
                  0,0,1];
               
  moreRot = transformMat * [xmore;ymore;1];

  transformed2jaunt = transformMat * [jaunt2plotx;jaunt2ploty;ones(1,40)];
  transformed2jaunt(1:2,:) = transformed2jaunt(1:2,:) + [xintermediate;yintermediate];
  plot(transformed2jaunt(1,:), transformed2jaunt(2,:), 'r');

  xFinal = moreRot(1) + xintermediate;
  yFinal = moreRot(2) + yintermediate;
  angleFinal = angle2 + angle1;

  plot([waypointRobotCoord(1), .1*cos(waypointRobotCoord(3)) + waypointRobotCoord(1)], [waypointRobotCoord(2), .1*sin(waypointRobotCoord(3)) + waypointRobotCoord(2)], 'g*');
  hold off;
end

function yplotstuff(h,event)
  
  newy = get(h, 'value');
  global waypointRobotCoord;
  fieldlength = 5;
  fieldwidth = 4;
  waypointRobotCoord(2) = newy;
  [Distance, Radius, Distance2, Radius2, xcenterUT, ycenterUT, xcenter2UT, ycenter2UT] = waypoint2setpointsv3(waypointRobotCoord(1), waypointRobotCoord(2), waypointRobotCoord(3));

  angle1 = Distance/Radius;

  angle1space = linspace(0,angle1,40);
  jaunt1plotx = Radius * cos(angle1space - pi/2);
  jaunt1ploty = Radius * sin(angle1space - pi/2) + Radius;
  plot(jaunt1plotx, jaunt1ploty, 'b');
  xlim([0,fieldlength]);
  ylim([-fieldwidth/2,fieldwidth/2]);
  %axes('xposition', [.1,.3,.8,.6]);
  pbaspect([fieldlength,fieldwidth,1]);
  hold on;
  xintermediate = Radius*cos(angle1 - pi/2);
  yintermediate = Radius*sin(angle1 - pi/2) + Radius;

  angle2 = Distance2/Radius2;

  angle2space = linspace(0,angle2,40);
  jaunt2plotx = Radius2 * cos(angle2space -pi/2);
  jaunt2ploty = Radius2 * sin(angle2space -pi/2) + Radius2;
  %plot(jaunt2plotx, jaunt2ploty, 'r');

  xmore = Radius2*cos(angle2 - pi/2);
  ymore = Radius2*sin(angle2 - pi/2) + Radius2;

  %transform xmore to coordinates with origin at intermediate position
  %translate to intermediate point, rotate to angle1

  transformMat = [cos(angle1), -sin(angle1), 0; %%ccw rotation of 'more' vector by angle1
                  sin(angle1), cos(angle1), 0;
                  0,0,1];
               
  moreRot = transformMat * [xmore;ymore;1];

  transformed2jaunt = transformMat * [jaunt2plotx;jaunt2ploty;ones(1,40)];
  transformed2jaunt(1:2,:) = transformed2jaunt(1:2,:) + [xintermediate;yintermediate];
  plot(transformed2jaunt(1,:), transformed2jaunt(2,:), 'r');

  xFinal = moreRot(1) + xintermediate;
  yFinal = moreRot(2) + yintermediate;
  angleFinal = angle2 + angle1;

  plot([waypointRobotCoord(1), .1*cos(waypointRobotCoord(3)) + waypointRobotCoord(1)], [waypointRobotCoord(2), .1*sin(waypointRobotCoord(3)) + waypointRobotCoord(2)], 'g*');
  hold off;
end

function thplotstuff(h,event)
  
  newth = get(h, 'value');
  global waypointRobotCoord;
  fieldlength = 5;
  fieldwidth = 4;
  waypointRobotCoord(3) = newth;
  [Distance, Radius, Distance2, Radius2, xcenterUT, ycenterUT, xcenter2UT, ycenter2UT] = waypoint2setpointsv3(waypointRobotCoord(1), waypointRobotCoord(2), waypointRobotCoord(3));

  angle1 = Distance/Radius;

  angle1space = linspace(0,angle1,40);
  jaunt1plotx = Radius * cos(angle1space - pi/2);
  jaunt1ploty = Radius * sin(angle1space - pi/2) + Radius;
  plot(jaunt1plotx, jaunt1ploty, 'b');
  xlim([0,fieldlength]);
  ylim([-fieldwidth/2,fieldwidth/2]);
  %axes('xposition', [.1,.3,.8,.6]);
  pbaspect([fieldlength,fieldwidth,1]);
  hold on;
  xintermediate = Radius*cos(angle1 - pi/2);
  yintermediate = Radius*sin(angle1 - pi/2) + Radius;

  angle2 = Distance2/Radius2;

  angle2space = linspace(0,angle2,40);
  jaunt2plotx = Radius2 * cos(angle2space -pi/2);
  jaunt2ploty = Radius2 * sin(angle2space -pi/2) + Radius2;
  %plot(jaunt2plotx, jaunt2ploty, 'r');

  xmore = Radius2*cos(angle2 - pi/2);
  ymore = Radius2*sin(angle2 - pi/2) + Radius2;

  %transform xmore to coordinates with origin at intermediate position
  %translate to intermediate point, rotate to angle1

  transformMat = [cos(angle1), -sin(angle1), 0; %%ccw rotation of 'more' vector by angle1
                  sin(angle1), cos(angle1), 0;
                  0,0,1];
               
  moreRot = transformMat * [xmore;ymore;1];

  transformed2jaunt = transformMat * [jaunt2plotx;jaunt2ploty;ones(1,40)];
  transformed2jaunt(1:2,:) = transformed2jaunt(1:2,:) + [xintermediate;yintermediate];
  plot(transformed2jaunt(1,:), transformed2jaunt(2,:), 'r');

  xFinal = moreRot(1) + xintermediate;
  yFinal = moreRot(2) + yintermediate;
  angleFinal = angle2 + angle1;

  plot([waypointRobotCoord(1), .1*cos(waypointRobotCoord(3)) + waypointRobotCoord(1)], [waypointRobotCoord(2), .1*sin(waypointRobotCoord(3)) + waypointRobotCoord(2)], 'g*');
  hold off;
end