function [x,y,theta] = newFindCPP(xc, yc, radius, rx, ry)

  theta = atan2(ry - yc, rx -xc) + pi/2 * sign(radius);
  theta = angleDiff(theta,0);
  
  tangenttheta = atan2(ry - yc, rx -xc);
 
  x = abs(radius)*cos(tangenttheta) + xc;
  y = abs(radius)*sin(tangenttheta) + yc;

end