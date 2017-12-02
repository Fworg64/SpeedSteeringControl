%transformPoseToRobotCoord
%takes a pose (like a waypoint) and transforms it to robot coord

function [x, y, th] = transformPoseToRobotCoord(rX, rY, rTh, pX, pY, pTh)
x = cos(rTh)*(pX-rX) + sin(rTh)*(pY-rY);
y = -sin(rTh)*(pX-rX) + cos(rTh)*(pY-rY);
th = angleDiff(pTh,rTh);
end
