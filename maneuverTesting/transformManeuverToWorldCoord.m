%transformManeuverToWorldCoord

function [distance, radius, xc, yc] = transformManeuverToWorldCoord(rX, rY, rTh, UTd, UTr, UTxc, UTyc)
    xc = cos(rTh)*UTxc + sin(rTh)*UTyc + rX;
    yc = -sin(rTh)*UTxc + cos(rTh)*UTyc +rY;
    distance = UTd;
    radius = UTr;
end