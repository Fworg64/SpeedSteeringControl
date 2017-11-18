%angleDiff
%finds the minimum difference between two angles
%output range -PI to PI

function [diff] = angleDiff(x, y)
    diff = atan2(sin(x-y),cos(x-y));
end