%robotcontrol
function [Ul, Ur] = robotcontrol(X) %15 states in, x,y,theta,leftW,rightW and 2 d/dt's
%also track disturbance on theta' for sidewall forces to be overcome
%need to add servo state as well