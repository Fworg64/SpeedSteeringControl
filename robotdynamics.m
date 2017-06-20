%robotdynamics
%TODO add wheel diameter to calculations (scale factor)
function dx = robotdynamics(Ul,Ur, worldTheta,dt) %input is wheel velocities
%x is fwd, y is left/right and cannot be travveled upon

width = .5; %width of axel
wheeldia = .3;

w = (Ur - Ul)/width; %positive is CCW, i.e. if right wheel is faster, angle is pos
if (w~=0)
  R = width/2 * (Ur + Ul)/(Ur - Ul);
  %let R be on the robots Y axis
  rot = [cos(w*dt), -sin(w*dt);sin(w*dt),cos(w*dt)];
  dPos = rot*[0;-R] + [0;R];
  dTheta = w*dt;
else
  dPos = [Ul;0;]*dt;
  dTheta = 0;
end

%dPos is in robot coordinates, must transform to world.
%rotate by worldrobot theta

wrot = [cos(worldTheta), -sin(worldTheta);sin(worldTheta),cos(worldTheta)];
dPosW = wrot*dPos;

dx = [dPosW(1);dPosW(2);dTheta]; 