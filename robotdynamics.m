%robotdynamics
%TODO add wheel diameter to calculations (scale factor)
%TODO add slip disturbance
%TODO make inputs torque
function dx = robotdynamics(Ul,Ur, worldTheta,dt, wheelR, AxelLen) %input is wheel velocities
%x is fwd, y is left/right and cannot be travveled upon

Ur = Ur*wheelR;
Ul = Ul*wheelR;

w = (Ur - Ul)/AxelLen; %positive is CCW, i.e. if right wheel is faster, angle is positive
if (w~=0)
  R = AxelLen/2 * (Ur + Ul)/(Ur - Ul);
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