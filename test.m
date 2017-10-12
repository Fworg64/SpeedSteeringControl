
VescGains = [-8,4,2];
LwheelSpeed =0;
LeftVesticle =0;
X = .3;
dt = .01;

time = 0:dt:10;
plots = zeros(2,length(time)); plotindex = 1;

for t = time
LwheelSpeed = LwheelSpeed - 8*(LwheelSpeed - X)*dt;
%LeftVesticle = LeftVesticle +VescGains(1)* (LwheelSpeed - X)*dt;
plots(1,plotindex) = LwheelSpeed;
plots(2,plotindex) = LeftVesticle;
plotindex = plotindex+1;

end

plot(time, plots);
legend('LwheelSpeed','LeftVesticle')