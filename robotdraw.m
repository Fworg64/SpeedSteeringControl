%robotdraw
function [] = robotdraw(x,y,theta)
fieldlength = 7.38;
fieldwidth = 3.78;
startinglength = 1.5;
obstaclelength = 2.94;
%clf
hold on;
plot([startinglength, startinglength,0],[-fieldwidth/2,0,0]);
plot([startinglength, startinglength,0],[fieldwidth/2,0,0]);
plot([startinglength + obstaclelength,startinglength + obstaclelength],[-fieldwidth/2,fieldwidth/2]);


plot([x-.25*cos(theta),x+.5*cos(theta)],[y-.25*sin(theta),y+.5*sin(theta)]);
xlim([0,fieldlength]);
ylim([-fieldwidth/2,fieldwidth/2]);
pbaspect([fieldlength,fieldwidth,1]);
hold off;
pause(.01)