%input cleaner
% makes sure that the input passed to the turn plotter
% will yield a solution by fudging the numbers a bit

function [xi, yi, thi, wpx, wpy, wpth] = inputCleaner(xi, yi, thi, wpx, wpy, wpth)

    fudged =0;
    %if waypoint and robot are exactly in line with each other
    %bump the supposed position of the robot a bit
    [TwpX, TwpY, TwpTh] = transformPoseToRobotCoord(xi, yi,thi, wpx, wpy, wpth);
    if (abs(TwpY) < .01)
         disp('Fudge1');
         fudged = 1;
         xi = xi + .02 * cos(thi + pi/2);
         yi = yi + .02 * sin(thi + pi/2);
         
         %%if they were in a line and pointing the same way
         if (abs(TwpTh) < .01)
             thi = thi - .01;
             wpth = wpth + .01;
         end
         return;
    end
    
    %if the xintercept would be zero
    %make it not that by sliding the robot fwd  or sideways a bit
    if (tan(TwpTh) ~= 0)
        xintercept = -TwpY / tan(TwpTh) + TwpX
        if (abs(xintercept) <0.01)
             disp("chocolate");
             xi = xi + .02 * cos(thi + pi/2);
             yi = yi + .02 * sin(thi + pi/2);
             %check x intercept again
             [TwpX, TwpY, TwpTh] = transformPoseToRobotCoord(xi, yi,thi, wpx, wpy, wpth);
             if (tan(TwpTh) ~= 0)
                 xintercept = -TwpY / tan(TwpTh) + TwpX
                 if (abs(xintercept) <0.01)
                     disp("chocolate again");
                     xi = xi + .02 * cos(thi) - .02 * cos(thi + pi/2);
                     yi = yi + .02 * sin(thi) - .02 * sin(thi + pi/2);
                 end
             end
         return; %return if a correction was made
         end
    end

    %if the waypoint is pointing exactly up or down
    %make sure it's not doing that
    if ((abs(angleDiff(pi/2, TwpTh)) < .01) || (abs(angleDiff(-pi/2, TwpTh)) < .01))
         disp('Caramel1');
         %fudged = 1;
         wpth = angleDiff(wpth,-.01);
         %TwpTh = angleDiff(TwpTh, -.01);
         return;
    end

    %if the waypoint and robot are parralel but opposite facing exactly
    %make them not do that
    if (abs(TwpTh) > (pi - .01))
         disp('Caramel2');
         wpth = angleDiff(wpth, -.01);
         return;
    end
    
    %if the waypoint and robot are parralel and facing the same
    %make them not do that
    if (abs(TwpTh) < .01)
         disp('Caramel3');
         thi = angleDiff(thi, -.01);
         return;
    end
    
    %returns normally here if no correction was made
end
