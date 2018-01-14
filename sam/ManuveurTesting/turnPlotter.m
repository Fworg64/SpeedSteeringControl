%turnPlotter
%calls the correct turn plotter based on the waypoint given

function [] =  turnPlotter(xi, yi, thi, wpx, wpy, wpth)

    [TwpX, TwpY, TwpTh] = transformPoseToRobotCoord(xi, yi,thi, wpx, wpy, wpth)
     if (abs(TwpTh) < .001)
         TwpTh = .0001;
     end

     disp('Start');
     xi = xi
     yi = yi
     thi = thi
     wpx = wpx
     wpy = wpy
     wpth = wpth
     AngleDiff = angleDiff(thi, wpth)
     
     fudged = 0;
     if (abs(TwpY) < .01)
         disp('Fudge1');
         fudged = 1;
         wpx = wpx + .02 * sin(wpth + pi/2);
         wpy = wpy + .02 * cos(wpth + pi/2);
         
         [TwpX, TwpY, TwpTh] = transformPoseToRobotCoord(xi, yi,thi, wpx, wpy, wpth)

         if (abs(TwpTh) < .001)
             wpth = wpth + .001;
             TwpTh = .001;
         end
     end
     
     angleDiff(pi,TwpTh)
     if (abs(angleDiff(pi,TwpTh)) < .001 && fudged ==0)
         disp('Fudge2');
         fudged = 1;
         wpth = angleDiff(wpth,-.0001);
         TwpTh = angleDiff(TwpTh, -.0001);
         
     end
     
     if ((abs(angleDiff(pi/2, TwpTh)) < .001) || (abs(angleDiff(-pi/2, TwpTh)) < .01))
         disp('Caramel1');
         %fudged = 1;
         wpth = angleDiff(wpth,-.0001)
         TwpTh = angleDiff(TwpTh, -.0001)
     end
     
     xintercept = -TwpY / tan(TwpTh) + TwpX
    % minSingleTurnRadius = .5;
     
     if (xintercept ==0) 
         xintercept = .0001; % make sign positive if zero
     end
     
     %if (abs(thi) < .001)
     %    thi = -.0001;
     %end

     if (sign(TwpY) ==1)
         if (sign(xintercept) ~= sign(TwpTh))
            %two turn
            twoTurnPlotter(xi, yi, thi, wpx, wpy, wpth)
         elseif (sign(xintercept) ==1 && sign(TwpTh) ==1)
            %single turn
            oneTurnPlotter(xi, yi, thi, wpx, wpy, wpth);
         elseif (sign(xintercept) ==-1 && sign(TwpTh) ==-1)
            %inverse single turn
            inverseOneTurnPlotter(xi, yi, thi, wpx, wpy, wpth)
         end
     elseif (sign(TwpY)== -1)
         if (sign(xintercept) ==sign(TwpTh))
             %two turn
             twoTurnPlotter(xi, yi, thi, wpx, wpy, wpth)
         elseif (sign(xintercept) ==1 && sign(TwpTh) ==-1)
             %single turn
             oneTurnPlotter(xi, yi, thi, wpx, wpy, wpth);
         elseif (sign(xintercept) ==-1 && sign(TwpTh) ==1)
             %inverse single turn
             inverseOneTurnPlotter(xi, yi, thi, wpx, wpy, wpth)
         end
     end

end
