%turnPlotter
%calls the correct turn plotter based on the waypoint given

function [] =  turnPlotter(xi, yi, thi, wpx, wpy, wpth)

     disp('Start');
     xi = xi
     yi = yi
     thi = thi
     wpx = wpx
     wpy = wpy
     wpth = wpth
     AngleDiff = angleDiff(thi, wpth)

    [xi, yi, thi, wpx, wpy, wpth] = inputCleaner(xi, yi, thi, wpx, wpy, wpth);

    [TwpX, TwpY, TwpTh] = transformPoseToRobotCoord(xi, yi,thi, wpx, wpy, wpth)
     
     xintercept = -TwpY / tan(TwpTh) + TwpX
    % minSingleTurnRadius = .5;
     
     if (xintercept ==0) 
         xintercept = .01 % make sign positive if zero
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
