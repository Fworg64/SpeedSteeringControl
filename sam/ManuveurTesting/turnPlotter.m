%turnPlotter
%calls the correct turn plotter based on the waypoint given

function [] =  turnPlotter(xi, yi, thi, wpx, wpy, wpth)

    [TwpX, TwpY, TwpTh] = transformPoseToRobotCoord(xi, yi,thi, wpx, wpy, wpth);

    
     if (abs(TwpTh) < .001)
         TwpTh = .0001;
     end

     xintercept = -TwpY / tan(TwpTh) + TwpX
     minSingleTurnRadius = .5;

     disp('Start');
     xi = xi
     yi = yi
     thi = thi
     wpx = wpx
     wpy = wpy
     wpth = wpth
     AngleDiff = angleDiff(thi, wpth)

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
