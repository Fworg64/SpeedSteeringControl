%turnPlotter
%calls the correct turn plotter based on the waypoint given

function [] =  turnPlotter(xi, yi, thi, wpx, wpy, wpth)

    [TwpX, TwpY, TwpTh] = transformPoseToRobotCoord(xi, yi,thi, wpx, wpy, wpth);

    xintercept = -TwpY / tan(TwpTh) + TwpX;
    minSingleTurnRadius = .5;

%     if (xintercept <0 || sign(TwpTh) ~= sign(TwpY))
%         twoTurnPlotter(xi, yi, thi, wpx, wpy, wpth);
%     else
%         oneTurnPlotter(xi, yi, thi, wpx, wpy, wpth);
%     end
% 
%     if ((sign(wpy) ==1 &&(sign(xintercept) ~= sign(wpth))) ...
%             || (sign(wpy) == -1 && (sign(xintercept) == sign(wpth))))
%         twoTurnPlotter(xi,yi,thi,wpx,wpy,wpth);
%     elseif ((sign(wpy) ==  1 && sign(xintercept) ==1 && sign(wpth) ==  1) || ...
%             (sign(wpy) == -1 && sign(xintercept) ==1 && sign(wpth) == -1)|| ...
%              (abs(abs(wpth -thi) -pi )< .01)|| ...
%              (xintercept <0 || sign(wpTh) ~= sign(wpY))) %The 'dead' area
%         oneTurnPlotter(xi, yi, thi, wpx, wpy, wpth);
%     else
%         disp('YOUR HOSED');
%     end

     disp('Start');
     xi = xi
     yi = yi
     thi = thi
     wpx = wpx
     wpy = wpy
     wpth = wpth
     AngleDiff = angleDiff(thi, wpth)
     if (abs(TwpTh) < .001)
         TwpTh = .0001;
     end
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
