%turnPlotter
%calls the correct turn plotter based on the waypoint given

function [] =  turnPlotter(xi, yi, thi, wpx, wpy, wpth)

    [TwpX, TwpY, TwpTh] = transformPoseToRobotCoord(xi, yi,thi, wpx, wpy, wpth);

    xintercept = -TwpY / tan(TwpTh) + TwpX;

    if (xintercept <0 || sign(TwpTh) ~= sign(TwpY))
        twoTurnPlotter(xi, yi, thi, wpx, wpy, wpth);
    else
        oneTurnPlotter(xi, yi, thi, wpx, wpy, wpth);
    end

end