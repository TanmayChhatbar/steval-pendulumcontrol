function varargout = plt_update(p, t1, t2)
if nargin <= 1
    if nargin == 0
        varargout{1} = plot3([0 .1 .1], [0 0 0], [0 .1 -.3]);
    end
    daspect([1 1 1])
    xlim([-.32 .32])
    ylim([-.32 .32])
    zlim([-.32 .32])
    grid on
    xlabel("X [m]")
    ylabel("Y [m]")
    zlabel("Z [m]")
    view(3)
elseif nargin == 3
    % update plot
    [ax, ay, az] = frt(-t1, -t2);
    p.XData = ax;
    p.YData = ay;
    p.ZData = az;
    drawnow limitrate
end
end

function [ax, ay, az] = frt(t1, t2)
l1 = [0.1 0 0]'; % [m]
l2 = [0 0 -0.3]'; % [m]
R1 = [cosd(t1) sind(t1) 0; 
    -sind(t1) cosd(t1) 0;
    0 0 1];
R2 = [1 0 0; 
    0 cosd(t2) sind(t2); 
    0 -sind(t2) cosd(t2)];
a0 = [0 0 0]';

a1 = a0 + R1*l1;
a2 = a1 + R1*R2*l2;
ax = [a0(1) a1(1) a2(1)];
ay = [a0(2) a1(2) a2(2)];
az = [a0(3) a1(3) a2(3)];
end