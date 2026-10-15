function varargout = plt_update(p, t1, t2)
if nargin <= 1
    if nargin == 0
        [ax, ay, az] = frt(0, 0);
        p(1) = plot3(ax, ay, az);
        hold on
        p(2) = plot3(ax(2:3)*0.18211/0.256, ay(2:3)*0.18211/0.256, az(2:3)*0.18211/0.256, 'r.', 'markersize', 30);
        p(3) = plot3(0, 0, 0, 'r.', 'markersize', 30)
        hold off
        varargout{1} = p;
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
    p(1).XData = ax;
    p(1).YData = ay;
    p(1).ZData = az;

    c1 = 0.18211/0.256;
    c1 = [1-c1, c1]';
    p(2).XData = ax(2:3)*c1;
    p(2).YData = ay(2:3)*c1;
    p(2).ZData = az(2:3)*c1;
end
end

function [ax, ay, az] = frt(t1, t2)
L1 = 0.14;                   % [m] distance between rotation axes and pendulum
L2 = 0.256;                  % [m] length of tip-arm

c1 = [L1 0 0]';              % [m]
c2 = [0 0 -L2]';             % [m]

R1 = [cosd(t1) sind(t1) 0; 
    -sind(t1) cosd(t1) 0;
    0 0 1];
R2 = [1 0 0; 
    0 cosd(t2) sind(t2); 
    0 -sind(t2) cosd(t2)];
a0 = [0 0 0]';

a1 = a0 + R1*c1;
a2 = a1 + R1*R2*c2;
ax = [a0(1) a1(1) a2(1)];
ay = [a0(2) a1(2) a2(2)];
az = [a0(3) a1(3) a2(3)];
end
