clear
close all
clc

ts = tic;
s = serialport("COM3",921600,Timeout=5);
pause(0.1)

Av = 200; % [deg/s]
fv = 2; % [hz]
currentLimit = 1500;
writeline(s, "c"+num2str(1500));
pause(0.05)
p = plot3([0 .1 .1], [0 0 0], [0 .1 -.3]);
daspect([1 1 1])
xlim([-.32 .32])
ylim([-.32 .32])
zlim([-.32 .32])
grid on
xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")
view(3)

is_moving = 1;
not_moving_timer = tic;
while toc(ts) < 7 | is_moving
    t = toc(ts);
    if toc(ts) < 5
        vc = Av * sin(2*pi*fv*t);
    else
        vc = 0;
    end
    writeline(s, "s" + num2str(vc));
    line_raw = readline(s);
    line = strip(line_raw);
    line = str2num(line);
    if length(line) == 3
        encPos = line(1);
        encVel = line(2);
        if abs(encVel) > 0.01
            not_moving_timer = tic;
            is_moving = 1;
        else
            if toc(not_moving_timer) > 1
                break
            end
        end
        stepperPos = line(3);
        fprintf("%f\t%f\t%f\t%f\n", t, encPos, encVel, stepperPos)
        
        % update plot
        [ax, ay, az] = frt(-stepperPos, -encPos);
        p.XData = ax;
        p.YData = ay;
        p.ZData = az;
        drawnow limitrate
    else
        line_raw
    end
    % pause(0.1)
end
writeline(s, "c"+num2str(250));
s.delete()

%% functions
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