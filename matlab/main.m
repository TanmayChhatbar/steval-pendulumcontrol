clear
close all
clc

s = serialport("COM3",921600,Timeout=5);

pen_writeCurrent(s, 1500);
pen_Reset(s);

% p = plt_update();
is_moving = 1;
not_moving_timer = tic;
ts = tic;

% mode = 0 == sinusoidal
% mode = 1 == inverted stabilize
mode = 1;
if mode == 0
    Av = 200; % [deg/s]
    fv = 2; % [hz]
elseif mode == 1
    setpoint = 180;
end

tlast = toc(ts);
encPos = 0;
encVel = 0;
stepperPos = 0;
errcum = 0;
t_read = tlast;
t_write = tlast;
while toc(ts) < 30
    t = toc(ts);

    % read
    [new_valid, encPos_new, encVel_new, stepperPos_new] = pen_readState(s);
    if ~new_valid
        continue
    else
        encPos = encPos_new;
        encVel = encVel_new;
        stepperPos = stepperPos_new;
        t_read(end+1) = t;
    end
    
    % stop condition
    if abs(encVel) > 0.05
        is_moving = 1;
        not_moving_timer = tic;
    else
        if toc(not_moving_timer) > 2
            % break
        end
    end

    % control
    if mode == 0
        vc = (Av * sin(2*pi*fv*t)) * (t < 5);
    elseif mode == 1
        dt = t - tlast;
        err = setpoint - encPos;
        errcum = errcum + err * dt;
        if abs(err) < 15
            kp = 5;
            ki = 0.05;
            kd = 7;
            vc = kp * err + ki * errcum * double(abs(err)<5) + kd * -encVel;
        else
            vc = 0;
        end
        tlast = t;
    end

    % output
    % pen_writeSpeed(s, vc);
    % t_write(end+1) = t;

    % output
    % fprintf("%f\t%f\t%f\t%f\n", t, encPos, encVel, stepperPos)
    % plt_update(p, stepperPos, encPos)
end

%% end
pen_writeSpeed(s, 0);
pen_writeCurrent(s, 250);
s.delete()

%% plot stats
subplot(211)
histogram(diff(t_read), numbins=301)
subplot(212)
histogram(diff(t_write), numbins=301)

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

function pen_writeSpeed(s, vc)
writeline(s, "s" + num2str(vc));
end

function pen_writeCurrent(s, c)
writeline(s, "c"+num2str(c));
end

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

function [new_valid, encPos, encVel, stepperPos] = pen_readState(s)
if s.NumBytesAvailable == 0
    encPos = nan;
    encVel = nan;
    stepperPos = nan;
    new_valid = 0;
else
    line_raw = readline(s);
    line = strip(line_raw);
    line = str2num(line);
    if length(line) == 3
        encPos = line(1);
        encVel = line(2);
        stepperPos = line(3);
        new_valid = 1;
    else
        encPos = nan;
        encVel = nan;
        stepperPos = nan;
        new_valid = 0;
        fprintf("unknown line: %s", line_raw);
    end
    end
end

function pen_Reset(s)
writeline(s, "r");
end

function l = readline2(s)
l = "";
while s.NumBytesAvailable > 0
    raw_data = read(s, s.NumBytesAvailable, "string");
    lines = split(raw_data, newline);
    l = lines(1);
end
end
