%{
DO NOT USE
matlab causes a lot of jitter for serial
use python/main.py instead
%}
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
    Av = 200;                % [deg/s]
    fv = 2;                  % [hz]
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
is_moving = 1;
while toc(ts) < 10
    t = toc(ts);

    % read
    [new_valid, encPos_new, encVel_new, stepperPos_new] = pen_readState(s);
    if ~new_valid
        continue
    else
        encVel = (encPos_new - encPos) / (t_read(end) - t);
        encPos = encPos_new;
        % encVel = encVel_new;
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
            kp = 16;
            ki = 0.;
            kd = 0.03;
            vc = kp * err + ...
                 ki * errcum + ...
                 kd * -encVel;
        else
            vc = 0;
        end
        tlast = t;
    end

    % output
    pen_writeSpeed(s, vc);
    t_write(end+1) = t;

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
function pen_writeSpeed(s, vc)
writeline(s, "s" + num2str(vc));
end

function pen_writeCurrent(s, c)
writeline(s, "c"+num2str(c));
end

function [new_valid, encPos, encVel, stepperPos, t] = pen_readState(s)
encPos = nan;
encVel = nan;
stepperPos = nan;
new_valid = 0;
t = nan;
% while s.NumBytesAvailable > 0
    line_raw = readline(s);
    if isempty(line_raw)
        return
    end
    line = sscanf(line_raw, "%f,%f,%f,%f");
    if numel(line) == 4
        t = line(1);
        encPos = line(2);
        encVel = line(3);
        stepperPos = line(4);
        new_valid = 1;
    else
        fprintf("unknown line: %s", line_raw);
    end
% end
end

function pen_Reset(s)
writeline(s, "r");
end
