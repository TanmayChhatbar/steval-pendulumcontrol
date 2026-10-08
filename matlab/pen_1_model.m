clear
close all
clc

use_data = true;

if ~use_data
    s = serialport("COM3",921600,Timeout=5);
    pen_writeCurrent(s, 1500);
    pen_Reset(s);
    pause(0.1)
    is_moving = 1;
    t = [];
    encPos = [];
    encVel = [];
    not_moving_timer = tic;
    t_start = tic;
    ts = [];
    while is_moving || toc(not_moving_timer) < 10
        % read
        [new_valid, encPos_new, encVel_new, ~, t_new] = pen_readState(s);
        if ~new_valid
            continue
        end
        encPos(end+1) = encPos_new;
        encVel(end+1) = encVel_new;
        t(end+1) = t_new;
        ts(end+1) = toc(t_start);
        
        % stop condition
        if abs(encVel(end)) > 0.05
            is_moving = 1;
            not_moving_timer = tic;
        else
            if toc(not_moving_timer) > .5
                is_moving = 0;
            end
        end
    
    end
    t = t(11:end)';
    ts = ts(11:end)';
    ts = ts - ts(1) + t(1);
    encPos = encPos(11:end)';
    encPos = encPos - encPos(end); % center
    encVel = encVel(11:end)';
    
    %% end
    pen_writeCurrent(s, 250);
    s.delete()
else
    load("data.mat")
end

%% plot response
if ~use_data
    n_start = 100;
    t_use = t;
    p = smooth(encPos, 10);
    v = diff(p)./diff(t_use); 
    v = [v; v(end)];
    % v = [v(1); v];
    
    a = diff(v)./diff(t_use);
    a = [a; a(end)];
    % a = [a(1); a];
    
    v = smooth(v, 5);
    a = smooth(a, 10);
    p = p(n_start:end);
    v = v(n_start:end);
    a = a(n_start:end);
    t_use = t_use(n_start:end);
end

i_static = find(v ~= 0, 1, 'last');
p = p(1:(i_static+10));
v = v(1:(i_static+10));
a = a(1:(i_static+10));
t_use = t_use(1:(i_static+10));

sp(1) = subplot(311);
plot(t_use, p)
grid on
ylabel("angle [deg]")

sp(2) = subplot(312);
plot(t_use, v)
grid on
ylabel("angular rate [deg/s]")

sp(3) = subplot(313);
plot(t_use, a)
grid on
linkaxes(sp, 'x')
ylabel("angular acceleration [deg/s]")
xlabel("time [s]")
fontsize(14, 'points')

%% fit
% theta_dot_dot = -g/l*sin(theta) - d/l^2 theta_dot - f sign(theta_dot)
p_fit = p;
v_fit = v;
a_fit = a;
m = .2;

H = [-sind(p_fit), -v_fit, -tanh(v_fit*m)];
params = H \ a_fit;
g = 9.81;
l = g / params(1) * 180/pi;
d = l^2 * params(2);
f = params(3);

t = table([m; l; d; f], 'RowNames', ["m [-]", "l [m]", "d [Nm.s/deg]", "f [Nm]"], VariableNames="Parameters");
disp(t)

% verify
a_est = -g/l*180/pi*sind(p_fit) - d/l^2*v_fit - f*tanh(v_fit*m);
subplot(313)
hold on
plot(t_use, a_est, '-.')
hold off

%% save
% save('data.mat', 'p', 'v', 'a', 't', 't_use')

%% functions
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
