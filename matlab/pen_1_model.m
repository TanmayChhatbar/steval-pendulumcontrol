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
    tstart = tic;
    ts = [];
    while is_moving | toc(not_moving_timer) < 10
        % read
        [new_valid, encPos_new, encVel_new, ~, t_new] = pen_readState(s);
        if ~new_valid
            continue
        end
        encPos(end+1) = encPos_new;
        encVel(end+1) = encVel_new;
        t(end+1) = t_new;
        ts(end+1) = toc(tstart);
        
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
    nstart = 100;
    tuse = t;
    p = smooth(encPos, 10);
    v = diff(p)./diff(tuse); 
    v = [v; v(end)];
    % v = [v(1); v];
    
    a = diff(v)./diff(tuse);
    a = [a; a(end)];
    % a = [a(1); a];
    
    v = smooth(v, 5);
    a = smooth(a, 10);
    p = p(nstart:end);
    v = v(nstart:end);
    a = a(nstart:end);
    tuse = tuse(nstart:end);
end
istatic = find(v ~= 0, 1, 'last');
p = p(1:(istatic+10));
v = v(1:(istatic+10));
a = a(1:(istatic+10));
tuse = tuse(1:(istatic+10));

sp(1) = subplot(311);
plot(tuse, p)
grid on
ylabel("angle [deg]")

sp(2) = subplot(312);
plot(tuse, v)
grid on
ylabel("angular rate [deg/s]")

sp(3) = subplot(313);
plot(tuse, a)
grid on
linkaxes(sp, 'x')
ylabel("angular acceleration [deg/s]")
xlabel("time [s]")
fontsize(14, 'points')

%% fit
% thetadotdot = -g/l*sintheta - d/l^2 thetadot - f sign(thetadot)
pfit = p;
vfit = v;
afit = a;
m = .2;

H = [-sind(pfit), -vfit, -tanh(vfit*m)];
params = H \ afit;
g = 9.81;
l = g / params(1) * 180/pi;
d = l^2 * params(2);
f = params(3);

t = table([m; l; d; f], 'RowNames', ["m [-]", "l [m]", "d [Nm.s/deg]", "f [Nm]"], VariableNames="Parameters");
disp(t)

% verify
aest = -g/l*180/pi*sind(pfit) - d/l^2*vfit - f*tanh(vfit*m);
subplot(313)
hold on
plot(tuse, aest, '-.')
hold off

%% save
% save('data.mat', 'p', 'v', 'a', 't', 'tuse')

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
