clear
close all
clc

ts = tic;
s = serialport("COM3",921600,Timeout=5);
pause(0.1)

Av = 150; % [deg/s]
fv = 2; % [hz]

% writeline(s, "c32.0");
while toc(ts) < 5
    t = toc(ts);
    vc = Av * sin(2*pi*fv*t);
    writeline(s, "s" + num2str(vc));
    line = readline(s);
    line = strip(line);
    line = str2num(line);
    encPos = line(1);
    encVel = line(2);
    fprintf("%f\t%f\t%f\n", t, encPos, encVel)
    pause(0.01)
end
writeline(s, "s" + num2str(0));