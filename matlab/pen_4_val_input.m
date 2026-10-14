clear
clc

x0 = [172 50]';
dt = 0.01;
t_span = 0:dt:10;
x = zeros(2, length(t_span));
x(:, 1) = x0;
a = zeros(size(t_span))';
for i = 2:length(t_span)
    fx = rk4(x(:, i-1), @f, 0, dt);
    a(i) = fx(2);
    x(:, i) = x(:, i-1) + fx*dt;
end

%% plot
figure(1)
clf
sp(1) = subplot(221);
plot(t_span, x(1, :))
grid on
xlabel("time [s]")
ylabel("angle [deg]")

sp(1) = subplot(222);
plot(t_span, x(2, :))
grid on
xlabel("time [s]")
ylabel("angular rate [deg/s]")

sp(1) = subplot(223);
plot(t_span, a)
grid on
xlabel("time [s]")
ylabel("angular rate[deg/s]")

linkaxes(sp, 'x')
fontsize(14, 'points')

% animate
figure(2);
pp = plt_update();
for i = 1:length(t_span)
    plt_update(pp, 0, x(1, i))
    drawnow
end

%% functions
function fx = f(x, u)
% global constants
g = 9.81;

% pendulum constants
l = 0.18211;                 % [m] for no weight attached
d = 0.021421;                % [Nm.s/deg]
% f = 27.692; % [Nm]
f = 0;                       % [Nm] remove friction (insignificant)
m = .2;                      % [-] constant for steepness of tanh approximation of friction

% input constants
lf = 0.14;                   % [m] distance between rotation axes and pendulum


% f(x) calculation
fx = [x(2);
    -g/l*180/pi*sind(x(1)) - d/l^2*x(2) - f*tanh(x(2)*m)];
end