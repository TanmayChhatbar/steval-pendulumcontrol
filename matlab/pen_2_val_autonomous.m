clear
clc

x0 = [172 61.306]';
dt = 0.01;
t_span = 0:dt:10;
x = zeros(2, length(t_span));
x(:, 1) = x0;
a = zeros(size(t_span))';
for i = 2:length(t_span)
    fx = rk4(x(:, i-1), @f, dt);
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
function fx = f(x)
g = 9.81;
l = 0.18211;
d = 0.021421;
% f = 27.692;
f = 0;                       % remove friction
m = .2;
fx = [x(2);
    -g/l*180/pi*sind(x(1)) - d/l^2*x(2) - f*tanh(x(2)*m)];
end

function dxdt = rk4(x, f, dt)
k1 = f(x);
k2 = f(x + 0.5 * dt * k1);
k3 = f(x + 0.5 * dt * k2);
k4 = f(x + dt * k3);
dxdt = (k1 + 2*k2 + 2*k3 + k4) / 6;
end
