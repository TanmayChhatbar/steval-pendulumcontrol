% On the Dynamics of the Furuta Pendulum
% https://onlinelibrary.wiley.com/doi/epdf/10.1155/2011/528341
clear
clc

% params
dt = 1e-3;
n = 1e4;
ts = (1:n)*dt;

%% linearization
% linearized model
% x = [stepPos, encPos, encVel, lastStepVel]
% u = [stepVel]

L1 = 0.14;                   % [m] distance between rotation axes and pendulum
l2 = 0.18211;                % [m] for no weight attached
b2 = deg2rad(0.021421);      % [Nm.s/rad]
Ii2 = 84655.81 / 1e3^3;      % [kg.m^2]
m2 = 15.48 / 1e3;            % [kg]
g = 9.81;

% linearized model in rad/s
A = [0,                         0,                   0,                                0;
     0,                         0,                   1,                                0;
     0, (g*l2*m2)/(m2*l2^2 + Ii2), -b2/(m2*l2^2 + Ii2), -(L1*l2*m2)/(dt*(m2*l2^2 + Ii2));
     0,                         0,                   0,                                -1];
B = [1
     0
     (L1*l2*m2)/(dt*(m2*l2^2 + Ii2))
     1];

%% LQR
Q = diag([1.1 15000 100 0]);
R = 1;
K = lqr(A, B, Q, R)

%% simulation
x = zeros(4, n);
x(:, 1) = [0 170 0 0]'; % [deg, deg, deg/s, deg/s]
x_sp = [0 180 0 0]';
u_p = 0;
for i = 2:width(x)
    % control input
    u = K*deg2rad(x_sp-x(:,i-1)); % [deg]
    du = min(max(u-u_p, -300/dt), 300/dt);
    u = u + du;
    u = min(max(u, -360), 360);
    u_p = u;

    % fx
    fx_rad = rk4(deg2rad(x(:, i-1)), @f2, deg2rad(u), dt);

    % state update
    x(:, i) = x(:,i-1) + rad2deg(fx_rad)*dt;
    x(4, i) = u;
end

%% plot
figure(1)
for i = 1:height(x)
    subplot(height(x), 1, i)
    plot(ts, x(i, :))
    grid on
end

% animate
figure(2)
p = plt_update();
fps = 50;
for i = find([0 diff(rem(ts, 1/fps))<0])
    tnow = tic;
    plt_update(p, x(1, i), x(2, i));
    title(num2str(ts(i)) + " s")
    drawnow
    tnow = toc(tnow);
    pause(max(1/fps - tnow, 0))
end
