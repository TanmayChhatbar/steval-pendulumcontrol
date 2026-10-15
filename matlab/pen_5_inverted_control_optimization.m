% On the Dynamics of the Furuta Pendulum
% https://onlinelibrary.wiley.com/doi/epdf/10.1155/2011/528341
clear
clc

x0 = [20 1000 100 0];
[optx, score] = fmincon(@valscore, x0, [], [], [], [], ones(size(x0))*0, ones(size(x0))*10000);
disp("qd = ["+join(string(num2str(optx')), ', ') + "];")
return

%% plot
[score, score_t, xv] = valscore(optx);
dt = 1e-3;
t = (1:length(score_t))*dt;
figure(3); clf
m = ceil(sqrt(height(xv)+1));
n = ceil((height(xv)+1) / m);
for i = 1:height(xv)
    subplot(m, n, i)
    plot(t, xv(i, :))
    xlabel("time [s]")
    ylabel("x"+num2str(i))
end
subplot(m, n, height(xv)+1)
plot(t, score_t)
xlabel("time [s]")
ylabel("score")

%% simulation
function [score, score_t, x] = valscore(xv)
L1 = 0.14;                   % [m] distance between rotation axes and pendulum
l2 = 0.18211;                % [m] for no weight attached
b2 = deg2rad(0.021421);      % [Nm.s/deg]
Ii2 = 84655.81 / 1e3^3;      % [kg.m^2]
m2 = 15.48 / 1e3;            % [kg]
g = 9.81;
dt = 1e-3;

% linearized model in rad/s
A = [0, 0, 0, 0;
    0, 0, 1, 0;
    0, (g*l2*m2)/(Ii2 + l2^2*m2), -b2/(Ii2 + l2^2*m2), -(L1*l2*m2)/(dt*(Ii2 + l2^2*m2));
    0, 0, 0, -1
];
B = [1;
    0;
    (L1*l2*m2)/(dt*(Ii2 + l2^2*m2));
    1
];

% LQR
Q = diag(xv);
R = 1;
try
    K = lqr(A, B, Q, R);
catch
    score = inf;
    return
end

dt = 1e-3;
n = 500;
x = zeros(4, n);
x(:, 1) = [0 165 0 0]'; % [deg, deg, deg/s, deg/s]
x_sp = [0 180 0 0]';
u_p = 0;
for i = 2:width(x)
    % control input
    u = K*deg2rad(x_sp-x(:,i-1)); % [deg]
    % du_max = inf/dt;
    % u_max = inf;
    % du = min(max(u-u_p, -du_max), du_max);
    % u = u + du;
    % u = min(max(u, -u_max), u_max);
    % u_p = u;

    % fx
    % fx_rad = rk4(deg2rad(x(:, i-1)), @f2, deg2rad(u), dt);
    fx_rad = f2(deg2rad(x(:, i-1)), deg2rad(u));
    % state update
    x(:, i) = x(:,i-1) + rad2deg(fx_rad)*dt;
    x(4, i) = u;
end

% calculate score
% score = sum((x-x_sp)*(x-x_sp)', 'all');
err = x-x_sp;
% [stepPos, encPos, encVel, stepVelLast]
err = err .* [40 2 1 0.1]';
score_t = sum(err.^2, 'all');
score = sum(score_t);
end
