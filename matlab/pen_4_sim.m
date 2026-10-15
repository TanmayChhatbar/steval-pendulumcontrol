% On the Dynamics of the Furuta Pendulum
% https://onlinelibrary.wiley.com/doi/epdf/10.1155/2011/528341
clear
clc

% params
dt = 1e-3;
n = 1e4;
ts = (1:n)*dt;

%% simulation
x = zeros(4, n);
x(:, 1) = [0 1 0 0]'; % [deg, deg, deg/s, deg/s]
u = 0;
for i = 2:width(x)
    % x(:, i) = x(:,i-1) + rk4(x(:, i-1), @f1, u, dt)*dt;
    
    % u = 30*sin(2*pi*1*ts(i)); % [deg]
    if i < 0.1/dt
        u = 880;
    else
        u = 0;
    end
    fx_rad = rk4(deg2rad(x(:, i-1)), @f2, deg2rad(u), dt);
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
