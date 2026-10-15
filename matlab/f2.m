function fx = f2(x, u)
% x = [stepPos[rad], encPos[rad], encVel[rad/s], lastStepVel[rad/s]]'
% u = [stepVel[rad/s]]

% global constants
g = 9.81;
dt = 1e-3;

% pendulum constants
L1 = 0.14;                   % [m] distance between rotation axes and pendulum
l2 = 0.18211;                % [m] for no weight attached
b2 = deg2rad(0.021421);      % [Nm.s/deg]
Ii2 = 84655.81 / 1e3^3;      % [kg.m^2]
m2 = 15.48 / 1e3;            % [kg]
% f2 = 27.692;                 % [Nm]
% f2 = 0;                      % [Nm] remove friction (insignificant)
% m = .2;                      % [-] constant for steepness of tanh approximation of friction

x1 = x(1);
x2 = x(2);
x3 = x(3);
x4 = x(4);

% f(x) calculation
fx = [
u;
x3;
-(b2*x3 - (Ii2*u^2*sin(2*x2))/2 + g*l2*m2*sin(x2) - (l2^2*m2*u^2*sin(2*x2))/2 + (L1*l2*m2*cos(x2)*(u - x4))/dt)/(m2*l2^2 + Ii2);
0];

end
