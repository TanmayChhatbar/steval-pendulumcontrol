function fx = f(x, u)
% x = [encPos, enVel, stepPos, stepVel]
% u = [stepTau]

% global constants
g = 9.81;

% pendulum constants (root-side)
L1 = 0.14;                   % [m] distance between rotation axes and pendulum
l1 = L1 / 2; % [m]
m1 = 7.86 / 1e3; % [kg]
b1 = 0; % [Nm.s/deg]
% f1 = 0;
Ii1 = 11123.60 / 1e3^3;

% pendulum constants (tip-side)
l2 = 0.18211;                 % [m] for no weight attached
b2 = 0.021421;                % [Nm.s/deg]
% f = 27.692; % [Nm]
% f2 = 0;                       % [Nm] remove friction (insignificant)
% m = .2;                      % [-] constant for steepness of tanh approximation of friction
Ii2 = 84655.81 / 1e3^3;
m2 = 15.48 / 1e3; % [kg]

% input constants
x1 = x(1);
x2 = x(2);
x3 = x(3);
x4 = x(4);

% f(x) calculation
fx = [
x2;
(2*Ii2*u + 2*l2^2*m2*u - 2*Ii2*b1*x2 - 2*Ii2^2*x2*x4*sin(2*x3) - 2*b1*l2^2*m2*x2 + 2*L1*l2^3*m2^2*x4^2*sin(x3) - 2*l2^4*m2^2*x2*x4*sin(2*x3) + L1*g*l2^2*m2^2*sin(2*x3) - 4*Ii2*l2^2*m2*x2*x4*sin(2*x3) + 2*L1*b2*l2*m2*x4*cos(x3) + 2*Ii2*L1*l2*m2*x4^2*sin(x3) + 2*L1*l2^3*m2^2*x2^2*sin(x3)*(sin(x3)^2 - 1) + 2*Ii2*L1*l2*m2*x2^2*sin(x3)*(sin(x3)^2 - 1))/(2*(Ii2^2 - Ii2^2*cos(x3)^2 + Ii1*Ii2 + l2^4*m2^2 - l2^4*m2^2*cos(x3)^2 + L1^2*l2^2*m2^2 + Ii2*L1^2*m2 + Ii2*l1^2*m1 + Ii1*l2^2*m2 + 2*Ii2*l2^2*m2 - 2*Ii2*l2^2*m2*cos(x3)^2 + l1^2*l2^2*m1*m2 - L1^2*l2^2*m2^2*cos(x3)^2));
x4;
(Ii2^2*x2^2*sin(2*x3) - (Ii2^2*x2^2*sin(4*x3))/2 - 4*Ii1*b2*x4 - 2*Ii2*b2*x4 + 2*Ii1*Ii2*x2^2*sin(2*x3) + l2^4*m2^2*x2^2*sin(2*x3) - (l2^4*m2^2*x2^2*sin(4*x3))/2 - 3*g*l2^3*m2^2*sin(x3) - 4*L1^2*b2*m2*x4 - 4*b2*l1^2*m1*x4 - 2*b2*l2^2*m2*x4 + g*l2^3*m2^2*sin(3*x3) + 2*Ii2*b2*x4*cos(2*x3) + 2*Ii2*l1^2*m1*x2^2*sin(2*x3) + 2*Ii1*l2^2*m2*x2^2*sin(2*x3) + 2*Ii2*l2^2*m2*x2^2*sin(2*x3) - Ii2*l2^2*m2*x2^2*sin(4*x3) + Ii2*g*l2*m2*sin(3*x3) - 4*L1^2*g*l2*m2^2*sin(x3) + 2*b2*l2^2*m2*x4*cos(2*x3) + 2*L1^2*l2^2*m2^2*x2^2*sin(2*x3) - 2*L1^2*l2^2*m2^2*x4^2*sin(2*x3) - 4*Ii1*g*l2*m2*sin(x3) - 3*Ii2*g*l2*m2*sin(x3) - 4*L1*l2*m2*u*cos(x3) + 2*Ii2*L1^2*m2*x2^2*sin(2*x3) + 2*L1*l2^3*m2^2*x2*x4*sin(x3) + 4*L1*b1*l2*m2*x2*cos(x3) + 2*L1*l2^3*m2^2*x2*x4*sin(3*x3) - 4*g*l1^2*l2*m1*m2*sin(x3) + 2*l1^2*l2^2*m1*m2*x2^2*sin(2*x3) + 2*Ii2*L1*l2*m2*x2*x4*sin(3*x3) + 2*Ii2*L1*l2*m2*x2*x4*sin(x3))/(4*(Ii2^2 - Ii2^2*cos(x3)^2 + Ii1*Ii2 + l2^4*m2^2 - l2^4*m2^2*cos(x3)^2 + L1^2*l2^2*m2^2 + Ii2*L1^2*m2 + Ii2*l1^2*m1 + Ii1*l2^2*m2 + 2*Ii2*l2^2*m2 - 2*Ii2*l2^2*m2*cos(x3)^2 + l1^2*l2^2*m1*m2 - L1^2*l2^2*m2^2*cos(x3)^2))    
];
end