% On the Dynamics of the Furuta Pendulum
% https://onlinelibrary.wiley.com/doi/epdf/10.1155/2011/528341
clear
clc

%% vars
syms l1 l2 m1 m2 L1 L2 Ii1 Ii2 theta_1(t) theta_2(t) g b1 b2
theta_1dot = diff(theta_1, t);
theta_2dot = diff(theta_2, t);
theta_1ddot = diff(theta_1dot, t);
theta_2ddot = diff(theta_2dot, t);

l1v = [l1; 0; 0];
l2v = [l2; 0; 0];
L1v = [L1; 0; 0];
L2v = [L2; 0; 0];

I1 = diag([0 Ii1 Ii1]);
I2 = diag([0 Ii2 Ii2]);

%% kinematics
R1 = eul2rotm2(theta_1, 0, 0).';
R2 = eul2rotm2(theta_2, 0, 0).'*[0 0 -1; 0 1 0; 1 0 0];
w1 = [0 0 theta_1dot].';
w1dot = [0 0 theta_1ddot].';

v1 = [0 0 0].';
v1c = v1 + cross(w1, l1v);

w2 = R2*w1 + [0; 0; theta_2dot];
v2 = R2*(cross(w1, L1v));
v2c = v2 + cross(w2, l2v);

%% outward iteration
v1dot = R1*[0; 0; g];
v1cdot = cross(w1dot, l1v) + cross(w1, cross(w1, l1v)) + v1dot;

F1 = m1*v1cdot;
N1 = I1*w1dot + cross(w1, I1*w1);

w2dot = R2*w1dot + cross(R2*w1, [0; 0; theta_2dot]) + [0; 0; theta_2ddot];
v2dot = R2*(cross(w1dot, L1v) + cross(w1, cross(w1, L1v))+v1dot);
v2cdot = cross(w2dot, l2v) + cross(w2, cross(w2, l2v)) + v2dot;
v2cdot = simplify(v2cdot);

F2 = m2*v2cdot;
N2 = I2*w2dot + cross(w2, I2*w2);

%% inward iteration
f2 = F2;
n2 = N2 + cross(l2v, F2);

f1 = F1 + R2.'*f2;
n1 = N1 + R2.'*n2 + cross(l1v, F1) + cross(L1v, R2.'*f2);

%% EOM
z1 = R1*[0; 0; 1];
z2 = R1.'*R2.'*z1;
syms T1 T2
eqn1 = simplify(dot2(n1, z1) + b1*theta_1dot) == T1;
eqn2 = simplify(dot2(n2, z2) + b2*theta_2dot) == T2*0;

syms a1 a2 d1 d2
e1 = subs(eqn1, [diff(theta_1(t), t, t), diff(theta_2(t), t, t)], [a1 a2]);
e2 = subs(eqn2, [diff(theta_1(t), t, t), diff(theta_2(t), t, t)], [a1 a2]);

%% function for acceleration with torque input
% x = [theta1; theta1dot; theta2; theta2dot]
% u = T1, the stepper-motor torque at the root
accel = solve([e1 e2], [a1, a2]);

syms x1 x2 x3 x4 u
fx_sym = [x2;
          subs(accel.a1, ...
               [theta_1dot, theta_2dot, theta_1, theta_2, T1], ...
               [x2, x4, x1, x3, u]);
          x4;
          subs(accel.a2, ...
               [theta_1dot, theta_2dot, theta_1, theta_2, T1], ...
               [x2, x4, x1, x3, u])];
simplify(fx_sym)

% % The final parameter vector is p = [g; l1; l2; m1; m2; Ii1; Ii2; b1; b2].
% % Thus f_accel(x,u,p) returns [theta1dot; theta1ddot; theta2dot; theta2ddot].
% p = [g; l1; l2; m1; m2; Ii1; Ii2; b1; b2];
% f_accel = matlabFunction(fx_sym, 'Vars', {[x1; x2; x3; x4], u, p});

%% function for acceleration with velocity input
%TODO

%% functions
function fx = f(x, u)
% x = [encPos, enVel, stepPos, stepVel]
% u = [stepTau]

% global constants
g = 9.81;

% pendulum constants (root-side)
L1 = 0.14;                   % [m] distance between rotation axes and pendulum
l1 = L1 / 2;
m1 = 0;
I1 = 0;
b1 = 0;
f1 = 0;
I1 = diag(0, 11123.60, 11123.60) / 1e3 / 1e3^2;

% pendulum constants (tip-side)
l2 = 0.18211;                 % [m] for no weight attached
b2 = 0.021421;                % [Nm.s/deg]
% f = 27.692; % [Nm]
f2 = 0;                       % [Nm] remove friction (insignificant)
m = .2;                      % [-] constant for steepness of tanh approximation of friction
I2 = diag(0, 82688.70, 82688.70) / 1e3 / 1e3^2;

% input constants


% f(x) calculation
fx = [x(2);
    -g/l*180/pi*sind(x(1)) - d/l^2*x(2) - f*tanh(x(2)*m)];
end

function R = eul2rotm2(z, y, x)
% ZYX
% Rz = [cos(a(1)) -sin(a(1)) 0; 
%     sin(a(1)) cos(a(1)) 0; 
%     0 0 1];
% Ry = [cos(a(2)) 0 sin(a(2)); 
%     0 1 0;
%     -sin(a(2)) 0 cos(a(2))];
% Rx = [1 0 0;
%     0 cos(a(3)) -sin(a(3)); 
%     0 sin(a(3)) cos(a(3))];
% Rx, Ry, Rz
% R = Rz*Ry*Rx;
c1 = cos(z);
c2 = cos(y);
c3 = cos(x);
s1 = sin(z);
s2 = sin(y);
s3 = sin(x);

R = [c1*c2, c1*s2*s3 - c3*s1, s1*s3 + c1*c3*s2;
     c2*s1, c1*c3 + s1*s2*s3, c3*s1*s2 - c1*s3;
     -s2,   c2*s3,            c2*c3           ];
end

function c = dot2(a, b)
c = sum(a .* b);
end
