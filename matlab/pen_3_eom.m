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
Ff2 = F2;
n2 = N2 + cross(l2v, F2);

Ff1 = F1 + R2.'*Ff2;
n1 = N1 + R2.'*n2 + cross(l1v, F1) + cross(L1v, R2.'*Ff2);

%% EOM
z1 = R1*[0; 0; 1];
% z2 = R1.'*R2.'*z1;
syms T1 T2 % torques
eqn1 = simplify(dot2(n1, z1) + b1*theta_1dot) - T1 == 0;
eqn2 = simplify(dot2(n2, z1) + b2*theta_2dot) - T2*0 == 0; % z1 is correct

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
fx_sym = simplify(fx_sym);

%% function for acceleration with velocity input
% x = [theta1; theta2; theta2dot; w1_cmd_last]
% u = [w1_cmd]
syms u x1 x2 x3 x4 dt real

% root acceleration is known from the commanded velocity profile
e2_vel = subs(e2, [theta_1dot, a1], [u, (u-x4)/dt]);

% solve the unactuated encoder-side EOM for theta2 acceleration
theta2ddot_vel = solve(e2_vel, a2);

f_vel_sym = [u;
             x3;
             subs(theta2ddot_vel, ...
                  [theta_2dot, theta_1, theta_2], ...
                  [x3, x1, x2]);
             0]; % x4 stores the previous command. need to update it to u after each step.
f_vel_sym = simplify(f_vel_sym)
