% Clear workspace
clear all;
close all;
clc;

% Parameters source: https://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf
g = 9.81;
m = 0.468;
Ix = 4.856*10^-3;
Iy = 4.856*10^-3;
Iz = 8.801*10^-3;

% States:
% X1: Position along x axis - x
% X2: Position along y axis - y
% X3: Position along z axis (height) - z
% X4: Velocity along x axis - x'
% X5: Velocity along y axis - y'
% X6: Velocity along z axis - z'
% X7: Roll angle - phi (y-axis)
% X8: Pitch angle - theta (x-axis)
% X9: Yaw angle - psi (z-axis)
% X10: Roll rate - phi' (y-axis)
% X11: Pitch rate - theta' (x-axis)
% X12: Yaw rate - psi' (z-axis)

% Inputs:
% U1: Total Upward Force on the quad rotor along z-axis
% U2: Pitch Torque (about x-axis)
% U3: Roll Torque (about y-axis)
% U4: Yaw Torque (about z-axis)

% Outputs:
% Y1: Position along x axis - x
% Y2: Position along y axis - y
% Y3: Position along z axis (height) - z
% Y4: Velocity along x axis - x'
% Y5: Velocity along y axis - y'
% Y6: Velocity along z axis - z'

% State Space Source: https://arxiv.org/ftp/arxiv/papers/1908/1908.07401.pdf
% X' = Ax + Bu
% y = Cx + Du

A = [0  0  0  1  0  0  0  0  0  0  0  0;...
     0  0  0  0  1  0  0  0  0  0  0  0;...
     0  0  0  0  0  1  0  0  0  0  0  0;...
     0  0  0  0  0  0  0  -g 0  0  0  0;...
     0  0  0  0  0  0  g  0  0  0  0  0;...
     0  0  0  0  0  0  0  0  0  0  0  0;...
     0  0  0  0  0  0  0  0  0  1  0  0;...
     0  0  0  0  0  0  0  0  0  0  1  0;...
     0  0  0  0  0  0  0  0  0  0  0  1;...
     0  0  0  0  0  0  0  0  0  0  0  0;...
     0  0  0  0  0  0  0  0  0  0  0  0;...
     0  0  0  0  0  0  0  0  0  0  0  0];

% Note: In paper, 1/m is in wrong spot
B = [0 0 0 0;...
     0 0 0 0;...
     0 0 0 0;...
     0 0 0 0;...
     0 0 0 0;...
     1/m 0 0 0;...
     0 0 0 0;...
     0 0 0 0;...
     0 0 0 0;...
     0 1/Ix 0 0;...
     0 0 1/Iy 0;...
     0 0 0 1/Iz];

C = [1 0 0 0 0 0 0 0 0 0 0 0;...
     0 1 0 0 0 0 0 0 0 0 0 0;...
     0 0 1 0 0 0 0 0 0 0 0 0;...
     0 0 0 0 0 0 1 0 0 0 0 0;...
     0 0 0 0 0 0 0 1 0 0 0 0;...
     0 0 0 0 0 0 0 0 1 0 0 0];

D = zeros(6,4);

continuous_system = ss(A, B, C, D);
T_s = 0.05;
discrete_system = c2d(continuous_system, T_s);

Q = diag([1, 1, 1, ... % x, y, z
          1, 1, 1, ... % x', y', z'
          1, 1, 1, ... % roll, pitch, yaw
          1, 1, 1]);   % roll', pitch', yaw'

R = diag([1e3, 1e3, 1e3, 1e3]); % upward force, pitch torque, roll torque, yaw torque

% u = -Kx
K = dlqr(discrete_system.A, discrete_system.B, Q, R, 0);

num_steps = 10;
X = zeros(12, num_steps);
% Initial Condition
X_t = zeros(12,1);
X(:,1) = [0, 0, 0, ... % x, y, z
          0, 0, 0, ... % x', y', z'
          deg2rad(5), deg2rad(0), deg2rad(0), ... % roll, pitch, yaw
          deg2rad(0), deg2rad(0), deg2rad(0)].';  % roll', pitch', yaw'

for i = 1:num_steps
    X(:,i+1) = (A - B*K) * X(:,i);
end

Y = C * X;

figure(1);
t = 1:num_steps;

subplot(1,2,1);
hold on;
plot(t, Y(1, t), 'r-');
plot(t, Y(2, t), 'b-');
plot(t, Y(3, t), 'g-');
legend('x', 'y', 'z')

subplot(1,2,2);
hold on;
plot(t, Y(4, t), 'r-');
plot(t, Y(5, t), 'b-');
plot(t, Y(6, t), 'g-');
legend('roll (y-axis)', 'pitch (x-axis)', 'yaw (z-axis)')
