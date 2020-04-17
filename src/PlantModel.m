% Clear Workspace
clear all;
close all;
clc;

% Parameters source: https://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf
g = 9.81;
m = 0.468;
Ix = 4.856E-3;
Iy = 4.856E-3;
Iz = 8.801E-3;

% State Space Source: https://arxiv.org/ftp/arxiv/papers/1908/1908.07401.pdf
% X' = Ax+Bu
% y = Cx+Du

% Inputs:
% U1: Total Upward Force on the quad rotor along z-axis
% U2: Pitch Torque (about x-axis)
% U3: Roll Torque (about y-axis)
% U4: Yaw Torque (about z-axis)
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
sys = ss(A,B,C,D);
sysd = c2d(sys, 1/100);

x0 = 0;
y0 = 0;
z0 = 0;
xdot0 = 0;
ydot0 = 0;
zdot0 = 0;
%phi0 = 5*(pi/180);
phi0 = 0;
%theta0 = 5*(pi/180);
theta0 = 0;
psi0 = 0;
phidot0 = 0;
thetadot0 = 0;
psidot0 = 0;
initialCondition = [x0, y0, z0, xdot0, ydot0, zdot0, phi0, theta0, psi0, phidot0, thetadot0, psidot0];

x_d = 1;
y_d = 1;
z_d = 1;

%%
%Gains
kpp = 0.4754; 
kdp = 0.1; 

kpt = 0.4754;
kdt = 0.1;

kpps = 0.0901;
kdps = 0.0486;

kpz = 4.0670;
kdz = 2.9031;
