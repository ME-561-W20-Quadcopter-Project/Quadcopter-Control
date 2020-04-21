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
% X1: x                         X4: x'
% X2: y                         X5: y'
% X3: z                         X6: z'
% X7: Pitch angle (x-axis)      X10: Pitch rate (x-axis)
% X8: Roll angle (y-axis)       X11: Roll rate (y-axis)
% X9: Yaw angle (z-axis)        X12: Yaw rate (z-axis)

% Inputs:                                   Outputs:
% U1: Total Upward Force (along z-axis)     Y1: Position along x axis
% U2: Pitch Torque (about x-axis)           Y2: Position along y axis
% U3: Roll Torque (about y-axis)            Y3: Position along z axis
% U4: Yaw Torque (about z-axis)             Y4: Pitch (about x-axis)
%                                           Y5: Roll (about y-axis)
%                                           Y6: Yaw (about z-axis)

% State Space Source: https://arxiv.org/ftp/arxiv/papers/1908/1908.07401.pdf
% X' = Ax + Bu
% Y = Cx

nStates = 12;
nInputs = 4;
nOutputs = 6;

A = [0  0  0  1  0  0  0  0  0  0  0  0;
     0  0  0  0  1  0  0  0  0  0  0  0;
     0  0  0  0  0  1  0  0  0  0  0  0;
     0  0  0  0  0  0  0  -g 0  0  0  0;
     0  0  0  0  0  0  g  0  0  0  0  0;
     0  0  0  0  0  0  0  0  0  0  0  0;
     0  0  0  0  0  0  0  0  0  1  0  0;
     0  0  0  0  0  0  0  0  0  0  1  0;
     0  0  0  0  0  0  0  0  0  0  0  1;
     0  0  0  0  0  0  0  0  0  0  0  0;
     0  0  0  0  0  0  0  0  0  0  0  0;
     0  0  0  0  0  0  0  0  0  0  0  0];

% Note: In paper, 1/m is in wrong spot
B = [0      0       0       0;
     0      0       0       0;
     0      0       0       0;
     0      0       0       0;
     0      0       0       0;
     1/m    0       0       0;
     0      0       0       0;
     0      0       0       0;
     0      0       0       0;
     0      1/Ix    0       0;
     0      0       1/Iy    0;
     0      0       0       1/Iz];

C = [1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0];

D = zeros(6,4);

continuous = ss(A, B, C, D);
T_s = 0.01;
discrete = c2d(continuous, T_s);

% Check if this works
impulse(discrete, 0:T_s:1);

% We should see that U1 gets us only translation in z, U2 couples Y2 and Y4,
% U3 couples Y1 and Y5, and U4 gets us Y6

%% Define goals
% Goal 1: settle at 1m height <2s
x_0_up = [0, 0, -1, ...
          0, 0, 0, ...
          0, 0, 0, ...
          0, 0, 0]'; %Redefine origin! 

% Goal 2: Stabilize from a 10-degree roll and pitch with <3deg overshoot
x_0_pitchroll = [0, 0, 0, ...
                 0, 0, 0, ...
                 10*(pi/180), 10*(pi/180), 0, ...
                 0, 0, 0]'; %Pitch and roll of 10 degrees (convert to radians)

x_0_roll = [0, 0, 0, ...
            0, 0, 0, ...
            0, 10*(pi/180), 0, ...
            0, 0, 0]'; %Roll of 10 degrees (convert to radians)
   
% Goal 3: Move from position (0,0,0) to within 5 cm of (1,1,1) within 5 seconds.
x_0_trans = [-1, -1, 0, ...
             0, 0, 0, ...
             0, 0, 0, ...
             0, 0, 0]'; %Redefine origin! 

%% Finite-Time Horizon LQR for Goal 1

% Define Q and R for the cost function. Begin with nominal ones for all.
Q = diag([1000, 1000, 1000, ... % x, y, z
          1, 1, 100, ... % x', y', z'
          200, 200, 1, ... % roll, pitch, yaw
          1, 1, 1]);   % roll', pitch', yaw'

R = diag([10, 20, 20, 1]); % upward force, pitch torque, roll torque, yaw torque

% Calculate number of timesteps.
tSpan = 0:T_s:2;
nSteps = length(tSpan);

% Determine gains
[K, P] = LQR_LTI(discrete.A, discrete.B, Q, R, nSteps);
FiniteLQR_Goal_1_K = K;

% Simulate nonlinear model
[x0, y0, z0, xdot0, ydot0, zdot0, phi0, theta0, psi0, phidot0, thetadot0, psidot0] = unpack(x_0_up);
set_param('LQRNonlinearSim', 'StopTime', '2')
simout = sim('LQRNonlinearSim', 'FixedStep', '.01');

state = [simout.x';
         simout.y';
         simout.z' + 1;
         simout.xdot';
         simout.ydot';
         simout.zdot';
         simout.pitch';
         simout.roll';
         simout.yaw';
         simout.dotpitch';
         simout.dotroll';
         simout.dotyaw'];
plot_states(state, tSpan);

%% Infinite-Time Horizon LQR for Goal 1

% Cost matrices
Q = diag([1, 1, 1000, ... % x, y, z
          1, 1, 100, ... % x', y', z'
          1, 1, 1, ... % roll, pitch, yaw
          1, 1, 1]);   % roll', pitch', yaw'

R = diag([10, 20, 20, 1]); % upward force, pitch torque, roll torque, yaw torque

% Calculate number of timesteps
tSpan = 0:T_s:2;
nSteps = length(tSpan);

% Determine Gains
[X, K_const, L, info] = idare(discrete.A, discrete.B, Q, R, [], []);

K = zeros(nInputs, nStates, nSteps);
for i = 1:nSteps
    K(:,:,i) = K_const;
end

% Simulate nonlinear model
[x0, y0, z0, xdot0, ydot0, zdot0, phi0, theta0, psi0, phidot0, thetadot0, psidot0] = unpack(x_0_up);
set_param('LQRNonlinearSim', 'StopTime', '2')
simout = sim('LQRNonlinearSim', 'FixedStep', '.01');

state = [simout.x';
         simout.y';
         simout.z' + 1;
         simout.xdot';
         simout.ydot';
         simout.zdot';
         simout.pitch';
         simout.roll';
         simout.yaw';
         simout.dotpitch';
         simout.dotroll';
         simout.dotyaw'];
plot_states(state, tSpan);

%% Finite-Time Horizon LQR for Goal 2

Q = diag([0, 0, 0, ... % x, y, z
          0, 0, 0, ... % x', y', z'
          1000, 1000, 1, ... % roll, pitch, yaw
          10, 10, 1]);   % roll', pitch', yaw'

R = diag([10, 20, 20, 1]); % upward force, pitch torque, roll torque, yaw torque

% Calculate number of timesteps.
tSpan = 0:T_s:4;
nSteps = length(tSpan);

% Determine gains
[K, P] = LQR_LTI(discrete.A, discrete.B, Q, R, nSteps);

% Simulate nonlinear model
[x0, y0, z0, xdot0, ydot0, zdot0, phi0, theta0, psi0, phidot0, thetadot0, psidot0] = unpack(x_0_pitchroll);
set_param('LQRNonlinearSim', 'StopTime', '4')
simout = sim('LQRNonlinearSim', 'FixedStep', '.01');

state = [simout.x';
         simout.y';
         simout.z' + 1;
         simout.xdot';
         simout.ydot';
         simout.zdot';
         simout.pitch';
         simout.roll';
         simout.yaw';
         simout.dotpitch';
         simout.dotroll';
         simout.dotyaw'];
plot_states(state, tSpan);

%% Infinite-Time Horizon LQR for Goal 2

% Cost matrices
Q = diag([0, 0, 0, ... % x, y, z
          0, 0, 0, ... % x', y', z'
          1000, 1000, 0, ... % roll, pitch, yaw
          10, 10, 0]);   % roll', pitch', yaw'

R = diag([10, 20, 20, 1]); % upward force, pitch torque, roll torque, yaw torque

% Calculate number of timesteps.
tSpan = 0:T_s:4;
nSteps = length(tSpan);

% Determine gains
[X, K, L, info] = idare(discrete.A, discrete.B, Q, R, [], []);

K = zeros(nInputs, nStates, nSteps);
for i = 1:nSteps
    K(:,:,i) = K_const;
end

% Simulate nonlinear model
[x0, y0, z0, xdot0, ydot0, zdot0, phi0, theta0, psi0, phidot0, thetadot0, psidot0] = unpack(x_0_pitchroll);
set_param('LQRNonlinearSim', 'StopTime', '4')
simout = sim('LQRNonlinearSim', 'FixedStep', '.01');

state = [simout.x';
         simout.y';
         simout.z' + 1;
         simout.xdot';
         simout.ydot';
         simout.zdot';
         simout.pitch';
         simout.roll';
         simout.yaw';
         simout.dotpitch';
         simout.dotroll';
         simout.dotyaw'];
plot_states(state, tSpan);

%% Finite-Time Horizon For Goal 3

Q = diag([1000, 1000, 1000, ... % x, y, z
          0, 0, 0, ... % x', y', z'
          1000, 1000, 0, ... % roll, pitch, yaw
          0, 0, 0]);   % roll', pitch', yaw'
      
R = diag([1, 20, 20, 1]); % upward force, pitch torque, roll torque, yaw torque

% Calculate number of timesteps.
tSpan = 0:T_s:5;
nSteps = length(tSpan);

% Determine gains
[K, P] = LQR_LTI(discrete.A, discrete.B, Q, R, nSteps);

% Simulate nonlinear model
[x0, y0, z0, xdot0, ydot0, zdot0, phi0, theta0, psi0, phidot0, thetadot0, psidot0] = unpack(x_0_trans);
set_param('LQRNonlinearSim', 'StopTime', '5')
simout = sim('LQRNonlinearSim', 'FixedStep', '.01');

state = [simout.x';
         simout.y';
         simout.z' + 1;
         simout.xdot';
         simout.ydot';
         simout.zdot';
         simout.pitch';
         simout.roll';
         simout.yaw';
         simout.dotpitch';
         simout.dotroll';
         simout.dotyaw'];
plot_states(state, tSpan);

%% Infinite-Time Horizon For Goal 3

% Cost matrices
Q = diag([1000, 1000, 1000, ... % x, y, z
          10, 10, 10, ... % x', y', z'
          1000, 1000, 1, ... % roll, pitch, yaw
          1, 1, 1]);   % roll', pitch', yaw'

R = diag([10, 20, 20, 1]); % upward force, pitch torque, roll torque, yaw torque

% Calculate number of timesteps.
tSpan = 0:T_s:5;
nSteps = length(tSpan);

% Determine gains
[X, K, L, info] = idare(discrete.A, discrete.B, Q, R, [], []);

K = zeros(nInputs, nStates, nSteps);
for i = 1:nSteps
    K(:,:,i) = K_const;
end

% Simulate nonlinear model
[x0, y0, z0, xdot0, ydot0, zdot0, phi0, theta0, psi0, phidot0, thetadot0, psidot0] = unpack(x_0_trans);
set_param('LQRNonlinearSim', 'StopTime', '5')
simout = sim('LQRNonlinearSim', 'FixedStep', '.01');

state = [simout.x';
         simout.y';
         simout.z' + 1;
         simout.xdot';
         simout.ydot';
         simout.zdot';
         simout.pitch';
         simout.roll';
         simout.yaw';
         simout.dotpitch';
         simout.dotroll';
         simout.dotyaw'];
plot_states(state, tSpan);

%% Helper Functions

function [K, P] = LQR_LTI(A, B, Q, R, nSteps)
    % Set P up
    P = zeros(size(Q, 1), size(Q, 2), nSteps);
    % Initial value of P
    P(:, :, nSteps) = 1/2 * Q;
    % Set K up, initial K is 0, so this is fine.
    K = zeros(length(R), length(Q), nSteps);
        
    for i = nSteps-1:-1:1
        P_ = P(:,:, i+1);
        
        K(:, :, i) = ( 1/2 * R + B' * P_ * B )^(-1) * B' * P_ * A;
        P(:, :, i) = A' * P_ * ( A - B * K(:, :, i) ) + Q * 1/2;
    end
end

function [ulqr, xlqr] = propagate(nInputs, nStates, nSteps, x_0, K, A, B)
    % Set up for propagation
    ulqr = zeros(nInputs, nSteps);
    xlqr = zeros(nStates, nSteps);
    xlqr(:, 1) = x_0; 

    for i = 1:(nSteps -  1)
        ulqr(:,i) = K(:,:,i) * xlqr(:,i);
        xlqr(:,i+1) = (A*xlqr(:, i) - B*ulqr(:, i));
    end
end

function plot_states(xlqr, tSpan)
    figure();
    subplot(2, 1, 1);
    plot(tSpan, xlqr(1, :), '-r', 'LineWidth', 2);
    hold on;
    plot(tSpan, xlqr(2, :), '-g');
    plot(tSpan, xlqr(3, :), '-b');
    plot(tSpan, xlqr(4, :), '--r', 'LineWidth', 2);
    plot(tSpan, xlqr(5, :), '--g');
    plot(tSpan, xlqr(6, :), '--b');
    legend('x', 'y', 'z', 'x`', 'y`', 'z`');
    title("Translations(-) and Velocities (--)");
    xlabel("Time(s)");
    ylabel("Displacement (m)");

    subplot(2, 1, 2);
    plot(tSpan, xlqr(7, :), '-r');
    hold on;
    plot(tSpan, xlqr(8, :), '-g');
    plot(tSpan, xlqr(9, :), '-b');
    plot(tSpan, xlqr(10, :), '--r');
    plot(tSpan, xlqr(11, :), '--g');
    plot(tSpan, xlqr(12, :), '--b');
    legend('Pitch (about x)', 'Roll (about y)', 'Yaw (about z)', 'Pitch Rate', 'Roll Rate', 'Yaw Rate');
    title("Angular Displacements(-) and Velocities(--)");
    xlabel("Time(s)");
    ylabel("Displacement (rad)");
end

function [x0, y0, z0, xdot0, ydot0, zdot0, phi0, theta0, psi0, phidot0, thetadot0, psidot0] = unpack(X0)
x0=X0(1); y0=X0(2); z0=X0(3); xdot0=X0(4); ydot0=X0(5); zdot0=X0(6); phi0=X0(7); theta0=X0(8); psi0=X0(9); phidot0=X0(10); thetadot0=X0(11); psidot0=X0(12);
end
