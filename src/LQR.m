% Clear workspace
clear all; close all; clc;

% Parameters source: https://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf
g = 9.81;   m = 0.468;  Ix = 4.856*10^-3;
Iy = 4.856*10^-3;   Iz = 8.801*10^-3;

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

%Check if this works
impulse(discrete, 0:T_s:1);

%We should see that U1 gets us only translation in z, U2 couples Y2 and Y4,
%U3 couples Y1 and Y5, and U4 gets us Y6

%% Define goals
%Goal 1: settle at 1m height <2s
x_0_up = [0, 0, -1, ...
       0, 0, 0, ...
       0, 0, 0, ...
       0, 0, 0]'; %Redefine origin! 

%Goal 2: Stabilize from a 10-degree roll and pitch with <3deg overshoot
x_0_pitch = [0, 0, 0, ...
       0, 0, 0, ...
       10, 0, 0, ...
       0, 0, 0]'; %Pitch of 10 degrees

x_0_roll = [0, 0, 0, ...
       0, 0, 0, ...
       0, 10, 0, ...
       0, 0, 0]'; %Roll of 10 degrees
   
%Goal 3: Move from position (0,0,0) to within 5 cm of (1,1,1) within 5 seconds.
x_0_trans = [-1, -1, -1, ...
       0, 0, 0, ...
       0, 0, 0, ...
       0, 0, 0]'; %Redefine origin! 
   
%Define Q and R for the cost function. Begin with nominal ones for all.
Q = diag([1000, 1000, 1000, ... % x, y, z
          1, 1, 100, ... % x', y', z'
          200, 200, 1, ... % roll, pitch, yaw
          1, 1, 1]);   % roll', pitch', yaw'

R = diag([10, 20, 20, 1]); % upward force, pitch torque, roll torque, yaw torque
%% Finite-Time Horizon LQR for Goal 1

%Calculate number of timesteps.
tSpan = 0:T_s:2;
nSteps = length(tSpan);

%Determine gains
[K, P] = LQR_LTI(discrete.A, discrete.B, Q, R, nSteps);

%Propagate
[ulqr, xlqr] = propagate(nInputs, nStates, nSteps, x_0_up, K, discrete.A, discrete.B);
%States are relative to origin, so we need to add the reference to the
%state to get global coordinates
xlqr(3,:) = xlqr(3,:) + 1;
%Plot
plot_states(xlqr, tSpan);
zd = diff(xlqr(6,:))./T_s

%% Finite-Time Horizon LQR for Goal 2

%Calculate number of timesteps.
tSpan = 0:T_s:2;
nSteps = length(tSpan);

%Determine gains
[K, P] = LQR_LTI(discrete.A, discrete.B, Q, R, nSteps);


%Pitch Goal
%Propagate
[ulqr, xlqr] = propagate(nInputs, nStates, nSteps, x_0_pitch, K, discrete.A, discrete.B);

%Plot
plot_states(xlqr, tSpan);
yd = diff(xlqr(5,:))./T_s
pd = diff(xlqr(7,:))./T_s
%Propagate
[ulqr, xlqr] = propagate(nInputs, nStates, nSteps, x_0_roll, K, discrete.A, discrete.B);

%Plot
plot_states(xlqr, tSpan);
xd = diff(xlqr(4,:))./T_s
rd = diff(xlqr(8,:))./T_s

%% Finite-Time Horizon For Goal 3

%Calculate number of timesteps.
tSpan = 0:T_s:5;
nSteps = length(tSpan);

%Determine gains
[K, P] = LQR_LTI(discrete.A, discrete.B, Q, R, nSteps);

%Pitch Goal
%Propagate
[ulqr, xlqr] = propagate(nInputs, nStates, nSteps, x_0_trans, K, discrete.A, discrete.B);
xlqr(1:3,:) = xlqr(1:3,:) + 1;

%Plot
plot_states(xlqr, tSpan);

%% Helper Functions

function [K, P] = LQR_LTI(A, B, Q, R, nSteps)
    %Set P up
    P = zeros(size(Q, 1), size(Q, 2), nSteps);
    %Initial value of P
    P(:, :, nSteps) = 1/2 * Q;
    %Set K up, initial K is 0, so this is fine.
    K = zeros(length(R), length(Q), nSteps);
        
    for i = nSteps-1:-1:1
        P_ = P(:,:, i+1);
        
        K(:, :, i) = ( 1/2 * R + B' * P_ * B )^(-1) * B' * P_ * A;
        P(:, :, i) = A' * P_ * ( A - B * K(:, :, i) ) + Q * 1/2;
    end
end

function [ulqr, xlqr] = propagate(nInputs, nStates, nSteps, x_0, K, A, B)
    %Set up for propagation
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
    subplot(1, 2, 1);
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

    subplot(1, 2, 2);
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
    ylabel("Displacement (deg)");
end