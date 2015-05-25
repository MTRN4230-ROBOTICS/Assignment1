function TAU = irb120_p_controller(robot, T, Q, QD, varargin)
% Basic Proportional controller for an IRB-120, treating each axis independently.
% Author: Ziwei Guo, Zicong He 
% UNSW Mechatronics
% Q and QD are the manipulator joint coordinate and velocity state 
% respectively, and T is the current time. TAU is the output torque.
% Edit desired_q and desired_qd as a function of time for trajectory
% following.

% joint angles
q2 = pi/180.*[-71.4, 70.3, -39.0, 0, 58.7,-71.4];

% simulate the gravity loading(toque against the gravity when there is no load in the pose)
global Torque0; 

desired_q = q2; % Move all joints to position 0.5 rads.
desired_qd = 0*ones(1, 6); % Make joints stationary at above position.
error_q = desired_q - Q;
error_qd = desired_qd - QD;

% controller
% Adjust the proportional constant independently for each axis and design your own controller.
% proportional gain
P = [1150 17000 6000 300 300 300]; 
% derivative gain 
D = [240 1000 500 50 50 50];

% Assume Ka (transconductance) is 1 for the amplifier and Kt (Km) is 0.2.
kt = 0.2*ones(1, 6); 

TAU = Torque0 + kt.*(P.*error_q + D.*error_qd);
% We are ignoring any effects due to the use of a discrete time controller,
% coupling between axes, encoder measurements and much more. 
% This is a really basic controller!

