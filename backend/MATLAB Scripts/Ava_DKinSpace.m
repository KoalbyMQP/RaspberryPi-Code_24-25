
%% THIS IS A WORK IN PROGRESS, DO NOT RELY

scriptpath = fileparts(mfilename('fullpath'));
addpath(scriptpath)
addpath('mr')
clc;
clear;

deltaX = 0;
deltaY = 20;
deltaZ = 100;

%% LEFT LEG
Slist = [[0;0;1; 468.7500; 22.6300; 0], ...
        [0;-1;0; -25.6400; 0; -21.3700], ...
        [1;0;0; 0; -29.1400; -450.4500], ...
        [1;0;0; 0; -29.1400; -266.5500], ...
        [-1;0;0; 0; 29.1400; 35]];

% Step One: Inverse Kinematics
M = [[1, 0, 0, 22.63]; [0, 1, 0, -468.74]; [0, 0, 1, 12.74]; [0, 0, 0, 1]];
T = [1.000000, 0.000000, 0.000000, 22.630000 + deltaX;
    0.000000, 1.000000, 0.000000, -443.685299 + deltaY;
    0.000000, 0.000000, 1.000000, -3.557260 + deltaZ;
    0.000000, 0.000000, 0.000000, 1.000000];
thetalist0 =[deg2rad(0); deg2rad(0); deg2rad(-20); deg2rad(40); deg2rad(20)];

eomg = 1;
ev = 0.001;
[thetalist, success] = IKinBody(Slist, M, T, thetalist0, eomg, ev);

%% Step Two: Forward Kinematics
T01 = FKinBody(M, Slist(:,1:1), thetalist(1));
T02 = FKinBody(M, Slist(:,1:2), thetalist(2));
T03 = FKinBody(M, Slist(:,1:3), thetalist(3));
T04 = FKinBody(M, Slist(:,1:4), thetalist(4));

%% Step Three: Adjoint Transformation
Ad1 = Adjoint(T01);
Ad2 = Adjoint(T01*T02);
Ad3 = Adjoint(T01*T02*T03);
Ad4 = Adjoint(T01*T02*T03*T04);

%% Step Four: Create Jacobian
J = [Slist(:,1) Ad1*Slist(:,2) Ad2*Slist(:,3) Ad3*Slist(:,4) Ad4*Slist(:,5)];
% Step Five: Solve DK
% qDot = [0; 0; 0.1; -0.2; -0.1];
% pDot = J * qDot

%       
pDot = [0;0;0.5;0;0;0];
qDot = pinv(J) * pDot

% Print the row vector separated by commas
fprintf('%f, ', qDot(1:end-1)); % Print all elements except the last one
fprintf('%f\n', qDot(end)); % Print the last element with a newline character
