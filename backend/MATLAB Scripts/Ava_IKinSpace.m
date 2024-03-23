addpath('\Users\sbpen\Desktop\Ava_MATLAB\mr')
clc;
clear;
%% LEFT LEG
% delta_y = 0;
% delta_z = -50;
% 
% Slist = [[0;0;1; 0; 0; 0], ...
%         [0;-1;0; 12.9; 0; 44], ...
%         [1;0;0; 0; 16.4; -18.3], ...
%         [1;0;0; 0; 16.4; -202.2], ...
%         [-1;0;0; 0; -16.4; 433.75]];
% M = [[1, 0, 0, 22.63]; [0, 1, 0, -468.74]; [0, 0, 1, 12.74]; [0, 0, 0, 1]];
% T = [[1, 0, 0, 22.63]; [0, 1, 0, -493.7947-delta_y]; [0, 0, 1, -3.5573-delta_z]; [0, 0, 0, 1]];
% thetalist0 =[deg2rad(0); deg2rad(0); deg2rad(-20); deg2rad(40); deg2rad(20)];

%% RIGHT LEG 
% delta_y = 0;
% delta_z = 50;
% 
% Slist = [[0;0;1;0;0;0], ...
%         [0;-1;0;12.9;0;-44], ...
%         [-1;0;0;0;-16.4;19.3], ...
%         [-1;0;0;0;-16.4;203.2], ...
%         [1;0;0;0;16.4;-434.75]];
% M = [[1, 0, 0, -22.63]; [0, 1, 0, -469.75]; [0, 0, 1, 12.74]; [0, 0, 0, 1]];
% T = [[1, 0, 0, -22.63]; [0, 1, 0, -494.8047-delta_y]; [0, 0, 1, 29.0373-delta_z]; [0, 0, 0, 1]];
% thetalist0 =[deg2rad(0); deg2rad(0); deg2rad(20); deg2rad(-40); deg2rad(-20)];

%% RIGHT ARM
% deltaX = 0;
% deltaY = 0;
% deltaZ = -40;
% Slist = [[-1;0;0;0;0;0], ...
%         [0;0;1;0;-25.2;0], ...
%         [1;0;0;0;10.6;0], ...
%         [0;1;0;-23.53;0;181.98], ...
%         [0;-1;0;29.51;0;-291.73]];
% M = [[1, 0, 0, -343.73]; [0, 1, 0, -8.7]; [0, 0, 1, -36.51]; [0, 0, 0, 1]];
% T = [[0, -1, 0, 33.9000+deltaX]; [0, 0, 1, 79.2373+deltaY]; [-1, 0, 0, 601.4429+deltaZ]; [0, 0, 0, 1]];
% thetalist0 =[deg2rad(-20); deg2rad(90); deg2rad(0); deg2rad(110); deg2rad(0)];

%% LEFT ARM
deltaX = 0;
deltaY = 0;
deltaZ = 40;
Slist = [[1;0;0;0;0;0], ...
        [0;0;1;0;25.2;0], ...
        [-1;0;0;0;-10.8;0], ...
        [0;1;0;-23.53;0;-181.98], ...
        [0;-1;0;29.51;0;291.73]];
M = [[1, 0, 0, 343.73]; [0, 1, 0, -8.2]; [0, 0, 1, -36.51]; [0, 0, 0, 1]];
T = [[0, 11, 0, -33.4000+deltaX]; [0, 0, 1, 79.2373+deltaY]; [1, 0, 0, 601.4429-deltaZ]; [0, 0, 0, 1]];
thetalist0 =[deg2rad(20); deg2rad(-90); deg2rad(0); deg2rad(-110); deg2rad(0)];

eomg = 1;
ev = 0.001;
[thetalist, success] = IKinSpace(Slist, M, T, thetalist0, eomg, ev);
success
thetalist = transpose(rad2deg(thetalist));

% Print the row vector separated by commas
fprintf('%f, ', thetalist(1:end-1)); % Print all elements except the last one
fprintf('%f\n', thetalist(end)); % Print the last element with a newline character