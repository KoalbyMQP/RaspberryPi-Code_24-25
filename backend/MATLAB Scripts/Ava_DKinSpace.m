addpath('\Users\sbpen\Desktop\Ava_MATLAB\mr')

%% LEFT LEG
delta_y = 50;
delta_z = 0;

Slist = [[0; 0; 1; 0; 0; 0], ...
        [0;-1;0; 12.9; 0; 44], ...
        [1;0;0; 0; 16.4; -18.3], ...
        [1;0;0; 0; 16.4; -202.2], ...
        [-1;0;0; 0; -16.4; 433.75]];

% Step One: Inverse Kinematics
M = [[1, 0, 0, 22.63]; [0, 1, 0, -468.74]; [0, 0, 1, 12.74]; [0, 0, 0, 1]];
T = [[1, 0, 0, 22.63]; [0, 1, 0, -493.7947-delta_y]; [0, 0, 1, -3.5573-delta_z]; [0, 0, 0, 1]];
thetalist0 =[deg2rad(0); deg2rad(0); deg2rad(-20); deg2rad(40); deg2rad(20)];

%% RIGHT LEG
% delta_y = 50;
% delta_z = 0;
% 
% Slist = [[0;0;1;0;0;0], ...
%         [0;-1;0;12.9;0;-44], ...
%         [-1;0;0;0;-16.4;19.3], ...
%         [-1;0;0;0;-16.4;203.2], ...
%         [1;0;0;0;16.4;-434.75]];
% M = [[1, 0, 0, -22.63]; [0, 1, 0, -469.75]; [0, 0, 1, 12.74]; [0, 0, 0, 1]];
% T = [[1, 0, 0, -22.63]; [0, 1, 0, -494.8047-delta_y]; [0, 0, 1, 29.0373-delta_z]; [0, 0, 0, 1]];
% thetalist0 =[deg2rad(0); deg2rad(0); deg2rad(20); deg2rad(-40); deg2rad(-20)];

eomg = 1;
ev = 0.001;
[thetalist, success] = IKinSpace(Slist, M, T, thetalist0, eomg, ev);

%% Step Two: Forward Kinematics
T01 = FKinSpace(M, Slist(:,1:1), thetalist(1));
T02 = FKinSpace(M, Slist(:,1:2), thetalist(2));
T03 = FKinSpace(M, Slist(:,1:3), thetalist(3));
T04 = FKinSpace(M, Slist(:,1:4), thetalist(4));

%% Step Three: Adjoint Transformation
Ad1 = Adjoint(T01);
Ad2 = Adjoint(T01*T02);
Ad3 = Adjoint(T01*T02*T03);
Ad4 = Adjoint(T01*T02*T03*T04);

%% Step Four: Create Jacobian
J = [Slist(:,1) Ad1*Slist(:,2) Ad2*Slist(:,3) Ad3*Slist(:,4) Ad4*Slist(:,5)];
% pinv(J)
% Step Five: Solve DK
qDot = [0; 0; 0.1; -0.2; -0.1];
pDot = J * qDot

%       y z x
% pDot = [1;0;0;0;0;0];
% qDot = pinv(J) * pDot

% Print the row vector separated by commas
fprintf('%f, ', pDot(1:end-1)); % Print all elements except the last one
fprintf('%f\n', pDot(end)); % Print the last element with a newline character
