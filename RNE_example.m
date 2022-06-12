clc; clear;
%% Exercise 2. RR
% Structure of robot, change the value of g to set gravity on or off. Other
% parameters are described in the report.pdf file.
g = 9.81;
robot = struct('njoints', 2, 'type' , ['R','R'], 'mass', [22,19], 'length', [1, 0.8], 'g', [0; g; 0]);
q = [90, 45];
qdot = [-0.8, 0.35];
q2dot = [-0.4, 0.1];
% Rotation about z-axis of q(1) angle
R01 = [cosd(q(1)) -sind(q(1)) 0;
    sind(q(1)) cosd(q(1)) 0;
    0 0 1];
% Rotation about z-axis of q(2) angle
R12 = [cosd(q(2)) -sind(q(2)) 0;
    sind(q(2)) cosd(q(2)) 0;
    0 0 1];
% Identity
R23 = eye(3,3);

robot.R = cell(3,1);
robot.R{1} = R01;
robot.R{2} = R12;
robot.R{3} = R23;

robot.inertia = cell(robot.njoints,1);
robot.inertia{1} = diag([0 0 0.4]);
robot.inertia{2} = diag([0 0 0.3]);
axis = [1; 0; 0];
disp("Exercise 2");
tau = RNE(robot, q, qdot, q2dot, axis)
%% Exercise 3. RP
g = 9.81;
robot = struct('njoints', 2, 'type' , ['R','P'], 'mass', [10,6], 'length', [1, 0], 'g', [0; g; 0]);
q = [20, 0.2];
qdot = [0.08, 0.03];
q2dot = [0.1, 0.01];
R01_1 = [cosd(q(1)) -sind(q(1)) 0;
    sind(q(1)) cosd(q(1)) 0;
    0 0 1];
R01_2 = [cosd(90) 0 sind(90); 0 1 0; -sind(90) 0 cosd(90)];
R12 = eye(3,3);
R23 = eye(3,3);
robot.R = cell(3,1);
robot.R{1} = R01_1 * R01_2;
robot.R{2} = R12;
robot.R{3} = R23;
robot.inertia = cell(robot.njoints,1);
robot.inertia{1} = diag([0 0 0.4]);
robot.inertia{2} = diag([0.3 0 0]);
axis = [0; 0; 1];
disp("Exercise 3");
tau = RNE(robot, q, qdot, q2dot, axis)


%% Exercise 4. RRR

g = 9.81;
robot = struct('njoints', 3, 'type' , ['R', 'R', 'R'], 'mass', [20, 20, 6], 'length', [1, 0.8, 0.35], 'g', [0;0;g]);
q = [20, 40, 10];
qdot = [0.2, 0.15, -0.2];
q2dot = [0.1, 0.085, 0];
% Rotation around x-axis of 90 degree
R01 = [1 0 0; 0 cosd(90) -sind(90); 0 sind(90) cosd(90)];
% Rotation about z-axis of q(1) angle
Rq1 = [cosd(q(1)) -sind(q(1)) 0;
    sind(q(1)) cosd(q(1)) 0;
    0 0 1];
% Rotation about z-axis of q(2) angle
R12 = [cosd(q(2)) -sind(q(2)) 0;
    sind(q(2)) cosd(q(2)) 0;
    0 0 1];
% Rotation about z-axis of q(3) angle
R23 = [cosd(q(3)) -sind(q(3)) 0;
    sind(q(3)) cosd(q(3)) 0;
    0 0 1];
% Identity
R34 = eye(3);

robot.R = cell(robot.njoints+1,1);
robot.R{1} = Rq1*R01;
robot.R{2} = R12;
robot.R{3} = R23;
robot.R{4} = R34;

robot.inertia = cell(robot.njoints,1);
robot.inertia{1} = diag([0.2 0.2 0.8]);
robot.inertia{2} = diag([0.2 0.2 0.8]);
robot.inertia{3} = diag([0.08 0.08 0.1]);
disp("Exercise 4");
tau = RNE(robot, q, qdot, q2dot)
%% Function implementation
%{
Recursive Newton-Euler algorithm to calculate torques/forces needed for
dynamics of the robot. 
Inputs: 
robot- a data structure of the robot, describes the physical and geometric
parameters
q - vector of joint values
qdot - vector of joint velocities
q2dot - vector of joint coordinates
axis - unit vector used to calculate position vector from origin of frame {i-1} to
{i} projected in frame {i}. 
%}
function tau = RNE(robot, q, qdot, q2dot, axis)
    [r, w, wdot, vdot_ci, r_ci] = forward(robot, q, qdot, q2dot, axis);
    tau = backwards(robot, w, wdot, vdot_ci, r, r_ci);
end
%{
input: [robot - struct that defines a robot; q - vector of angles/position
of the joints; qdot - vector of joint velocities; q2dot - vector of joint
accelerations]
output: r - cell array of position vectors from origin of frame {i-1} to
{i} projected in frame {i}; 
w - cell array of angular velocities of each frame {i} projected in frame
{i}
wdot - cell array of angular accelerations of each frame {i} projected in frame
{i}
vdot_ci - cell array of linear accelerations of each center of mass of link
i projected in frame {i}
r_ci - cell array of position vector from origin of frame {i} to center of
mass of link i
function: does forward propogation to calculate joints' angular/linear
speed/acceleration 
%}
function [r, w, wdot, vdot_ci, r_ci] = forward(robot, q, qdot, q2dot, axis)
    r = cell(robot.njoints, 1); % position vector of link {i} wrt {i-1}
    r_ci = cell(robot.njoints, 1); % position vector of center of masses of link {i} from O{i-1} projected in frame {i}
    w = cell(robot.njoints, 1); % angular velocity vector of frame {i} projected in frame {i}
    wdot = cell(robot.njoints, 1); % angular acceleration vector of frame {i} projected in frame {i}
    vdot = cell(robot.njoints, 1); % linear acceleration vector of frame {i} projected in frame {i}
    vdot_ci = cell(robot.njoints, 1);

    vdot_ci{1} = [0; 0; 0];
    vdot{1} = robot.g; % vdot o/o linear acceleration of {0} frame
    wdot{1} = [0; 0; 0]; % wdot o/o angular acceleration of {0} frame
    w{1} = [0; 0; 0]; % wo/o angular velocity of {0} frame
    r{1} = [0; 0; 0]; % ro/o position vector of {0} frame from {0} frame in homogenous coordinates
    r_ci{1} = [0; 0; 0];
    for i = 2:robot.njoints+1 % starting from 2 because MatLab indexing starts from 1
        if robot.type(i-1) == 'R'
            r{i} = robot.length(i-1) * axis; % projected in {i} frame
            r_ci{i} = -robot.length(i-1)/2 * axis; % projected in {i} frame
            w{i} = transpose(robot.R{i-1})*(w{i-1} + [0; 0; qdot(i-1)]); % projected in {i} frame
            wdot{i} = transpose(robot.R{i-1})*(wdot{i-1} + [0; 0; q2dot(i-1)] + qdot(i-1)*cross(w{i-1},[0;0;1]));
            vdot{i} = transpose(robot.R{i-1})*vdot{i-1} + cross(wdot{i},r{i}) + cross(w{i}, cross(w{i}, r{i}));
            vdot_ci{i} = vdot{i} + + cross(wdot{i}, r_ci{i}) +cross(w{i}, cross(w{i}, r_ci{i}));
        else
            r{i} =  2*q(i-1) * axis; % projected in {i} frame
            r_ci{i} = -q(i-1) * axis;
            w{i} = transpose(robot.R{i-1})*w{i-1}; % projected in {i} frame
            wdot{i} = transpose(robot.R{i-1})*wdot{i-1};
            vdot{i} = transpose(robot.R{i-1})*(vdot{i-1} + [0;0;q2dot(i-1)]) + cross(2*qdot(i-1)*w{i},transpose(robot.R{i-1})*[0;0;1]) + cross(wdot{i},r{i}) + cross(w{i}, cross(w{i}, r{i}));
            vdot_ci{i} = vdot{i} + + cross(wdot{i}, r_ci{i}) +cross(w{i}, cross(w{i}, r_ci{i}));
        end
    end
end
%{
input: [robot - struct that defines a robot; q - vector of angles/position
of the joints; 
w - cell array of angular velocities of each frame {i} projected in frame
{i}; 
vdot_ci - cell array of linear accelerations of each center of mass of link
i projected in frame {i}; 
r - cell array of position vectors from origin of frame {i-1} to
{i} projected in frame {i};
r_ci - cell array of position vector from origin of frame {i} to center of
mass of link i;

output: tau - vector of joint torques

function: computes joints' torques needed to move the robot in given
configuration
%}
function tau = backwards(robot, w, wdot, vdot_ci, r, r_ci)
    tau = zeros(robot.njoints,1);
    F = cell(robot.njoints+1, 1);
    M = cell(robot.njoints+1, 1);
    
    F{robot.njoints+1} = [0;0;0]; % N. Force of joint n (end-effector) projected in frame {n}
    M{robot.njoints+1} = [0;0;0]; % Nm. Moment of joint n (end-effector) projected in frame {n}
    for i = robot.njoints:-1:1
        F{i} = robot.R{i+1}*F{i+1} + robot.mass(i)*vdot_ci{i+1};
        M{i} = robot.R{i+1}*M{i+1} + cross(robot.R{i+1}*F{i+1}, r_ci{i+1}) + cross(r{i+1} + r_ci{i+1}, F{i}) + robot.inertia{i}*wdot{i+1} + cross(w{i+1}, robot.inertia{i}*w{i+1}); 
        if robot.type(i) == 'R'
            tau(i) = transpose(M{i})*transpose(robot.R{i})*[0;0;1];
        else
            tau(i) = transpose(F{i})*transpose(robot.R{i})*[0;0;1];
        end
    end
end