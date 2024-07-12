clear; close all; clc
%% HECTOR OPEN SOURCE SIMULATION SOFTWARE IN MATLAB/SIMULINK%%
% PLEASE READ LICENSE AGREEMENT BEFORE PROCEEDING

%% Run this script before simulation to load parameters

%% Pre-formulate functions
% addpath(genpath('casadi-windows-matlabR2016a-v3.5.1'))
addpath(genpath('\Users\v4pal\Desktop\UNIVERSITA\[02] - MASTER\II ANNO\II SEMESTRE\Field and Service Robotics\Project'));
addpath(genpath('C:\Users\v4pal\Downloads\FSR_bipedal-main\FSR_bipedal-main'));

import casadi.*
% generate global symbolic functions:
global Contact_Jacobian Rotm_foot
[Contact_Jacobian,Rotm_foot]=Formulate_Contact_Jacobian;

%% General (sim world physics)
world_damping = 1e-3;
world_rot_damping = 1e-3;
joint_stiffness = 0;
joint_damping = 0.001;
% elastic stiff ground:
contact_stiffness = 1e5;
contact_damping = 1e3; 
contact_point_radius = 0.001; % contact cloud visual
ankle_stiffness = 0.0;
ankle_damping = 0.0;
% elastic joint hard stop:
limit_stiffness = 1e3;
limit_damping = 1e2;
% ground friction (static/kinematic)
mu_s = 1.0;
mu_k = 1.0;
mu_vth = 0.001; % critical velocity for transition (m/s)

%% Controller global params
global p_foot_w p_foot_pre t_start gaitcycle dt_MPC N gait dt_MPC_vec ...
    acc_t current_height addArm

N = 5000; % place holder for total MPC prediction length
p_foot_w = zeros(6,1); % initial foot position
t_start = 0; % simulation start time

% MPC setup:
gaitcycle = 0.4; % gait period
h = 10; % horizon length
dt_MPC = gaitcycle/10; % MPC sample time
p_foot_pre = zeros(6,N+10); % foot matrix initialization 

% Gait setup:
gait = 1; % Standing = 0; walking = 1; 

%% contact cloud:
% foot contact approximated by point clouds:
npt = 5; % number of contact points per line
contact_cloud = [ [[linspace(-0.06,0.09,npt)]',ones(npt,1)*0, ones(npt,1)*0]; 
                    [[linspace(-0.06,0.09,npt)]',ones(npt,1)*0.01, ones(npt,1)*0]; 
                    [[linspace(-0.06,0.09,npt)]',ones(npt,1)*-0.01, ones(npt,1)*0]];
mark_distance=1;
mark_x=-5:mark_distance:5;
mark_y=-5:mark_distance:5;
mark_cloud_x=repmat(mark_x,length(mark_y),1);
mark_cloud_x=reshape(mark_cloud_x,[length(mark_x)*length(mark_y),1]);
mark_cloud_y=repmat(mark_y',length(mark_x),1);
mark_cloud=[mark_cloud_x,mark_cloud_y,zeros(length(mark_x)*length(mark_y),1)];

%% vairable MPC dt (feature will be implemented soon)
% See function AssignMPCStage(t) %
global fixed_MPC_dt
fixed_MPC_dt = 1; % fixed dt = 1;
dt_MPC_vec = define_dt_MPC(dt_MPC); % vector defining MPC dts
acc_t(:,1) = zeros(length(dt_MPC_vec)+1,1);
for ith = 2:length(dt_MPC_vec)+1
    acc_t(ith,1) = sum(dt_MPC_vec(1:ith-1));
end

%% Initial Condition/Parameter of Robot State/Joints
body_x0 = 0;
body_y0 = 0;
body_z0 = 0.55; % m
current_height = body_z0;
body_v0 = [0 0 0];
body_R = [0 0 0]; % deg.
% hip1 (Rz joint)
hip1_q0 = 0;
hip1_dq0 = 0;
hip_max = 30;
hip_min = -30;
% hip2 (Rx joint)
hip2_q0_L = 0;
hip2_dq0_L = 0;
hip2_q0_R = 0;
hip2_dq0_R = 0;
hip2_max = 18;
hip2_min = -18;
% thigh (Ry joint)
thigh_q0 = 45*1;
thigh_dq0 = 0;
thigh_max = 120;
thigh_min = -120;
% calf (Ry joint)
calf_q0 = -90*1;
calf_dq0 = 0;
knee_max = -15;
knee_min = -160;
% toe/ankle (Ry joint)
toe_q0 = 45*1;
toe_dq0 = 0;
ankle_max = 75;
ankle_min = -75;

% arms:
%initiate addArm
addArm = 1;
% shoulder Roll(Rx joint):
shoulderx_q0 = 0;
shoulderx_min = -15;
shoulderx_max = 90;
% shoulder Pitch(Ry joint):
shouldery_q0 = 0;
shouldery_min = -90;
shouldery_max = 90;
% Elbow (Ry joint):
elbow_q0 = -90;
elbow_min = -150;
elbow_max = -10;

%% grid surface definition (if using uneven terrain)
% make sure to switch to "Mesh Grid Surface" in Hector model
row = 1; col = 5; type = 5;

% start and goal points
q_start = [body_x0; body_y0; body_z0];
q_goal = [4; 0; body_z0];

[x_grid, y_grid, z_heights, map] = Occupancy_Map(row, col, type, q_start, q_goal);


%% Binary MAP

%[path, f, U, q_o, X,Y,f_x,f_y] = Motion_Planner(q_start(1:2), q_goal(1:2), map, 3);
[path, vel, filledMap] = Motion_Planner(q_start(1:2), q_goal(1:2), map, 1);

% Create a time vector, ensuring it is a column vector (100x1 in this case)
time_vector = linspace(1, 5, size(vel, 2))';

% Create the timeseries object
v = timeseries(vel(1:3,:)',vel(4,:)');

%% Fncs
function out = define_dt_MPC(vec) 
global dt_MPC fixed_MPC_dt
    out = []; % define your variable MPC dts here
    for i=1:length(vec)
        out = [out;vec(i)*ones(5,1)];
    end
    if boolean(fixed_MPC_dt)
        out = [out;dt_MPC*ones(5*(1000-i),1)]; % fixed dt_MPC vec
    end
end


