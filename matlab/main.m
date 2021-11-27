%% ROBOT KINEMATICS INTERPRETER examples
% Author: Daniel Huczala et al.
% Department of Robotics, Faculty of Mechanical Engineering
% VSB-Technical University of Ostrava, Czech Republic

% Link to repository:
% Licence: GPL-3.0

% This repository provides calculations and converts robot kinematic representations between each other
% If DH or screws are converted to RPYXYZ, corresponding URDF file is generated automatically

clear all
clc
format compact

% DEPENDENCIES 
% All dependecies are part of the project

% 1) Spatial Math Toolbox by Peter Corke: https://github.com/petercorke/spatialmath-matlab
% If you have installed Spatial Math Toolbox by yourself, comment the line bellow
addpath('dependencies/spatialmath-matlab')  

% 2) [WITH CHANGES] Distance between two lines by Alexander Brodsky: https://www.mathworks.com/matlabcentral/fileexchange/29130-shortest-distance-between-two-lines-in-n-dimensions
% Do not uncomment the following line, this file has been changed during work on this project
addpath('dependencies')

%% RRPR robot
% joint type - revolute = 1; prismatic = 0
type = [1 1 0 1];

% dh=[a   d   alpha theta]
dh = [0   0   0     0; 
      0   0.2 -pi/2 0; 
      0.3 0   0     -pi/2; 
      0.2 0   pi    pi/2; 
      0.1 0   0     0];

% base and tool transformation matrices
identity = [1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];
base = SE3(identity);
tool = SE3(identity);

disp('Input DH parameters of RRPR robot');
dh

disp('Output Screw representation:');
[M, S] = dh2screws(dh, tool, type)

disp('Output RPYXYZ representation, generation of robot.urdf file:');
rpyxyz = dh2rpyxyz(dh, type, base, tool, 'robot.urdf')

disp('Output DH representation obtained from Screw representation:');
[new_dh, tool] = screws2dh(M, S, type)

% disp('Output DH representation obtained from RPYXYZ:');
% [new_dh2, tool2] = rpyxyz2dh(rpyxyz, type)
% 
% disp('Output Screw representation obtained from RPYXYZ:');
% [M2, S2] = rpyxyz2screws(rpyxyz, type)
% 
% disp('Output RPYXYZ representation obtained from Screws, generation of robot2.urdf file::');
% rpyxyz2 = screws2rpyxyz(M, S, type, base, 'robot2.urdf')

%% RRR robot
% % joint type - revolute = 1; prismatic = 0
% type = [1 1 1];
% 
% M = [0.8260654  -0.0732850 -0.5587891 0.05;
%      -0.3732283 -0.8140570 -0.4449852 -0.4;
%      -0.4222754 0.5761428  -0.6998164 0.4;
%      0 0 0 1];
%  
% S = [0.6    -0.9129 0.8018; 
%      -0.8   0.3651  -0.5345; 
%      0      0.1826  -0.2673; 
%      0.0640 0.0833  0.3286;
%      0.0480 -0.1467 0.3143;
%      -0.1   -0.1233 0.2571];
% 
% % base specification
% identity = [1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];
% base = SE3(identity);
% 
% disp('Input Screw parameters of RRR robot');
% M, S
% 
% disp('Output RPYXYZ representation, generation of robot.urdf file:');
% rpyxyz = screws2rpyxyz(M, S, type, base, 'robot.urdf')
% 
% disp('Output DH representation obtained from Screw representation:');
% [dh, tool] = screws2dh(M, S, type)


%% UR5 DH-standard
% % from: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
% 
% % joint type - revolute = 1; prismatic = 0
% type = [1 1 1 1 1 1];
% 
% % base and tool transformation matrices
% identity = [1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];
% base = SE3(identity);
% tool = SE3(identity);
% 
% % dh=[a       d      alpha  theta]
% dh = [0       0      0      0;
%       0       0.089  pi/2   0;
%       -0.425  0      0      0;
%       -0.392  0      0      0;
%       0       0.109  pi/2   0;
%       0       0.095  -pi/2  0;
%       0       0.082  0      0];
% 
% disp('Output Screw representation:');
% [M, S] = dh2screws(dh, tool, type)
