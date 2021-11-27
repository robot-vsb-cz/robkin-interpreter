% Kinematics visualization
% In case you have a licence to Matlab Robotics System Toolbox, after
% generating a URDF you can visualize it with this script

clear all;

% Be careful to provide according number of joint variables
% For the RRPR robot
robot = importrobot('robot.urdf');
robot.DataFormat = 'column';

joint_variables = [pi*3/4; -pi/4; 0.3; -pi*3/4];
% joint_variables = [0;0;0;0];

show(robot, joint_variables);
axis equal;

