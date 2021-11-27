function [M, S, base] = rpyxyz2screws(rpyxyz, type)
% create a M matrix and Screw list out of given RPY XYZ parameters table

% get number of joints
[n_joints] = size(rpyxyz);
n_joints = n_joints(1)-2;

% initiate a set of SE3 transforms representing joints
joints(n_joints+2) = SE3;

% partial transforms
for i=1:n_joints+2
    joints(i) = rpy2tr(rpyxyz(i,1:3));
    joints(i).t = transpose(rpyxyz(i,4:6));
end

% get base frame
base = joints(1);

% full transrofms in robot base
for i=2:n_joints+2
    joints(i) = joints(i-1)*joints(i);
end

% get M matrix
M = joints(n_joints+2);

% get list of screws
S = [];
for i = 2:n_joints+1
    if type(i-1) == 0 % prismatic joint
        screw = [0; 0; 0; joints(i).a];
    else % revolute joint
        screw = [joints(i).a; cross(-joints(i).a, joints(i).t)];
    end
    S = [S screw];
end

end