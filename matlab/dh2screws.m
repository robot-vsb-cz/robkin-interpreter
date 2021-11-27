function [M, S] = dh2screws(dh, tool, type)
% create a M matrix and Screw list out of given DH parameters table

% get number of joints
[n_joints] = size(dh);
n_joints = n_joints(1)-1;

% initiate a set of SE3 transforms representing joints
joints(n_joints+2) = SE3;

% partial transforms, following DH convention
for i=2:n_joints+2
    joints(i) = joints(i)*joints(i).Rz(dh(i-1,4));
    joints(i) = joints(i)*SE3(transl(0, 0, dh(i-1,2)));
    joints(i) = joints(i)*SE3(transl(dh(i-1,1), 0, 0));
    joints(i) = joints(i)*joints(i).Rx(dh(i-1,3));
end

% adding tool transform
joints(n_joints+2) = joints(n_joints+2)*tool;

% get full transrofms
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