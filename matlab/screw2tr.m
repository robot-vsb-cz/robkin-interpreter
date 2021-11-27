function [T] = screw2tr(screw,tr_prev,type)
% put a transformation frame on a screw axis

% define vectors
w = screw(1:3,:);
v = screw(4:6,:);

if type == 1 % revolute joint
    % obtain point q on the screw axis using Plucker class
    pl = Plucker([v; w]);
    q = -pl.closest(tr_prev.t);
else % prismatic joint
    % in case of prismatic joints, place it in the previous frame with
    % updated Z axis
    w = v;
    v = cross(tr_prev.n,w);
    q = tr_prev.t;
end

if norm(v) ~= 0
    % get a unit vector representing Y axis
    v = v/norm(v);
else
    % if the screw axis is coincident with base Z axis, define joint Y 
    % axis coincident with base Y axes
    v = [0; 1; 0];
end

% create transform
T = SE3.oa(v, w);

% place the frame on the nearest point of Plucker object
T.t = q;

end

