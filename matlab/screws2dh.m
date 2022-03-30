function [dh, tool] = screws2dh(M,S,type)
% obtain DH parameters from screws and M matrix

% create SE3 object
M = SE3(M);

% get number of joints
[n_joints] = size(S);
n_joints = n_joints(2);

% initiate a set of SE3 transforms representing joints
joints(n_joints+2) = SE3;

% get transforms coordinates from screw vectors related to base frame
for i=1:n_joints
    % define vectors
    w = S(1:3,i);
    v = S(4:6,i);
    
    if type(i) == 1 % revolute joint
        % obtain point q on the screw axis using Plucker class
        pl = Plucker([v; w]);
        q = -pl.closest(joints(i).t);
    else
        % in case of prismatic joints, place it in the previous frame with updated Z axis
        w = v;
        q = joints(i).t;
    end
    
    % define line 1 represented by n-1 joint Z vector
    L1 = zeros(2,3);
    L1(1,:) = transpose(joints(i).t);
    L1(2,:) = transpose(joints(i).t+10*joints(i).a);
    
    % define line 2 represented by screw vector
    L2 = zeros(2,3);
    L2(1,:) = transpose(q);
    L2(2,:) = transpose(q+10*w);
    
    % calculate the nearest distance using common perpendicular and determine intersection points
    [dist, P, Q] = dist2lines(L1,L2);
    P = transpose(P); % intersecting point 1
    Q = transpose(Q); % intersecting point 2
    PQ = Q-P; % vector from P to Q
    
    % place SE3 coordinate frame following DH convention
    if dist ~= 0 % Z axes are skew or parallel
        % normalize PQ - future X axis
        PQ = PQ/norm(PQ);
        
        % obtain o - future Y axis
        o = cross(PQ,w);
        
        % from w (future Z axis) and o create an SE3 object
        joints(i+1) = SE3.oa(o,w);
        
        % place the coordinate to intersection point Q
        joints(i+1).t = Q;
        
    else % Z axes are intersecting or coincident
        if round(dot(joints(i).a,w),6) == 1 % Z axes are coincident 
            joints(i+1) = joints(i);
            
        elseif round(dot(joints(i).a,w),6) == -1 % Z axes are coincident but differ in orientation
            % rotate the new frame around X by pi [rad]
            joints(i+1) = joints(i)*joints(i).Rx(pi);
            
        else % Z axis intersect with an angle
            % obtain n - future X axis as cross product of previous Z axis and new Z axis
            n = cross(joints(i).a,w);
            
            % obtain o - future Y axis and normalize it
            o = cross(n,w);
            o = o/norm(o);
            
            % from w (future Z axis) and o create an SE3 object
            joints(i+1) = SE3.oa(o,w);
            
            % place the coordinate to the intersection point
            joints(i+1).t = Q;
        end
    end
    
    
    
end

% check if the end-effector obeys DH convention
tool = inv(joints(n_joints+1))*M;
[d, th, a, al] = getDH(joints(n_joints+1),M);
ad_transform = transl([a 0 d]);
tool_dh = SE3(SE3.Rz(th).double*ad_transform*SE3.Rx(al).double);

% compare matrices between the last joint and tool frame
if norm(tool-tool_dh) < 0.00001
    joints(n_joints+2) = M;
    tool = SE3;
else
    joints(n_joints+2) = joints(n_joints+1)*tool_dh;
    %     joints(n_joints+2) = joints(n_joints+1);
    tool = inv(joints(n_joints+2))*M;
end

% get DH params from full transforms
dh = [];
for i=1:n_joints+1
    [d, th, a, al] = getDH(joints(i),joints(i+1));
    dh = [dh; a d al th];
end

end



