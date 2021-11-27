function [d, th, a, al] = getDH(Li1,Li)
% Obtain standard DH parameters between 2 links.
% Li1 - Link i-1 
% Li - Link i 

% parameter for visualization (lenght of the line)
plotParam = 1;

%% th offset - not used
th = atan2(norm(cross(Li1.n, Li.n)), dot(Li1.n, Li.n));

% using right-hand rule, if a dot product of Xi axis is in negative
% direction of Yi-1 axis, the angle has to be negative
if dot(Li1.o, Li.n) < 0
    th = -th;
end

%% d
L1 = zeros(2,3);
L1(1,:) = transpose(Li1.t);
L1(2,:) = transpose(Li1.t + plotParam*Li1.n);

L2 = zeros(2,3);
L2(1,:) = transpose(Li.t);
L2(2,:) = transpose(Li.t + plotParam*Li.n);

[d, P, Q] = dist2lines(L1,L2);
% using right-hand rule, if the dot product of vectors between two links 
% and z axis is less than 0 it has to be different direction
if dot(Li.t - Li1.t, Li1.a) < 0
    d = -d;
end


%% a
L1 = zeros(2,3);
L1(1,:) = transpose(Li1.t);
L1(2,:) = transpose(Li1.t + plotParam*Li1.a);

L2 = zeros(2,3);
L2(1,:) = transpose(Li.t);
L2(2,:) = transpose(Li.t + plotParam*Li.a);

[a, P, Q] = dist2lines(L1,L2);
% using right-hand rule, if the dot product of vectors between two links 
% and x axis is less than 0 it has to be different direction
if dot(Li.t - Li1.t, Li.n) < 0
    a = -a;
end

%% al
al = atan2(norm(cross(Li1.a, Li.a)), dot(Li1.a, Li.a));

% using right-hand rule, if a dot product of Zi-1 axis is in negative
% direction of Yi axis, the angle has to be negative
if dot(Li.o, Li1.a) < 0
    al = -al;
end

%% check if DH do exist
transformZ = SE3();
transformZ.t = [0; 0; d];

transformX = SE3();
transformX.t = [a; 0; 0];

T = Li1*Li1.Rz(th)*transformZ*transformX*Li1.Rx(al);

end

