function [dh, tool, base] = rpyxyz2dh(rpyxyz, type)
% get DH parameters from RPY XYZ table

% transfer the RPY XYZ to screws (following DH convention) and then to DH
[M, S, base] = rpyxyz2screws(rpyxyz, type);
[dh, tool] = screws2dh(M, S, type);

end