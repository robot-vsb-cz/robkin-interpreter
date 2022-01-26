function [urdf] = rpyxyz2urdf(rpyxyz,type,name)
% create a rpyxyz table from given DH parameters and generate its URDF

% get number of joints
n_joints = size(rpyxyz,1)-2;

% table of RPY XYZ parameters
base_rpyxyz = rpyxyz(1,:);
j_rpyxyz = rpyxyz(2:n_joints+1,:);
ee_rpyxyz = rpyxyz(n_joints+2,:);

urdf = [base_rpyxyz; j_rpyxyz; ee_rpyxyz];


%% create a urdf file
fileID = fopen(name,'w');

% write header
fprintf(fileID, '<?xml version="1.0" ?>\n<robot name="robot_dh">\n\n');

% JOINTS
fprintf(fileID, '<!-- KINEMATICS (joints) -->\n');

% write base
fprintf(fileID, '  <joint name="world_joint" type="fixed">\n');
fprintf(fileID, '    <parent link="world"/>\n');
fprintf(fileID, '    <child link="base_link"/>\n');
fprintf(fileID, '    <origin rpy="%f %f %f" xyz="%f %f %f"/>\n', base_rpyxyz);
fprintf(fileID, '  </joint>\n');

% 1st joint
if type(1) == 1
    fprintf(fileID, '  <joint name="joint1" type="continuous">\n');
else
    fprintf(fileID, '  <joint name="joint1" type="prismatic">\n');
    fprintf(fileID, '    <limit effort="10" velocity="1.0" lower="0.0" upper="1.0"/>\n'); 
end
fprintf(fileID, '    <parent link="base_link"/>\n');
fprintf(fileID, '    <child link="link1"/>\n');
fprintf(fileID, '    <origin rpy="%f %f %f" xyz="%f %f %f"/>\n', j_rpyxyz(1,:));
fprintf(fileID, '    <axis xyz="0 0 1"/>\n');
fprintf(fileID, '  </joint>\n');

for i=2:n_joints
    if type(i) == 1
        fprintf(fileID, '  <joint name="joint%d" type="continuous">\n', i);
    else
        fprintf(fileID, '  <joint name="joint%d" type="prismatic">\n', i);
        fprintf(fileID, '    <limit effort="10" velocity="1.0" lower="0.0" upper="1.0"/>\n');
    end
    fprintf(fileID, '    <parent link="link%d"/>\n', i-1);
    fprintf(fileID, '    <child link="link%d"/>\n', i);
    fprintf(fileID, '    <origin rpy="%f %f %f" xyz="%f %f %f"/>\n', j_rpyxyz(i,:));
    fprintf(fileID, '    <axis xyz="0 0 1"/>\n');
    fprintf(fileID, '  </joint>\n');
end

% end effector
fprintf(fileID, '  <joint name="ee_joint" type="fixed">\n');
fprintf(fileID, '    <parent link="link%d"/>\n', n_joints);
fprintf(fileID, '    <child link="ee_link"/>\n');
fprintf(fileID, '    <origin rpy="%f %f %f" xyz="%f %f %f"/>\n', ee_rpyxyz);
fprintf(fileID, '  </joint>\n');

% LINKS
fprintf(fileID, '\n<!-- DYNAMICS (links) -->\n');

% write world and base links
fprintf(fileID, '  <link name="world"/>\n');
fprintf(fileID, '  <link name="base_link"/>\n');

% write links
for i=1:n_joints
    fprintf(fileID, '  <link name="link%d"/>\n', i);
end

% end effector link
fprintf(fileID, '  <link name="ee_link"/>\n');

% close document
fprintf(fileID, '\n</robot>\n');
fclose(fileID);

end
