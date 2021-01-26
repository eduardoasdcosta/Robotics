function [rot_mat] = create_rot_mat(angle,rot_axis)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    rot_mat = zeros(4);
    c = cosd(angle);
    s = sind(angle);
    
    if rot_axis == 'X'
        
        rot_mat(1,1) = 1;
        rot_mat(2,2) = c;
        rot_mat(2,3) = -s;
        rot_mat(3,2) = s;
        rot_mat(3,3) = c;
        rot_mat(4,4) = 1;
        
    elseif rot_axis == 'Y'
        
        rot_mat(1,1) = c;
        rot_mat(1,3) = s;
        rot_mat(2,2) = 1;
        rot_mat(3,1) = -s;
        rot_mat(3,3) = c;
        rot_mat(4,4) = 1;
       
    elseif rot_axis == 'Z'
        
        rot_mat(1,1) = c;
        rot_mat(1,2) = -s;
        rot_mat(2,1) = s;
        rot_mat(2,2) = c;
        rot_mat(3,3) = 1;
        rot_mat(4,4) = 1;
    end

end

