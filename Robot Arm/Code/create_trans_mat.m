function [trans_mat] = create_trans_mat(distance, trans_axis)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    trans_mat = eye(4);
    
    if trans_axis == 'X'
        
        trans_mat(1,4) = distance;
        
    elseif trans_axis == 'Y'
        
        trans_mat(2,4) = distance;
        
    elseif trans_axis == 'Z'
        
        trans_mat(3,4) = distance;
    
    end

end

