function [distance] = FixOrientation(lidar, side)
    %function that makes the pioneer parallel with the corridor
    
    %side = 0 -> check right side
    %side = 1 -> check left side
    
    global sp
    pioneer_set_controls(sp, 0, 0);
    [angle, distance] = GetOrientation(lidar, side);
    
    while(abs(angle) > 1)
        if(angle > 0)
            pioneer_set_controls(sp, 0, 5);
            pause(angle/5);
        elseif(angle < 0)
            pioneer_set_controls(sp, 0, -5);
            pause(-angle/5);
        end
        pioneer_set_controls(sp, 0, 0);
            
        [angle, distance] = GetOrientation(lidar, side);
    end
end

