function [trajx, trajy] = ChangeTraj(lidar, stage)
    %function that chooses the section of the total trajectory
    
    if(stage < 2)
        [trajx, trajy] = Section1(stage);
    elseif(stage < 8)
        [trajx, trajy] = Section2(lidar, stage);
    elseif(stage < 15)
        [trajx, trajy] = Section3(lidar, stage);
    elseif(stage < 24)
        [trajx, trajy] = Section4(lidar, stage);
    elseif(stage < 30)
        [trajx, trajy] = Section5(lidar, stage);
    elseif(stage < 33)
        [trajx, trajy] = Section6(lidar, stage);
    end