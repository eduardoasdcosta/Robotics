function [finish_traj] = IdEvent(lidar)
	%identifies if obstruction is a person or a wall
    %person -> finish_traj == 0 %%%%% wall -> finish_traj == 1

    scan = LidarScan(lidar);
    
    clear temp;
    len = 682;
    NumPoints = 150;
    Threshold = 300;
	group = 5;

    i = (len - NumPoints)/2 + 1;
    j = 1;

    while(i < (len + NumPoints)/2)
        temp(j,:) = sort(scan(i:i+group-1));

        i = i + group;
        j = j + 1;
    end

    temp = sort(temp(:,ceil(group/2)));
    if(temp(1) + Threshold < temp(NumPoints/group))
        finish_traj = 0;
    else
        finish_traj = 1;
    end
end