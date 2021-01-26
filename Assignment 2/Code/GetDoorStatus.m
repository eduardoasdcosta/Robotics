function GetDoorStatus(lidar)
    %find if the door is closed, open or between both status
	global sp
    pioneer_set_controls(sp, 0, 0);
    scan = LidarScan(lidar);
    
    clear temp;
    len = 682;
    
    NumPoints = 150;
    group = 3;
    MaxDistance = 1500;
    MinDistance = 50;   

    i = (len - NumPoints)/2 + 1;
    j = 1;
    k = 0;

    while(i < (len + NumPoints)/2)
        temp(j,:) = sort(scan(i:i+group-1));

        if(temp(j, ceil(group/2)) > MaxDistance || temp(j, ceil(group/2)) < MinDistance)
            k = k + 1;
        end
        i = i + group;
        j = j + 1;
    end

    if(k < 3)
        txt = sprintf("Door Closed");
        title(txt, 'FontSize', 15);
		PlaySound("ClosedDoor");
    elseif(k < 25)
        txt = sprintf("Door Semi-Open");
        title(txt, 'FontSize', 15);
		PlaySound("SemiOpenDoor");
    else
        txt = sprintf("Door Open");
        title(txt, 'FontSize', 15);
		PlaySound("OpenDoor");
    end
end