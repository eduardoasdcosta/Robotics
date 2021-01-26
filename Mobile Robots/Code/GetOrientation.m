function [finalAngle, distance] = GetOrientation(lidar, side)
    %function that gets the angle difference between the pioneer and the
    %wall (considering finalAngle == 0 when they are parallel)

    %side = 0 -> check right side
    %side = 1 -> check left side
        
    clear temp;
    clear temp2;
    clear temp3;
    len = 682;
    angle = (240/len)*(1:1:len) - 120;
    NumPoints = 150;
    group = 5;
    
    scan = LidarScan(lidar);
   
    
    i = floor(len*1/8 + len*side*6/8 - NumPoints/2 + 1);
    j = 1;

    while(i < ceil(len*1/8 + len*side*6/8 + NumPoints/2))
        temp(j,:) = sort(scan(i:i+group-1));
        temp2(j) = sind(angle(i + ceil(group/2))) * temp(j,ceil(group/2));
        temp3(j) = cosd(angle(i + ceil(group/2))) * temp(j,ceil(group/2));
        
        i = i + group;
        j = j + 1;
    end
    
    distance = temp(ceil(length(temp)/2), ceil(group/2));
    
    [~, finalAngle, ~] = regression(temp3, temp2);
    finalAngle = atand(finalAngle);
end