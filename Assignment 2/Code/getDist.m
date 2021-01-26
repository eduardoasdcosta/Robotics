function wallDistance = getDist(lidar)
    %gets front distance of the pioneer to the wall/door on the right
    
    scan = LidarScan(lidar);
    
    len = 682;
    i = ceil(len*1/8);
    
    while(scan(i) > 1500 || scan(i) < 100)
        i = i + 1;
    end
    
    wallDistance = scan(i);
end

