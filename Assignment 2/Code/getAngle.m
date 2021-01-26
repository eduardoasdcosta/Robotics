function angle = getAngle(orientation)

    odom = pioneer_read_odometry();
    
    angle = odom(3) * (360/4096) - orientation;
end

