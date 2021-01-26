function Rotation(direction)
    %rotates the pioneer

    % direction = n  -> rotates n*90 degrees 

    global sp

    pioneer_set_controls(sp, 0, 0);
    odom = pioneer_read_odometry();
    ver = odom(3);
    PlaySound("Rotating");
    while(abs(abs(ver) - abs(odom(3))) < 500)
        pioneer_set_controls(sp, 0, direction * 30);
        pause(2.9);
        odom = pioneer_read_odometry();
    end
    pioneer_set_controls(sp, 0, 0);

end

