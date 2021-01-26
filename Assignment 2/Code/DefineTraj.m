function [trajx, trajy] = DefineTraj(x, y)
    %receives the destination position and updates the trajectory
    %also prints the trajectory onto the screen

    odom = pioneer_read_odometry();
    pause(0.5);
    
    line = [odom(1)/1000, odom(2)/1000; odom(1)/1000 + x, odom(2)/1000 + y];
    line_t = 1:size(line, 1);
    line_p = 1:((size(line,1)-1)/1):size(line, 1);
    trajx = pchip(line_t, line(:, 1), line_p);
    trajy = pchip(line_t, line(:, 2), line_p);
    plot(line(:,1), line(:,2), 'o', trajx, trajy, '-.');
    hold on;
end

