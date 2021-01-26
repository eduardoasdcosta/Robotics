function PlotTrajectory(stage, e, sonars, odom)
    %plots the current position of the pioneer (according to the odometry)
    %and prints some useful variables that represent the state of the robot

    global RunningFlag
    global SoundFlag
    
    txt = sprintf("Stage %d at Distance %f. Sonars read %d and %d. Running = %d Blocked = %d\n", stage, e, sonars(4), sonars(5), RunningFlag, ~SoundFlag);
    title(txt, 'FontSize', 15);
    
    hold on;
    plot(odom(1)/1000, odom(2)/1000, 'x');
end

