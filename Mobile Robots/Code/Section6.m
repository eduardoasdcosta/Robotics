function [trajx, trajy] = Section6(lidar, stage)
    %trajectory that enters the lab
    
    global fixAngle
    global rightDistance
    
    switch stage
        case 30
            [wallDistance] = FixOrientation(lidar, 0);
            
            fixAngle = getAngle(270);
            
            [trajx, trajy] = DefineTraj(sind(fixAngle) * 3.7 - (wallDistance - rightDistance + 100)/1200, -cosd(fixAngle) * 3.7);
            
		case 31
            Rotation(-1);
			
            [trajx, trajy] = DefineTraj(-cosd(fixAngle) * 2.5, -sind(fixAngle) * 2.5); 
            
        case 32
            Rotation(2);
            
            [trajx, trajy] = DefineTraj(0, 0); 
            
        otherwise
    end
end 