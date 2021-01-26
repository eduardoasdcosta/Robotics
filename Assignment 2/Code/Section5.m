function [trajx, trajy] = Section5(lidar, stage)
    %trajectory of the 4th corridor
    
	global WallFlag
	global V
	global Vmax
	global DefaultRange
    global CollisionRange
    global fixAngle
    global rightDistance
    global leftDistance
	
    switch stage
		case 24
            distance = getDist(lidar);
            Rotation(1);
			
            [wallDistance] = FixOrientation(lidar, 0);
            
            fixAngle = getAngle(270);
			
            [trajx, trajy] = DefineTraj(sind(fixAngle) * 2.25 - (wallDistance - rightDistance + 100)/2000, -cosd(fixAngle) * (2.25 + (distance - rightDistance)/1000));
			
            CollisionRange = DefaultRange;
            WallFlag = 0;      
			V = Vmax;
			
		case 25	
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
            
            [wallDistance] = FixOrientation(lidar, 1);
            
            fixAngle = getAngle(270);
			
            [trajx, trajy] = DefineTraj(sind(fixAngle) * 2.85 + (wallDistance - leftDistance - 100)/2000, -cosd(fixAngle) * 2.85);
			
		case 26
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
			
            [wallDistance] = FixOrientation(lidar, 1);
            
            fixAngle = getAngle(270);
            
            [trajx, trajy] = DefineTraj(sind(fixAngle) * 1.6 + (wallDistance - leftDistance - 100)/2000, -cosd(fixAngle) * 1.6);
			
		case 27	
            Rotation(1);
            GetDoorStatus(lidar);
            Rotation(-1);
            
            wallDistance = getDist(lidar);
			
            [trajx, trajy] = DefineTraj(sind(fixAngle) * 1.5 - (wallDistance - rightDistance + 100)/2000, -cosd(fixAngle) * 1.5);
			
		case 28
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
			
            [wallDistance] = FixOrientation(lidar, 1);
            
            fixAngle = getAngle(270);
			
            [trajx, trajy] = DefineTraj(sind(fixAngle) * 2.9 + (wallDistance - leftDistance - 100)/2000, -cosd(fixAngle) * 2.9);
			
		case 29
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
			
            [wallDistance] = FixOrientation(lidar, 1);
            
            fixAngle = getAngle(270);
			
            [trajx, trajy] = DefineTraj(sind(fixAngle) * 3.55 + (wallDistance - leftDistance - 100)/2000, -cosd(fixAngle) * 3.55);
			
        otherwise
    end
end