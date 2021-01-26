function [trajx, trajy] = Section3(lidar, stage)
    %trajectory of the 2nd corridor
    
	global WallFlag
	global V
	global Vmax
	global DefaultRange
    global CollisionRange
	global fixAngle
    global rightDistance
    global leftDistance
	
    switch stage
        case 8
            distance = getDist(lidar);
            Rotation(1);
            
            [wallDistance] = FixOrientation(lidar, 0);
            
            fixAngle = getAngle(90);
            
            [trajx, trajy] = DefineTraj(-sind(fixAngle) * 2.55 + (wallDistance - rightDistance)/1200, cosd(fixAngle) * (2.55 - (distance - rightDistance)/1000));
			
            CollisionRange = DefaultRange;
            WallFlag = 0;      
			V = Vmax;
			
		case 9
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
			
            [trajx, trajy] = DefineTraj(-sind(fixAngle) * 1.6, cosd(fixAngle) * 1.6);
			
        case 10
            [wallDistance] = FixOrientation(lidar, 0);
            
            fixAngle = getAngle(90); 
            
            [trajx, trajy] = DefineTraj(-sind(fixAngle) * 2.75 + (wallDistance - rightDistance)/1200, cosd(fixAngle) * 2.75);
            
		case 11
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
            Rotation(1);
            GetDoorStatus(lidar);
            Rotation(-1);
			
            [trajx, trajy] = DefineTraj(-sind(fixAngle) * 1.59, cosd(fixAngle) * 1.59);
            
		case 12
            Rotation(1);
            GetDoorStatus(lidar);
            Rotation(-1);
            
            [wallDistance] = FixOrientation(lidar, 0);
            
            fixAngle = getAngle(90);
			
            [trajx, trajy] = DefineTraj(-sind(fixAngle) * 1.5 + (wallDistance - rightDistance)/1200, cosd(fixAngle) * 1.5);
			
		case 13
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
            
            [wallDistance] = FixOrientation(lidar, 1);
            
            fixAngle = getAngle(90);
			
            [trajx, trajy] = DefineTraj(-sind(fixAngle) * 1.4 - (wallDistance - leftDistance)/1200, cosd(fixAngle) * 1.4);
			
		case 14
            Rotation(1);
            GetDoorStatus(lidar);
            Rotation(-1);
            
            [wallDistance] = FixOrientation(lidar, 0);
            
            fixAngle = getAngle(90);
			
            [trajx, trajy] = DefineTraj(-sind(fixAngle) * 1.4 + (wallDistance - rightDistance)/1200, cosd(fixAngle) * 3.45);
            
            WallFlag = 2;
			
        otherwise
    end
end