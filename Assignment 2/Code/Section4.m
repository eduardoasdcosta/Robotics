function [trajx, trajy] = Section4(lidar, stage)
    %trajectory of the 3rd corridor
    
	global WallFlag
	global V
	global Vmax
	global DefaultRange
    global CollisionRange
    global fixAngle
    global rightDistance
    
    switch stage
		case 15
            [Distance] = FixOrientation(lidar, 0);
			
            Rotation(1);
            
            fixAngle = getAngle(180);
			
            [trajx, trajy] = DefineTraj(-cosd(fixAngle) * max(0, 0.3 + (Distance - rightDistance)/1000), -sind(fixAngle) * 0.3);
			
            CollisionRange = DefaultRange;
            WallFlag = 0;      
			V = Vmax;
        
		case 16
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
			
            [trajx, trajy] = DefineTraj(-cosd(fixAngle) * 2.1, -sind(fixAngle) * 2.1);
		
		case 17
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
			
            [trajx, trajy] = DefineTraj(-cosd(fixAngle) * 1.3, -sind(fixAngle) * 1.3);
			
        case 18
            [wallDistance] = FixOrientation(lidar, 0);
            
            fixAngle = getAngle(180);
            
            [trajx, trajy] = DefineTraj(-cosd(fixAngle) * 3.45, -sind(fixAngle) * 3.45 + (wallDistance - rightDistance)/1200);
            
        case 19
            Rotation(1);
            GetDoorStatus(lidar);
            Rotation(-1);
            [wallDistance] = FixOrientation(lidar, 0);
            
            fixAngle = getAngle(180);
            
            [trajx, trajy] = DefineTraj(-cosd(fixAngle) * 2.5, -sind(fixAngle) * 2.5 + (wallDistance - rightDistance)/1200);
            
		case 20
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
            
            wallDistance = getDist(lidar);
			
            [trajx, trajy] = DefineTraj(-cosd(fixAngle) * 1.7, -sind(fixAngle) * 1.7 + (wallDistance - rightDistance)/1200);
			
        case 21
            [wallDistance] = FixOrientation(lidar, 0);
            
            fixAngle = getAngle(180);
            
            [trajx, trajy] = DefineTraj(-cosd(fixAngle) * 1.2, -sind(fixAngle) * 1.2 + (wallDistance - rightDistance)/2000);
            
		case 22
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
            
            wallDistance = getDist(lidar);
            
            [trajx, trajy] = DefineTraj(-cosd(fixAngle) * 2.2, -sind(fixAngle) * 2.2 + (wallDistance - rightDistance)/1200);
			
		case 23
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
            
            [trajx, trajy] = DefineTraj(-cosd(fixAngle) * 1, -sind(fixAngle) * 1);
            
            WallFlag = 1; 
		
        otherwise
    end
end