
function [trajx, trajy] = Section2(lidar, stage)
    %trajectory of the 1st corridor

	global WallFlag
	global fixAngle
    global leftDistance
    global rightDistance
    
    switch stage
        case 2
            [Distance] = FixOrientation(lidar, 1);
            Rotation(-1);
            
            fixAngle = 0;
            
            [trajx, trajy] = DefineTraj(cosd(fixAngle) * (2 - (Distance - rightDistance)/1000), sind(fixAngle) * 2);
            
        case 3
            [wallDistance] = FixOrientation(lidar, 0);
            
            fixAngle = getAngle(0);
            
            [trajx, trajy] = DefineTraj(cosd(fixAngle) * 2.8, - (wallDistance - rightDistance)/1200);
            
        case 4
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
            
            [trajx, trajy] = DefineTraj(cosd(fixAngle) * 1.5, sind(fixAngle) * 1.5);
            
        case 5   
            [wallDistance] = FixOrientation(lidar, 0);
            
            fixAngle = getAngle(0);
            
            [trajx, trajy] = DefineTraj(cosd(fixAngle) * 3.77, sind(fixAngle) * 3.77 - (wallDistance - rightDistance)/1200);
            
        case 6
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
            
            [wallDistance] = FixOrientation(lidar, 1);
            
            fixAngle = getAngle(0);
            
            [trajx, trajy] = DefineTraj(cosd(fixAngle) * 4.53, sind(fixAngle) * 4.53 + (wallDistance - leftDistance)/1200);
                  
        case 7
            Rotation(-1);
            GetDoorStatus(lidar);
            Rotation(1);
           
            [trajx, trajy] = DefineTraj(cosd(fixAngle) * 1, sind(fixAngle) * 1);
            
            WallFlag = 1;
            
        otherwise
    end
end