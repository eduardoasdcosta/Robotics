function [trajx, trajy] = Section1(stage)
    %trajectory that exits the lab

	global WallFlag
    global fixAngle
	global V
	global Vmax
	global DefaultRange
    global CollisionRange

    switch stage
        case 0
            [trajx, trajy] = DefineTraj(cosd(fixAngle) * 3, sind(fixAngle) * 3);
            
            CollisionRange = DefaultRange;
            WallFlag = 2;      
			V = Vmax;
            
        case 1
            Rotation(1);
            
            [trajx, trajy] = DefineTraj(-sind(fixAngle) * 3.65, cosd(fixAngle) * 3.65);
            
            CollisionRange = DefaultRange;
            WallFlag = 0;      
			V = Vmax;
			
        otherwise
    end
end