function [FinishTraj] = CheckCollision(sonars, lidar, FinishTraj)
    %Function responsible of detecting collisions, and identifying the
    %correspondent event (person or wall)

    global sp
    global V
    global Vmin
    global CollisionRange
    global WallFlag
    global RunningFlag
    global SoundFlag
    global ContinueFlag

    if(((sonars(4) < CollisionRange && sonars(4) ~= 0) || (sonars(5) < CollisionRange && sonars(5) ~= 0)) && RunningFlag == 1)
        if(WallFlag == 2)
			V = Vmin;
			WallFlag = 1;
			CollisionRange = CollisionRange - 300;
        elseif(WallFlag == 1)
            pioneer_set_controls(sp, 0, 0);
            FinishTraj = IdEvent(lidar); 
            if(FinishTraj == 0 && SoundFlag == 1)
                PlaySound("Hello");
                SoundFlag = 0;
                ContinueFlag = 1;
            elseif(FinishTraj == 1)
                PlaySound("Wall");
                SoundFlag = 1;
            else
                ContinueFlag = 1;
            end
        else
            pioneer_set_controls(sp, 0, 0);
            if(SoundFlag == 1)
                PlaySound("Hello");
                SoundFlag = 0;
            end
            ContinueFlag = 1;
        end
    end
end

