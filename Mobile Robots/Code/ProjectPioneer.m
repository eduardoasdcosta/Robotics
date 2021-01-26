clear;
clc;
close all;
instrreset;
pause('on');

%% Setups

%Lidar setup
SetupLidar();

%Sound effects setup
ImportSounds();

%System description
K1 = 2;
K2 = 10;
K3 = 40;
epsilon = 0.1;

%Initial State
stage = 0;
FinishTraj = 0;

global exit
global V
global W
global Vmax
global Vmin
global Wmax
global prevSonar
global DefaultRange
global CollisionRange
global RunningFlag
global SoundFlag
global ContinueFlag
global WallFlag
global fixAngle
global leftDistance
global rightDistance
global intV
global intW
exit = 0;
Vmax = 0.2;
Vmin = 0.05;
V = Vmax;
Wmax = 60;
prevSonar = [0 0 0 0 0 0 0 0];
DefaultRange = 500;
CollisionRange = DefaultRange;
RunningFlag = 0;
SoundFlag = 1;
ContinueFlag = 0;
WallFlag = 0;
fixAngle = 0;
leftDistance = 1000;
rightDistance = 700;
intV = 0;
intW = 0;

% Pioneer setup
global sp;
sp = serial_port_start("COM5");
pioneer_init(sp);

PlotInit();
pause(2);

%get initial trajetory
[trajx, trajy] = ChangeTraj(lidar, stage);
ref = [trajx(2) trajy(2)];
stage = stage + 1;

%read initial odometry
odom = pioneer_read_odometry();
sonars = pioneer_read_sonars();
P = odom(1:2)/1000;
T = odom(3)*(360/4096);
W = 0;

%% Execution

PlaySound("StartTrack");

while 1
    
    if (exit == 1) 
        close all;
        break;
    end
    
    odom = pioneer_read_odometry();
    sonars = pioneer_read_sonars();
    T = odom(3)*(360/4098);
    
    [T, e, phi, alpha] = CalculateParameters(odom, T, ref);
    
    PlotTrajectory(stage, e, sonars, odom);
    
    FinishTraj = CheckCollision(sonars, lidar, FinishTraj);
    
    if(ContinueFlag == 1 && RunningFlag == 1)
        ContinueFlag = 0;
        continue;
    end
    
    if(SoundFlag == 0 && RunningFlag == 1)
        SoundFlag = 1;
    end  
    
    if(RunningFlag == 1 && (e < epsilon || FinishTraj == 1))
        FinishTraj = 0;
        [trajx, trajy] = ChangeTraj(lidar, stage);
        stage = stage + 1;
        if(stage == 33)
            PlaySound("EndTrack");
            break;
        end
        ref = [trajx(2) trajy(2)];
        continue;
    end
    
    ComputeControls(K1, K2, K3, e, phi, alpha);
    
    if (intV <= (Vmax * 1000) && abs(intW) < Wmax && RunningFlag == 1)
        pioneer_set_controls(sp, intV, intW);
    end
    
    pause(0.1);    
end

pioneer_set_controls(sp, 0, 0);
pioneer_close(sp);

