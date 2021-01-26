function PlotInit()
    %opens a figure and plots the buttons used for the user interface

    figure('units', 'normalized', 'outerposition', [0 0 1 1]);
    hold on;

    xlim([-2 20]);
    ylim([-2 20]);

    button1 = uicontrol('Position', [10 70 100 250], 'String', 'Exit', 'Callback', @ExitProg);
    posb1 = getpixelposition(button1);
    button2 = uicontrol('Position', posb1 + [110 0 0 0], 'String', 'Pause/Resume', 'Callback', @PauseProg);
end

%% User interactions

function ExitProg(src, event)
    global sp
    global exit
    pioneer_set_controls(sp, 0, 0);
    pioneer_close(sp)
    exit = 1;
end

function PauseProg(src, event)
    global sp
    global intV
    global intW
    global RunningFlag
    
    if (RunningFlag == 1)
        pioneer_set_controls(sp, 0, 0);
    else
        pioneer_set_controls(sp, intV, intW);
    end
    PlaySound("Paused");
    RunningFlag = ~RunningFlag;
end