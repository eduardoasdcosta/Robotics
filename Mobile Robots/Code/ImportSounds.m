function ImportSounds()
    %imports all the necessary sounds
    
    global Sounds
    
    cd Sons

    [Sounds(1).file, Sounds(1).fs] = audioread("StartTrack.mp3");
    Sounds(1).file = Sounds(1).file * 0.2;
    Sounds(1).name = "StartTrack";

    [Sounds(2).file, Sounds(2).fs] = audioread("Hello.mp3");
    Sounds(2).file = Sounds(2).file * 6;
    Sounds(2).name = "Hello";

    [Sounds(3).file, Sounds(3).fs] = audioread("OpenDoor.mp3");
    Sounds(3).file = Sounds(3).file * 8;
    Sounds(3).name = "OpenDoor";

    [Sounds(4).file, Sounds(4).fs] = audioread("SemiOpenDoor.wav");
    Sounds(4).file = Sounds(4).file * 5;
    Sounds(4).name = "SemiOpenDoor";

    [Sounds(5).file, Sounds(5).fs] = audioread("ClosedDoor.mp3");
    Sounds(5).file = Sounds(5).file * 5;
    Sounds(5).name = "ClosedDoor";

    [Sounds(6).file, Sounds(6).fs] = audioread("Paused.mp3");
    Sounds(6).file = Sounds(6).file * 2;
    Sounds(6).name = "Paused";
    
    [Sounds(7).file, Sounds(7).fs] = audioread("Rotating.mp3");
    Sounds(7).file = Sounds(7).file * 1.5;
    Sounds(7).name = "Rotating";

    [Sounds(8).file, Sounds(8).fs] = audioread("Wall.mp3");
    Sounds(8).file = Sounds(8).file * 2;
    Sounds(8).name = "Wall";

    [Sounds(9).file, Sounds(9).fs] = audioread("EndTrack.mp3");
    Sounds(9).file = Sounds(9).file * 0.4;
    Sounds(9).name = "EndTrack";

    cd ..

end

