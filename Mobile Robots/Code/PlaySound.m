function PlaySound(name)
    %plays a previously loaded sound

    global Sounds

    for i=1:length(Sounds)
        if(Sounds(i).name == name)
            sound(Sounds(i).file, Sounds(i).fs);
            break;
        end
    end   
end

