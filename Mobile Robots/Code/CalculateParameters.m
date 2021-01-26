function [T, e, phi, alpha] = CalculateParameters(odom, T, ref)
    %calculates the parameters for the controller
    
    if(T > 180)
        T = T - 360;
    end
    
    e = sqrt(dot(ref - odom(1:2)/1000, ref - odom(1:2)/1000));
    phi = atan2(ref(2) - odom(2)/1000, ref(1) - odom(1)/1000)*(180/pi);
    alpha = phi - T;
    
    if(alpha > 180)
        alpha = alpha - 360;
    elseif(alpha < -180)
        alpha = alpha + 360;
    end
end