function ComputeControls(K1, K2, K3, e, phi, alpha)
    %computes the linear and angular velocity of the pioneer

    global V
    global W
    global Vmin
    global Wmax
    global intV
    global intW
    
    modV = V*tanh(K1*e);
    
    if(modV < Vmin)
        modV = Vmin;
    end
    if(alpha ~= 0)
        W = Wmax/1000*((1 + K2*(phi/alpha))*(tanh(K1*e/2)/e)*sind(alpha) + K3*tanh(alpha));
    else
        W = 0;
    end
    
    intV = int32(modV*1000);
    intW = int32(W);
end

