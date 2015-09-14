for i = 1:500
    %get closest object and plot location
    [x y th] = closestObject(robot);
    
    %get total distance
    distance = sqrt(x^2 + y^2); 
    leeway = .1;
    turnLeeway = 0.1;
    velocity = 0;
    turn = 0;
    turnSpeed = .25
    speed = 0.25
    VL = 0;
    VR = 0;
    
    %% closer than ideal, move backwards
    if(0 <= distance) && (distance <= 1-leeway)
        velocity = -distance*2*speed;
    %ideal range, don't move
    elseif(1-leeway < distance) && (distance <= 1+leeway)
        velocity = 0;
    %further than ideal, move forwards 
    elseif(1+leeway < distance) && (distance <= 2-leeway)
        velocity = distance*speed;
    %nothing in view, don't move
    elseif(2 < distance)
        velocity = 0;
    end
    
    turn = -th*turnSpeed;
    if(-turnLeeway < th) && (th < turnLeeway)
      VL = velocity;
      VR = velocity;
    elseif (-pi/2 < th) && (th < turnLeeway)    
      VL = velocity + turn;
      VR = velocity - turn;
    elseif (turnLeeway < th) && (th < pi/2)    
      VL = velocity + turn;
      VR = velocity - turn;
    end    
    
    %% Cap Velocity
    if(VL > 0.3)
        VL = 0.3;
    elseif(VL < -0.3)
        VL = -0.3;
    end
    
    if(VR > 0.3)
        VR = 0.3;
    elseif(VR < -0.3)
        VR = -0.3;
    end
    

    %% Send Velocity Commands
    robot.sendVelocity(double(VL), double(VR));
    pause(0.1);
    
end
    velocity = 0;
    robot.sendVelocity(velocity, velocity);
