function neatoEncoderEventListener(obj,msg)
    %globals
    %global initTime;
    %initTime = tic;
    global currTime;
    global encL;
    global encR;
    
    %Get time stamp
    prevTime = currTime;
    currTime = toc; %- initTime; %obj.LatestMessage.Header.Stamp.Sec; 
    %+(obj.LatestMessage.Header.Stamp.Nsec/1000000000);
    
    %Get encoder values
    prevEncL = encL;
    prevEncR = encR;
    encL = msg.Left;
    encR = msg.Right;
    
    %Check if left encoder changed
    if(encL ~= prevEncL)
        %Calculate Ds
        dsLeft = encL - prevEncL;
        %Calculate dT
        dt = currTime - prevTime;
        %Calculate V
        vL = double(dsLeft) / double(dt);
        
        % For Plotting
        %V = [V vL];
        %T = [T currTime]; 
        
        %Plot
        figure(1);
        plot(currTime,vL), title('Robot Velocity vs Time'), xlabel('Time'), ylabel('Velocity');

    end
        
    %Check if right encoder changed
    if(encR ~= prevEncL)
    end
    
end

