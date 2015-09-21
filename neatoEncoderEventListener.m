function neatoEncoderEventListener(obj,msg)
    %% GLOBALS %%
    %global initTime;
    %initTime = tic;
    global currTime;
    global encL;
    global encR;
    global V;
    global T;
    global INITIAL_TIME;
   
    %% Update Previous Time Stamp
    prevTime = currTime;
    
    %% Get Current Time Stamp
    %currTime = toc; %- initTime; 
    currTime = double(obj.LatestMessage.Header.Stamp.Sec) ...
        + double(obj.LatestMessage.Header.Stamp.Nsec) / 1000000000.0;
    currTime = currTime - INITIAL_TIME;
    
    
    %% Update Previous Encoder Values
    prevEncL = encL;
    prevEncR = encR;
    
    %% Get Current Encoder Values
    encL = double(msg.Left);
    encR = double(msg.Right);
    
    %Check if Left Encoder Changed
    if(encL ~= prevEncL)
        %Calculate Ds (Encoder Distance)
        dsLeft = encL - prevEncL;
        %Calculate dT
        dt = currTime - prevTime;
        %Calculate V
        %vL = double(dsLeft) / double(dt);
        vL = dsLeft / dt;

        
        %% CULL OUT SPIKES
        %if(vL < 200)
            %% For Plotting
            V = [V vL];
            T = [T currTime];
            %Plot
            %figure(1);
            %plot(currTime,vL), title('Robot Velocity vs Time'), xlabel('Time'), ylabel('Velocity');
            %plot(T, V), title('Robot Velocity vs Time'), xlabel('Time'), ylabel('Velocity');
            %line(currTime,vL);
            %drawnow;
        %end
        
    end
        
    %Check if right encoder changed
    if(encR ~= prevEncL)
    end
    
end

