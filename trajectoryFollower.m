%% robotModel > figure8ReferenceControl > robotTrajectory > trajectoryFollower
classdef trajectoryFollower < robotTrajectory

    properties
        %% ------ SIMULATED (theoretical) ------- %%
        % current state variables (simulation)
        curX;
        curY;
        curTh;
        % arrays for plotting
        timeSamples;  
        xSamples;
        ySamples;
        thSamples;
        
        %% ------ ACTUAL (real) ------- %%
        % actual state variables (real)
        actX;
        actY;
        actTh;
        % actual arrays for plotting
        actXSamples;
        actYSamples;
        actThSamples;
    end
    
    methods(Static = true)
        % this function makes the robot follow the figure 8
        function obj = trajectoryFollower(robot)
            % numSamples
            n = 250; 
            index = 0;
            % getting superclass information
            obj = obj@robotTrajectory(n); 
            
            % initialize state variables
            obj.curX = 0.0;
            obj.curY = 0.0;
            obj.curTh = 0.0;
            obj.actX = 0.0;
            obj.actY = 0.0;
            obj.actTh = 0.0;
            
            % initialize arrays
            obj.timeSamples = zeros(1,n);
            obj.xSamples = zeros(1,n);
            obj.ySamples = zeros(1,n);
            obj.thSamples = zeros(1,n); 
            obj.actXSamples = zeros(1,n);
            obj.actYSamples = zeros(1,n);
            obj.actThSamples = zeros(1,n); 
            % initialize encoders
            %encR = 0.0;
            %encL = 0.0;
            encR = double(robot.encoders.LatestMessage.Right)*0.001;
            encL = double(robot.encoders.LatestMessage.Left)*0.001;    
            
            lastEncR = 0.0;
            lastEncL = 0.0;
            %lastEncR = double(robot.encoders.LatestMessage.Right)*0.001;
            %lastEncL = double(robot.encoders.LatestMessage.Left)*0.001;    
                
            %initialize time
            tic;
            t = toc;
            lastTime = t;
            dt = 0.01;
            % run the loop
            while(t < obj.tf)
                
                if(index > 0)
                    %% ------ SIMULATED (theoretical) ------- %%
                    % get linear and angular velocity
                    [V, w] = obj.computeControl(obj, t);
                    % convert to wheel velocities
                    [vl, vr] = obj.VwTovlvr(obj, V, w);
                    % cap wheel speeds
                    [vl, vr] = obj.capvlvr(obj, vl,vr);
                    % send wheel velocities
                    robot.sendVelocity(vl,vr);   

                    % update pose
                    [obj.curX, obj.curY, obj.curTh] = ...
                    obj.updatePose(obj, t, dt, obj.curX, obj.curY, obj.curTh);
                    %update arrays
                    obj.timeSamples(index) = t;
                    obj.xSamples(index) = obj.curX;
                    obj.ySamples(index) = obj.curY;
                    obj.thSamples(index) = obj.curTh;

                    %% ------ ACTUAL (real) ------- %%
                    %Calculate encoder change (ds)
                    dEncR = (encR - lastEncR);
                    dEncL = (encL - lastEncL);
                    %Calculate actual wheel velocities
                    actVR = (dEncR / dt);
                    actVL = (dEncL / dt);
                    [actV, actw] = obj.vlvrToVw(obj, actVL, actVR);
                    % update pose
                    [obj.actX, obj.actY, obj.actTh] = ...
                    obj.updateActualPose(obj, t, dt, obj.actX, obj.actY, obj.actTh, actV, actw, obj.curTh);      
                    %update arrays (converted mm to m)
                    obj.actXSamples(index) = obj.actX;
                    obj.actYSamples(index) = obj.actY;
                    obj.actThSamples(index) = obj.actTh;                

                    %% ------ Plot ACTUAL vs THEORETICAL ------- %%
                    figure(1);
                    subplot(2,2,1), plot(obj.timeSamples(1:index), obj.xSamples(1:index), obj.timeSamples(1:index), obj.actXSamples(1:index)), title('X vs Time');
                    subplot(2,2,2), plot(obj.timeSamples(1:index), obj.ySamples(1:index), obj.timeSamples(1:index), obj.actYSamples(1:index)), title('Y vs Time');
                    subplot(2,2,3), plot(obj.timeSamples(1:index), obj.thSamples(1:index), obj.timeSamples(1:index), obj.actThSamples(1:index)), title('Th vs Time');
                    subplot(2,2,4), plot(obj.xSamples(1:index), obj.ySamples(1:index), obj.actXSamples(1:index), obj.actYSamples(1:index)), title('X vs Y, actX vs actY');
                    
                    %Plot Error
                    figure(2);
                    subplot(2,1,1), plot(obj.timeSamples(1:index), (obj.xSamples(1:index) - obj.actXSamples(1:index))*100), title('X error');
                    subplot(2,1,2), plot(obj.timeSamples(1:index), (obj.ySamples(1:index) - obj.actYSamples(1:index))*100), title('Y error');
                    
                end
                    
                % pause 0.01
                pause(0.01);
            
                % update the encoders
                lastEncR = encR;
                lastEncL = encL;
                encR = double(robot.encoders.LatestMessage.Right)*0.001;
                encL = double(robot.encoders.LatestMessage.Left)*0.001;    
                
                % update the last time
                lastTime = t;
                % update current frame's time 
                t = toc; 
                %compute dt
                dt = t - lastTime; 
                % increment the index
                index = index + 1;
            end
            
            % stop the robot
            robot.sendVelocity(0.0,0.0);
            obj.xSamples;
        end
    end
end
