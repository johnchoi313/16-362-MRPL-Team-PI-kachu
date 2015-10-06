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
            index = 1;
            % getting superclass information
            obj = obj@robotTrajectory(n); 
            
            % initialize state variables
            obj.curX = 0.0;
            obj.curY = 0.0;
            obj.curTh = 0.0;
            obj.actX = 0.0
            obj.actY = 0.0
            obj.actTh = 0.0
            
            % initialize arrays
            obj.timeSamples = zeros(1,n);
            obj.xSamples = zeros(1,n);
            obj.ySamples = zeros(1,n);
            obj.thSamples = zeros(1,n); 
            obj.actXSamples = zeros(1,n);
            obj.actYSamples = zeros(1,n);
            obj.actThSamples = zeros(1,n); 
            % initialize encoders
            encR = 0.0;
            encL = 0.0;
            lastEncR = 0.0;
            lastEncL = 0.0; 
            %initialize time
            tic;
            t = toc;
            lastTime = t;
            dt = t - lastTime; 
            
            % run the loop
            while(t < obj.tf)           
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
                obj.xSamples(index) = obj.curX*1000;
                obj.ySamples(index) = obj.curY*1000;
                obj.thSamples(index) = obj.curTh*1000;
                
                %% ------ ACTUAL (real) ------- %%
                %Calculate encoder change (ds)
                dEncR = (encR - lastEncR);
                dEncL = (encL - lastEncL);
                %Calculate actual wheel velocities
                actVR = (dEncR / dt);
                actVL = (dEncL / dt);
                %actV = (actVL + actVR) / 2.0;
                %actw = (actVL - actVR) / 235.0; 
                [actV, actw] = obj.vlvrToVw(obj, actVL, actVR);
                % update pose
                [obj.actX, obj.actY, obj.actTh] = ...
                obj.updateActualPose(obj, t, dt, obj.actX, obj.actY, obj.actTh, actV, actw);      
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
                
                % pause 0.01
                pause(0.01);
            
                % update the encoders
                lastEncR = encR;
                lastEncL = encL;
                encR = (robot.encoders.LatestMessage.Right);
                encL = (robot.encoders.LatestMessage.Left);    
                
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
