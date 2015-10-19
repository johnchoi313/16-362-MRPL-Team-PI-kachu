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
        function obj = trajectoryFollower(robot,x,y,th,sgn)
            % numSamples
            n = 202; 
            index = 1;
            
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
            encR = double(robot.encoders.LatestMessage.Right)*0.001;
            encL = double(robot.encoders.LatestMessage.Left)*0.001;    
            lastEncR = 0.0;
            lastEncL = 0.0;
            
            % compute the curve and get the trajectories.
            curve = cubicSpiral([1.0,1.0,1.0],n);
            curve = curve.planTrajectory(x,y,th,sgn);
            curve.planVelocities(0.25);
            %initialize time
            tic;
            t = 0.01
            dt = 0.01;
            lastTime = t;
            tf = curve.getTrajectoryDuration()
            
            % run the loop
            while(t < tf)    
                % send wheel velocities
                vl = curve.getVLAtTime(t); %curve.vlArray(index);
                vr = curve.getVRAtTime(t); %curve.vrArray(index);
                
                V_feedback = 
                w_feedback = 
                
                vl_feedback = function(V_feeback,w_feedback);
                vr_feedback = function(V_feedback,w_feedback);
                
                vl = vl + vl_feedback;
                vr = vr + vr_feedback;
                
                robot.sendVelocity(vl,vr);

                index = index + 1;
         
                 obj.actXSamples(index) = obj.actX;
                 obj.actYSamples(index) = obj.actY;
                 obj.actThSamples(index) = obj.actTh;                

                if (index > 5)
                    %% ------ ACTUAL (real) ------- %%
                    %Calculate encoder change (ds)
                    dEncR = (encR - lastEncR);
                    dEncL = (encL - lastEncL);
                    %Calculate actual wheel velocities
                    actVR = (dEncR / dt);
                    actVL = (dEncL / dt);
                    [actV, actw] = robotModel.vlvrToVw(actVL, actVR);
                    % update pose
                    [obj.actX, obj.actY, obj.actTh] = ...
                    obj.updateActualPose(obj, t, dt, obj.actX, obj.actY, obj.actTh, actV, actw, obj.curTh);      

                    %% ------ Plot ACTUAL vs THEORETICAL ------- %%
                    curve.plot(obj.actXSamples,obj.actYSamples);
                end
                
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
            end
            
            %Stop the robot
            robot.sendVelocity(0,0); 
            
        end
    end
end
