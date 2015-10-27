%% robotModel > figure8ReferenceControl > robotTrajectory > trajectoryFollower
classdef trajectoryFollower < handle

    properties
        % numSamples and current index
        n; 
        index;
        % robot
        robot;
        % curve
        curve;
        % actual state variables (real)
        actX;
        actY;
        actTh;
        % actual arrays for plotting
        actXSamples;
        actYSamples;
        actThSamples;
        % simulated state variables
        simX;
        simY;
        simTh;
        % simulated arrays for plotting
        simXSamples;
        simYSamples;
        simThSamples;
        % encoder variables
        lastEncR;
        lastEncL;
        encR;
        encL;    
        % feedback variables
        feedback;
        kx;
        ky;
    end
    
    methods(Access = public)
        % class constructor
        function obj = trajectoryFollower(robot)
            % numSamples
            obj.n = 202;
            obj.index = 1;
            % set robot
            obj.robot = robot;
            % create curve
            obj.curve = cubicSpiral([1.0,1.0,1.0],obj.n);
            % initialize actual variables
            obj.actX = 0.0;
            obj.actY = 0.0;
            obj.actTh = 0.0;
            % initialize actual arrays
            obj.actXSamples = [];
            obj.actYSamples = [];
            obj.actThSamples = []; 
            % initialize simulated variables
            obj.simX = 0.0;
            obj.simY = 0.0;
            obj.simTh = 0.0;
            % initialize simulated arrays
            obj.simXSamples = [];
            obj.simYSamples = [];
            obj.simThSamples = []; 
            % initialize encoders
            obj.encR = double(robot.encoders.LatestMessage.Right)*0.001;
            obj.encL = double(robot.encoders.LatestMessage.Left)*0.001;    
            obj.lastEncR = 0.0;
            obj.lastEncL = 0.0;
            % initialize feedback variables
            obj.feedback = 1;
            obj.kx = 0.07; 
            obj.ky = 0.07;
        end
        
        % follow given trajectory with current pose and target pose
        function followTrajectory(obj, x,y,th,sgn)
            % compute the curve and get the trajectories.
            obj.curve = obj.curve.planTrajectory(x,y,th,sgn);
            obj.curve.planVelocities(0.25);
            %initialize time
            tic;
            t = 0;
            tf = obj.curve.getTrajectoryDuration()
            % run the loop
            while(t < tf)    
                %% UPDATE ENCODERS, TIME, AND INDEX %%
                % update encoder variables
                obj.lastEncR = obj.encR;
                obj.lastEncL = obj.encL;
                obj.encR = double(obj.robot.encoders.LatestMessage.Right)*0.001;
                obj.encL = double(obj.robot.encoders.LatestMessage.Left)*0.001;    
                % update the last time
                lastTime = t;
                t = toc; 
                dt = t - lastTime;
                % increment the index
                obj.index = obj.index + 1;
                
                %% ADJUST WITH FEEDBACK AND SEND VELOCITY %%
                % get wheel velocities from feedforward
                vl = obj.curve.getVLAtTime(t);
                vr = obj.curve.getVRAtTime(t);
                % correct with feedback
                if(obj.feedback == 1)
                    obj.applyFeedback(vl,vr);
                end
                % send the velocity
                obj.robot.sendVelocity(vl,vr);        
                
                %% UPDATE ACTUAL POSE (X, Y AND TH) VARIABLES %% 
                %Calculate encoder change (ds)
                dEncR = (obj.encR - obj.lastEncR);
                dEncL = (obj.encL - obj.lastEncL);
                %Calculate actual wheel velocities
                actVR = (dEncR / dt);
                actVL = (dEncL / dt);
                % update simulated pose
                [obj.actX,obj.actY,obj.actTh] = obj.updatePose(dt, actVL,actVR, obj.actX,obj.actY,obj.actTh);
                % update position arrays
                obj.actXSamples(obj.index) = obj.actX;
                obj.actYSamples(obj.index) = obj.actY;
                obj.actThSamples(obj.index) = obj.actTh;  
                
                %% UPDATE SIMULATED POSE (X, Y AND TH) VARIABLES %% 
                % get simulated vl and vr
                simVL = vl;
                simVR = vr;
                % update simulated pose
                [obj.simX,obj.simY,obj.simTh] = obj.updatePose(dt, simVL,simVR, obj.simX,obj.simY,obj.simTh);
                % update position arrays
                obj.simXSamples(obj.index) = obj.simX;
                obj.simYSamples(obj.index) = obj.simY;
                obj.simThSamples(obj.index) = obj.simTh;  
            end
            %Stop the robot
            obj.robot.sendVelocity(0,0); 
            % plot the information
            obj.plotData();   
        end     
        
        %%% GET FEEDBACK AND APPLY IT %%%
        function [vl,vr] = applyFeedback(obj,vl,vr)
            %convert pose to matrices for processing
            p = pose(obj.actX,obj.actY,obj.actTh);
            % get error
            wrp = zeros(2);
            wrp(1) = obj.simX - obj.actX;
            wrp(2) = obj.simY - obj.actY;
            % multiply the two
            rrp = p.aToBRot() * wrp;
            % make feedback constant matrix
            kxy = zeros(2,2);
            kxy(1,1) = obj.kx;
            kxy(2,2) = obj.ky;
            % get V and W
            fb_Vw = kxy * rrp;
            fb_V = fb_Vw(1);
            fb_w = fb_Vw(2);
            % Convert to vl and vr
            [fb_vl, fb_vr] = robotModel.VwTovlvr(fb_V, fb_w);
            % add feedback to wheel velocities
            vl = vl + fb_vl;
            vr = vr + fb_vr;
            % cap speeds
            if(vl > 0.3) vl = 0.3; end
            if(vl < -0.3) vl = -0.3; end
            if(vr > 0.3) vr = 0.3; end
            if(vr < -0.3) vr = -0.3; end
        end
        
        %%% UPDATE POSE VARIABLES AND ARRAY %%%
        function [x,y,th] = updatePose(obj, dt, vl,vr, x,y,th)
            %get simulated wheel velocities
            [V, w] = robotModel.vlvrToVw(vl, vr);
            % update theta
            th = double(th + w*dt);
            % get x,y vector from theta
            dx = double(V*cos(th));
            dy = double(V*sin(th));
            % update x and y position
            x = x + dx*dt;
            y = y + dy*dt;
        end
        
        %%% PLOTS DATA, END TO END FOR SIMULATOR VS ACTUAL %%%
        function plotData(obj)
            % keep plot on 
            hold on;
            % plot the information (meters)
            plot(obj.simXSamples,obj.simYSamples,obj.actXSamples,obj.actYSamples),...
            % plot information
            axis([-1 1 -1 1]),...
            title('X/Y Position'),...
            xlabel('Y (meters)'),...
            ylabel('X (meters)');        
        end
        
    end
end
