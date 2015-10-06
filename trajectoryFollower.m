%% robotModel > figure8ReferenceControl > robotTrajectory > trajectoryFollower
classdef trajectoryFollower < robotTrajectory

    properties
        % current state variables
        curX;
        curY;
        curTh;
        curDist;
        
        % arrays for plotting
        timeSamples;  
        distSamples; 
        xSamples;
        ySamples;
        thSamples; 
    end
    
    methods(Static = true)
        % this function makes the robot follow the figure 8
        function obj = trajectoryFollower(robot)
            n = 250; % numSamples
            index = 1;
      
            % getting superclass information
            obj = obj@robotTrajectory(n); 
            
            % initialize state variables
            obj.curX = 0.0;
            obj.curY = 0.0;
            obj.curTh = 0.0;
            obj.curDist = 0.0;
            
            % initialize arrays
            obj.timeSamples = zeros(1,n);
            obj.distSamples = zeros(1,n);
            obj.xSamples = zeros(1,n);
            obj.ySamples = zeros(1,n);
            obj.thSamples = zeros(1,n); 
            
            %initialize time
            tic;
            t = toc;
            lastTime = t;
            dt = t - lastTime; 
            
            while(t < obj.tf)           
                % get linear and angular velocity
                [V, w] = obj.computeControl(obj, t);
                % convert to wheel velocities
                [vl, vr] = obj.VwTovlvr(obj, V, w);
                % cap wheel speeds
                [vl, vr] = obj.capvlvr(obj, vl,vr);
                % send wheel velocities
                robot.sendVelocity(vl,vr);   
             
                % update distance
                obj.curDist = obj.updateDistance(obj, t, dt, obj.curDist);
                % update pose
                [obj.curX, obj.curY, obj.curTh] = obj.updatePose(obj, t, dt, obj.curX, obj.curY, obj.curTh);      
                
                %update arrays
                obj.timeSamples(index) = t;
                obj.distSamples(index) = obj.curDist;
                obj.xSamples(index) = obj.curX;
                obj.ySamples(index) = obj.curY;
                obj.thSamples(index) = obj.curTh;
                
                %Plot   
                figure(1);
                subplot(2,2,1), plot(obj.timeSamples(1:index), obj.xSamples(1:index)), title('X vs Time');
                subplot(2,2,2), plot(obj.timeSamples(1:index), obj.ySamples(1:index)), title('Y vs Time');
                subplot(2,2,3), plot(obj.timeSamples(1:index), obj.thSamples(1:index)), title('Th vs Time');
                subplot(2,2,4), plot(obj.xSamples(1:index), obj.ySamples(1:index)), title('X vs Y');
                
                % pause 0.01
                pause(0.01);
                
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
