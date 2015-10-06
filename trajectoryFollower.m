%% robotModel > figure8ReferenceControl > robotTrajectory > trajectoryFollower
classdef trajectoryFollower < robotTrajectory

    properties
    end
    
    methods(Static = true)
        % this function makes the robot follow the figure 8
        function obj = trajectoryFollower(robot)
            obj = obj@robotTrajectory(1000); 
            
            tic;
            t = toc;
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
                obj.updateDistance(obj, t);
                % update pose
                obj.updatePose(obj, t);      
                %update time
                obj.updateTime(obj, t);
                
                % pause 0.01
                pause(0.01);
                %update current frame's time 
                t = toc; 
                
            end
            % stop the robot
            robot.sendVelocity(0.0,0.0);
        end
    end
end
