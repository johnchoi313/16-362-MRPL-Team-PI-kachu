%% robotModel > figure8ReferenceControl > robotTrajectory > trajectoryFollower
classdef robotTrajectory < figure8ReferenceControl
    
    properties
    end
    
    methods(Static = true)
        % initialize robotTrajectory properties
        function obj = robotTrajectory(n)
            obj = obj@figure8ReferenceControl(0.4,0.4,0.1); 
        end
               
        function velocity = getVelocity(obj,t)
            % compute velocity from superclass
            [V, w] = obj.computeControl(obj,t);
            velocity = V;
        end
        function omega = getOmega(obj,t)
            % compute omega from superclass
            [V, w] = obj.computeControl(obj,t);
            omega = w;
        end
        
        function dist = updateDistance(obj,t,dt,d)
            % get velocity
            V = obj.getVelocity(obj,t);
            % return updated distance
            dist = d + V*dt;
        end
      
        function [x, y, th] = updatePose(obj,t,dt,X,Y,Th)   
            % get omega
            w = obj.getOmega(obj,t);
            % update theta
            th = Th + w*dt;
            
            % get velocity
            V = obj.getVelocity(obj,t);
            % get x,y vector from theta
            dx = V*cos(obj.curTh);
            dy = V*sin(obj.curTh);
            % update x and y position
            x = X + dx*dt;
            y = Y + dy*dt;
        end 
    end    
end
