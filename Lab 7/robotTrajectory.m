%% robotModel > figure8ReferenceControl > robotTrajectory > trajectoryFollower
classdef robotTrajectory < handle
    
    properties
    end
    
    methods(Static = true)
        % initialize robotTrajectory properties
        function obj = robotTrajectory(n)
        end
        
        function [x, y, th] = updatePose(obj,t,dt,X,Y,Th)   
            % get omega
            w = obj.getOmega(obj,t);
            % update theta
            th = Th + w*dt;
            
            % get velocity
            V = obj.getVelocity(obj,t);
            % get x,y vector from theta
            dx = V*cos(th);
            dy = V*sin(th);
            % update x and y position
            x = X + dx*dt;
            y = Y + dy*dt;
        end 
        
        function [x, y, th] = updateActualPose(obj,t,dt,X,Y,Th,V,w,simTh)   
            % update theta
            if(simTh == 0)
                th = double( (Th + w*dt) );
            else
                th = double(((Th + w*dt) + (simTh + w*dt))*0.5);
            end
            
            % get x,y vector from theta
            dx = double(V*cos(th));
            dy = double(V*sin(th));
            % update x and y position
            x = X + dx*dt;
            y = Y + dy*dt;
        end 
        
    end    
end
