%% robotModel > figure8ReferenceControl > robotTrajectory 
classdef robotModel
     
    properties
        W; % wheel circumference
        W2; % half wheel circumference
        wheelbase; % wheel to wheel distance
    end
    
    methods(Static = true)
        function obj = robotModel()
            obj.W = 0.23939; % wheel circumference
            obj.W2 = obj.W/2; % half wheel circumference
            obj.wheelbase = 0.235; % wheel to wheel distance
        end
        
        % convert wheel velocities to linear/angular velocity
        function [V w] = vlvrToVw(vl, vr)
            V = (vl + vr) / 2;
            w = (vl - vr) / robotModel.wheelbase; 
        end
        
        % convert linear/angular velocity to wheel velocities
        function [vl vr] = VwTovlvr(V, w)
            vl = V - (robotModel.wheelbase/2)*w;
            vr = V + (robotModel.wheelbase/2)*w;
        end
    
    end
end
