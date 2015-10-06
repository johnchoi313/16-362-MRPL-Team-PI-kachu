%% robotModel > figure8ReferenceControl > robotTrajectory > trajectoryFollower
classdef robotModel
     
    properties
        maxV; % maximum velocity
        W; % wheel circumference
        W2; % half wheel circumference
        wheelbase; % wheel to wheel distance
    end
    
    methods(Static = true)
        function obj = robotModel()
            obj.maxV = 0.3; % maximum velocity
            obj.W = 0.23939; % wheel circumference
            obj.W2 = obj.W/2; % half wheel circumference
            obj.wheelbase = 0.235; % wheel to wheel distance
        end
        
        % convert wheel velocities to linear/angular velocity
        function [V, w] = vlvrToVw(obj, vl, vr)
            V = (vl + vr) / 2;
            w = (vl - vr) / obj.wheelbase; 
        end
        
        % convert linear/angular velocity to wheel velocities
        function [vl, vr] = VwTovlvr(obj, V, w)
            vl = V - (obj.wheelbase/2)*w;
            vr = V + (obj.wheelbase/2)*w;
        end
        
        % cap vl and vr values
        function [vl,vr] = capvlvr(obj, VL,VR)
            if(VL > obj.maxV)
                VL = obj.maxV;
            end
            if(VL < -obj.maxV)
                VL = -obj.maxV;
            end
            if(VR > obj.maxV)
                VR = obj.maxV;
            end
            if(VR < -obj.maxV)
                VR = -obj.maxV;
            end
            vl = VL;
            vr = VR;            
        end
    end
end
