%% robotModel > figure8ReferenceControl > robotTrajectory > trajectoryFollower
classdef figure8ReferenceControl < robotModel
    
    properties
        k_v; % Figure 8 velocity
        k_s; % Figure 8 curvature
        
        t0; % Time initial
        tf; % Time final
        initial_delay; % AKA tPause on Lab docs.
    end
        
    methods(Static = true) 
        function obj = figure8ReferenceControl(Ks,Kv,tPause)
            % Construct a figure 8 trajectory. It will not start until 
            % tPause has elapsed and it will stay at zero for tPause
            % afterwards. Kv scales velocity up when > 1 and Ks scales 
            % the size of the curve itself down. 0.4 is a good value
            % for both.
            
            % Get robotModel's constructor
            obj = obj@robotModel();  
            
            % Set velocity and curvature variables
            obj.k_v = Kv;
            obj.k_s = Ks;
            
            % set time variables
            obj.t0 = 0.0;
            obj.tf = 13.565; %/(Kv*Ks);
            obj.initial_delay = tPause;     
        end
        
        function [V, w] = computeControl(obj,t)
            % Return the linear and angular velocity that the robot
            % should be executing at time timeNow. Any zero velocity
            % pauses specified in the constructor are implemented here 
            % too.
        
            % Shorten constant names
            Kv = obj.k_v;
            Ks = obj.k_s;
        
            if (t < 12.565)
            % Calculate left and right wheel velocities
            vr = 0.3*Kv + 0.14125 * (Kv/Ks) * sin(t*Kv/(2*Ks));
            vl = 0.3*Kv - 0.14125 * (Kv/Ks) * sin(t*Kv/(2*Ks));
            
            % Convert to V (linear velocity) and w (angular velocity)
            [V, w] = robotModel.vlvrToVw(obj,vl,vr);
            else
            V = 0.01;
            w = 0.0;
            end
           
        
        end
        
        function duration = getTrajectoryDuration(obj)
            % Return the total time required for motion and for the 
            % initial and terminal pauses.
            duration = obj.tf - obj.t0;
        end
    end
    
end
