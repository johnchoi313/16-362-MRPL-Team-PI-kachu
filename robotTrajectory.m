%% robotModel > figure8ReferenceControl > robotTrajectory > trajectoryFollower
classdef robotTrajectory < figure8ReferenceControl
    
    properties
        numSamples; % total number of samples
        index; % current index we are at
        
        % state variables
        totalDistance;
        totalTime;
        curX;
        curY;
        curTh;
        
        % arrays
        timeSamples; % Time samples        
        distSamples; % Distance samples WRT time        
        poseSamples; % Pose list in [x;y;th] WRT time
                     % x = 1; y = 2; th = 3; for 2D array access
    end
    
    methods(Static = true)
        % initialize robotTrajectory properties
        function obj = robotTrajectory(n)
            obj = obj@figure8ReferenceControl(0.4,0.4,0.1); 
            
            % intialize state variables
            obj.totalTime = obj.t0;
            obj.totalDistance = 0;
            obj.curX = 0;
            obj.curY = 0;
            obj.curTh = 0;
            
            % initialize arrays
            obj.numSamples = n;
            obj.timeSamples = zeros(1,n); %time starts at 0  
            obj.distSamples = zeros(1,n); % distance starts at 0
            obj.poseSamples = zeros(3,n); % pose starts at global origin 
                                          % (x = 0,y = 0,th = 0)
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
        
        function dist = updateDistance(obj,t)
            % get dt
            dt = t - obj.totalTime;
            % get velocity
            V = obj.getVelocity(obj,t);
            % update total distance with increase in distance
            obj.totalDistance = obj.totalDistance + V*dt;
            % return updated distance
            dist = obj.totalDistance;
            % add distance to distSamples
            obj.distSamples(obj.index) = dist;
        end
      
        function [x, y, th] = updatePose(obj,t)  
            % get dt
            dt = t - obj.totalTime;
            
            % get omega
            w = obj.getOmega(obj,t);
            % update theta
            obj.curTh = obj.curTh + w*dt;
            
            % get velocity
            V = obj.getVelocity(obj,t);
            % get x,y vector from theta
            dx = V*cos(obj.curTh);
            dy = V*sin(obj.curTh);
            % update x and y position
            obj.curX = obj.curX + dx*dt;
            obj.curY = obj.curY + dy*dt;
            
            %return x, y and theta as pose
            x = obj.curX;
            y = obj.curY;
            th = obj.curTh;
            % add pose to poseSamples
            obj.poseSamples(1, obj.index) = x;
            obj.poseSamples(2, obj.index) = y;
            obj.poseSamples(3, obj.index) = th;
        end
        
        function updateTime(obj,t)
            % update the total time elapsed
            obj.totalTime = t;
            % add the time to timeSamples
            obj.timeSamples(obj.index) = t;
            % increment to the next index
            obj.index = obj.index + 1;
        end
        
    end    
end
