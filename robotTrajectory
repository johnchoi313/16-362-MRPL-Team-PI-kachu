%% robotModel > figure8ReferenceControl > robotTrajectory 
classdef robotTrajectory < figure8ReferenceControl
    
    properties
        numSamples; % total number of samples
        
        timeSamples; % Time samples
        velSamples; % Velocity samples WRT time
        distSamples; % Distance samples WRT time
        poseSamples; % Pose list in [x;y;th] WRT time
                     % x = 1; y = 2; th = 3; for 2D array access
    end
    
    methods(Static = true)
        function obj = robotTrajectory(n)
            obj = obj@figure8ReferenceControl(0.4,0.4,0.1); 
            obj.numSamples = n;
            
            obj.timeSamples = zeros(1,n); %time starts at 0  
            obj.velSamples = zeros(1,n); % velocity starts at 0
            obj.distSamples = zeros(1,n); % distance starts at 0
            obj.poseSamples = zeros(3,n); % pose starts at global origin 
                                          % (x = 0,y = 0,th = 0)
        end
               
        function velocity = getVelocityAtTime(obj,t)
            [V w] = obj.computeControl(obj,t);
            velocity = V;
        end
        
        function dist = getDistAtTime(obj,t)
            dist = 0;
        end
      
        function pose = getPoseAtTime(obj,t)            
            pose = [1;2;3];
        end
        
    end    
end
