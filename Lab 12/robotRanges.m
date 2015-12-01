classdef robotRanges < handle
    %robotRanges collects 
    
     properties
        robot;
     end
     
     methods(Access = public)
         % constructor
         function obj = robotRanges(robot)
             obj.robot = robot;
             obj.robot.startLaser();
         end
         % scans the environment n times and returns as single array
         function Ranges = getRanges(obj,n) 
             % pre-allocate range array
             Ranges = zeros(1, 360*n);
             % scan multiple times
             for i = 0:n              
                iStart = i*360+1;
                iEnd = i*360+360;
                ranges = obj.robot.laser.LatestMessage.Ranges;
                Ranges(iStart:iEnd) = ranges;
                pause(0.01);
             end                          
         end                     
     end
end

         
     