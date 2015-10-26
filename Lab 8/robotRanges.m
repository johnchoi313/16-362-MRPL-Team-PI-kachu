classdef robotRanges < handle

     properties
        robot;
     end
     
     methods(Access = public)
         % constructor
         function obj = robotRanges(robot)
             obj.robot = robot;
             obj.robot.startLaser();
         end
         % get ranges
         function ranges = getRanges(obj) 
             ranges = obj.robot.laser.LatestMessage.Ranges;
         end   
         
         
     end
end

         
     