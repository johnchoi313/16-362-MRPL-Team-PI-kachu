classdef controller < trajectoryFollower
   properties
       feedback;
       logging;
       robot;
       
       %Encoder Values
       curEncR;
       curEncL;
       prevEncR;
       prevEncL;
       
       %Time
       prevTime;
       
       %Position
       prevX;
       prevY;
       
       
       
   end
   
   methods(Static = true)
       function obj = controller(robot, logging, feedback)
           obj = obj@trajectoryFollower(robot);
           
           obj.robot = robot;
           obj.logging = logging;
           obj.feedback = feedback;
           
           %Encoder Values
           obj.curEncR = 0;
           obj.curEncL = 0;
           obj.prevEncR = 0;
           obj.prevEncL = 0;
           
           %Time
           obj.prevTime = 0;
           
           %Positoin
           obj.prevX = 0;
           obj.prevY = 0;
           
      
       end
       
       function updateEncVals(obj)
           %Update Previous Encoder Values
           obj.prevEncR = obj.curEncR;
           obj.prevEncL = obj.curEncL;
           
           %Update Encoder Values
           obj.curEncR = obj.robot.encoders.LatestMessage.right();
           obj.curEncL = obj.robot.encoders.LatestMessage.left();    
       end
       
       function [V, w] = getActualVelocity(obj, t)
           %Calculate Dt
           dt = t - obj.prevT;
           
           %Update Previous Time
           obj.prevTime = t;
           
           %Calculate encoder change (ds)
           dEncR = (obj.curEncR - obj.prevEncR);
           dEncL = (obj.curEncL - obj.prevEncL);
           
           %Calculate Velocity Right/Left
           vr = dEncR / dt;
           vl = dEncL / dt;
           
           V = (vr + vl) / 2;
           w = (vr - vl) / wheelbase;
       end
      
        

       function [actualX, actualY, actualTh] = getActualPose(obj, t)
           %Calculate Dt
           dt = t - obj.prevTime;
           
           %Get Velovity and Omega
           [V, w] = getActualVelovity(obj,t);
           
           %Get Theta
           th = obj.prevTheta + w*dt;
           %Update Previous Theta
           obj.prevTheta = th;
           
           %Get X/Y vector from theta
           dx = V*cos(th);
           dy = V*sin(th);
           
           %Update X/Y position
           x = obj.prevX + dx*dt;
           y = obj.prevY + dy*dt;
           %Update Previous X/Y
           obj.prevX = X;
           obj.prevY = Y;
           
           %Return X/Y/Th as Pose (MIGHT NOT NEED THIS HERE)
           obj.actualXSamples(1, obj.index) = x;
           obj.actualYSamples(2, obj.index) = y;
           obj.actualTHSamples(3, obj.index) = th;
           
           actualX = x;
           actualY = Y;
           actualTh = th;
       end
           
       function error = getError(obj, t);
           %Calculate Dt
           dt = t - obj.prevTime;
           
           %Get Actual Positions
           [actualX, actualY, actualTh] = getActualPose(obj,t);
           
           %Get Theoretical Postions
           [theoreticalX, theoreticalY, theoreticalTh] = obj.getPose(obj, t, dt, actualX, actualY, actualTh);
           
           
           %Calculate Error
           errorX = actualX - theoreticalX;
           errorY = actualY - theoreticalY;
           errorTh = actualTh - theoreticalTh
                   
       end    
end
