classdef laserLocalize < handle
   
    properties
        %% Initialize the map in world cordinates
        mapLen = 1.3; %Meters

        %Define Points
        p0;    
        p1;      
        p2;     
        p3;    

        %Define the three lines
        line1;
        line2;
        line3;

        %Define Walls
        wallsX; 
        wallsY; 
    end
        
    methods(Static)
        function obj = laserLocalize(obj)
            %obj.localize(robot, robotPose, niter);
            
            %Define Points
            obj.p0 = [0; obj.mapLen];    
            obj.p1 = [0; 0];      
            obj.p2 = [obj.mapLen; 0];     
            obj.p3 = [obj.mapLen; obj.mapLen];    

            %Define the three lines
            obj.line1 = [obj.p0 obj.p1];
            obj.line2 = [obj.p1 obj.p2];
            obj.line3 = [obj.p2 obj.p3];
            
            %Define Walls
            obj.wallsX = [obj.line1(1,:) obj.line2(1,:) obj.line3(1,:)]; 
            obj.wallsY = [obj.line1(2,:) obj.line2(2,:) obj.line3(2,:)];
            
            %Call Localize
            %obj.localize(robot, robotPose, niter, plot)
        end
        
        
        function [updatedPose, success] = localize(obj, robot, robotPose, niter, plot)
            success = false;
            
            
            % initialize the range finder
            rr = robotRanges(robot);
            
            % initialize the line map localizer
            lml = lineMapLocalizer(obj.line1,obj.line2, 0.01,0.0015,0.0005);
            
            while(niter > 0 && success == false)
                % Get Ranges
                ranges = rr.getRanges(0);
                % Conver Pt to X, Y, TH
                ri = rangeImage(ranges, 10, true);

                % CONVERSION MATRIX, ROBOT=>WORLD FRAME
                %(world)Sensor=>Robot(world) 
                %Finds the robot pose in world given the sensor pose in the world.
                robotToWorld = robotModel.robotToWorld(robotPose);

                %Convert ranges to xy coordinates
                modelPts = [ri.xArray ; ri.yArray; ones(1,length(ri.xArray))];        
                %Convert lidar scan to world frame ROBOT=>WORLD
                worldPts = robotToWorld*modelPts;
                worldX = worldpts(1,:);
                worldY = worldpts(2,:);

                % Get Robot Body in world frame
                body = robotToWorld*robotModel.bodyGraph();

                % Updated Robot Pose
                [success, updatedPose] = lml.refinePose(robotPose, modelPts, 40);
                
                %Decrement niter
                niter = niter - 1;
                
                if(plot)
                    % Plot Everything
                    figure(1);
                    plot(wallsX, wallsY, 'b', ...
                        body(1,:), body(2,:), 'g*', ...
                        worldLidarPts(1,:), worldLidarPts(2,:), 'b*', ...
                        robPoseVec(1), robPoseVec(2), 'r*');
                    %worldX, worldY, 'r*');% ...
                        %ri.xArray, ri.yArray, 'y*');
                    title('X/Y Ranges'),...
                    axis([-.5 1.5 -.5 1.5]),...
                    xlabel('X (meters)'),... 
                    ylabel('Y (meters)');
                end
            end
         end
        
    end
         
    
end
