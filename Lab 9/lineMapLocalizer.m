classdef lineMapLocalizer < handle
    %mapLocalizer A class to match a range scan against a map in 
    % order to find the true location of the range scan relative to 
    % the map.
    
    properties (Constant)
        maxErr = 0.05; % 5 cm
        minPts = 5; % min # of points that must match
    end

    properties (Access = private)
    end

    properties (Access = public)
        lines_p1 = [];
        lines_p2 = [];
        gain = 0.0;
        errThresh = 0.0;
        gradThresh = 0.0;  
    end
    
    methods (Access = public)
        function obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)
            % create a lineMapLocalizer
            obj.lines_p1 = lines_p1;
            obj.lines_p2 = lines_p2;
            obj.gain = gain;
            obj.errThresh = errThresh;
            obj.gradThresh = gradThresh;
        end

        function ro2 = closestSquaredDistanceToLines(obj,pi)
            % Find the squared shortest distance from pi to any line 
            % segment in the supplied list of line segments.
            % pi is an array of 2d points
            % throw away homogenous flag
            pi = pi(1:2,:);
            r2Array = zeros(size(obj.lines_p1,2),size(pi,2));

            for i = 1:size(obj.lines_p1,2)
                [r2Array(i,:) , ~] = closestPointOnLineSegment(pi,obj.lines_p1(:,i),obj.lines_p2(:,i));   
            end

            ro2 = min(r2Array,[],1);
        end

        function ids = throwOutliers(obj,pose,ptsInModelFrame)
            % Find ids of outliers in a scan. 
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            ids = find(sqrt(r2) > obj.maxErr);
        end

        function avgErr = fitError(obj,pose,ptsInModelFrame)
            % Find the standard deviation of perpendicular distances of
            % all points to all lines
            % transform the points
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            r2(r2 == Inf) = [];
            err = sum(r2);
            num = length(r2);
            if (num >= lineMapLocalizer.minPts)
                avgErr = sqrt(err)/num;
            else % not enough points to make a guess
                avgErr = inf;   
            end
        end

        function [errPlus0,J] = getJacobian(obj,poseIn,modelPts)
            % Computes the gradient of the error function
            errPlus0 = fitError(obj,poseIn,modelPts)
            eps = 0.001;
            dp = [eps ; 0.0 ; 0.0];
            newPose = pose(poseIn.poseVec + dp);
            % Fill me in...
            errorX = (fitError(obj,newPose,modelPts) - errPlus0)/eps;
                        
            dp = [0.0 ; eps ; 0.0];
            newPose = pose(poseIn.poseVec+dp);
            errorY = (fitError(obj,newPose,modelPts) - errPlus0)/eps;
            
            dp = [0.0 ; 0.0 ; eps];
            newPose = pose(poseIn.poseVec+dp);
            errorTh = (fitError(obj,newPose,modelPts) - errPlus0)/eps;
            J = [errorX;errorY;errorTh] 
        end
 
        function [success, outPose] = refinePose(obj,inPose,ptsInModelFrame,maxIters)
            % refine robot pose in world (inPose) based on lidar
            % registration. Terminates if maxIters iterations is
            % exceeded or if insufficient points match the lines.
            % Even if the minimum is not found, outPose will contain
            % any changes that reduced the fit error. Pose Changes that
            % increase fit error are not included and termination
            % occurs thereafter.

            % get rid of outliers
            modelPts = ptsInModelFrame;
            %ids = obj.throwOutliers(inPose,modelPts);
            %modelPts(:,ids) = [];
            % use sensor offset coords
            outPose = pose(robotModel.senToWorld(inPose));
            % initialize error
            errPlus0 = obj.errThresh + 1;
            
            % compute Jacobian up to maxIters
            i = 0;
            while (i < maxIters) && (errPlus0 > obj.errThresh) 
                i = i + 1
                [errPlus0,J] = obj.getJacobian(outPose,modelPts)
                outPose.poseVec = J .* inPose.poseVec;       
                dPose = (-J * obj.gain).*inPose.poseVec;
                outPose.poseVec = outPose.poseVec + dPose;
                %dPose = -J; %(-J * obj.gain).*inPose.poseVec;
                %outPose.poseVec = inPose.poseVec + dPose;
                % outPose = pose(J .* inPose.poseVec) + dPose;
                %outPose.poseVec = outPose.bToA() * outPose.poseVec;
            end
            
            %adjust robot sensor offset
            outPose = pose(robotModel.robToWorld(outPose));
            
            % check whether within error threshold
            if (errPlus0 < obj.errThresh) 
                success = true;
            else
                success = false;
            end            
        end        
    end    
end
