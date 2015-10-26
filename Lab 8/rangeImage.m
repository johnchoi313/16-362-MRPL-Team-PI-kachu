classdef rangeImage < handle
    %rangeImage Stores a 1D range image and provides related services.

    properties(Constant)
        maxUsefulRange = 2.0;
        minUsefulRange = 0.05;
        maxRangeForTarget = 1.0;
    end

    properties(Access = public)
        rArray = [];
        tArray = [];
        xArray = [];
        yArray = [];
        numPix;
    end
    
    methods(Access = public)
    
        function obj = rangeImage(ranges,skip,cleanFlag)
            % Constructs a rangeImage for the supplied data.
            % Converts the data to rectangular coordinates
            if(nargin == 3)
                n=0;
                %for i=1:skip:length(ranges) % SKIP DOES NOT WORK, BUT
                %ARGUMENT IS NEEDED FOR SOME REASON
                for i=1:length(ranges)
                    n = n + 1;
                    obj.rArray(n) = ranges(i);
                    obj.tArray(n) = (i-1)*(pi/180);
                    obj.xArray(n) = ranges(i)*cos(obj.tArray(n));
                    obj.yArray(n) = ranges(i)*sin(obj.tArray(n));
                end
                obj.numPix = n;
                if cleanFlag; obj.removeBadPoints(); end;
            end
        end

        function removeBadPoints(obj)
            % takes all points above and below two range thresholds
            % out of the arrays. This is a convenience but the result
            % should not be used by any routine that expects the points
            % to be equally separated in angle. The operation is done
            % inline and removed data is deleted.
        
            % list of indices to be removed
            remove = [];
            % check every range
            for i = 1:length(obj.rArray)
                % if out of range, add index to list of indices to be removed
                if (obj.rArray(i) > obj.maxUsefulRange) || (obj.rArray(i) < obj.minUsefulRange)
                    remove(length(remove)+1) = i;
                end
            end
            % remove out of range indices
            obj.rArray(remove) = [];
            obj.tArray(remove) = [];
            obj.xArray(remove) = [];
            obj.yArray(remove) = [];        
            % update numpix
            obj.numPix = length(obj.rArray);
        end

        function plotRvsTh(obj, maxRange)
            % plot the range image after removing all points exceeding
            % maxRange
            
            % figure 1
            figure(1);
            % plot the information (meters)
            plot(obj.rArray,obj.tArray,'*'),...
            % plot information
            % axis([-1 1 -1 1]),...
            title('Range vs Theta'),...
            xlabel('Range (meters)'),...
            ylabel('Theta (degrees)');      
        end

        function plotXvsY(obj, maxRange)
            % plot the range image after removing all points exceeding
            % maxRange
            
            % figure 1
            figure(2);
            % plot the information (meters)
            plot(obj.xArray,obj.yArray,'*'),...
            % plot information
            axis([-obj.maxUsefulRange obj.maxUsefulRange -obj.maxUsefulRange obj.maxUsefulRange]),...
            title('X/Y Ranges'),...
            % X is FRONT of robot
            xlabel('X (meters)'),... 
            ylabel('Y (meters)');         
        end

        function [x y th] = bestLineCandidate(obj,maxLen)
            numMax = 1;
            % check every possible midpoint
            for mid = 1:length(obj.rArray)
                % check this candidate
                [e n t] = obj.findLineCandidate(mid,maxLen);
                % find line with greatest number of pixels
                if n > numMax;
                    numMax = n;
                    x = obj.xArray(mid);
                    y = obj.yArray(mid);
                    th = 0;                   
                end                    
            end            
        end
            
        function [err num th] = findLineCandidate(obj,middle,maxLen)
            % Find the longest sequence of pixels centered at pixel
            % “middle” whose endpoints are separated by a length less
            % than the provided maximum. Return the line fit error, the
            % number of pixels participating, and the angle of
            % the line relative to the sensor.
    
            % make a copy of the range data
            rArray = obj.rArray;
            tArray = obj.tArray;
            xArray = obj.xArray;
            yArray = obj.yArray;
            % get mid pixel
            midX = obj.xArray(middle);
            midY = obj.yArray(middle);
           
            % find all points within maxLen of middle pixel
            remove = [];
            dist = [];
            % check every range
            for i = 1:length(rArray)
                % get distance of this pixel from mid
                dist(i) = ((xArray(i)-midX)^2.0 + (yArray(i)-midY)^2.0)^0.5;
                
                % if out of range, add index to list of indices to be removed
                if(dist(i) > maxLen) remove(length(remove)+1) = i; end
                
                % square distance after done using it
                dist(i) = dist(i)^2;
            end
            % remove out of range indices
            distArray(remove) = [];
            rArray(remove) = [];
            tArray(remove) = [];
            xArray(remove) = [];
            yArray(remove) = [];      
            
            % return data
            num = length(rArray);
            err = sum(distArray)/num
            th = 0;
        end

        function num = numPixels(obj)
            num = obj.numPix;
        end

        % Modulo arithmetic on nonnegative integers. MATLABs choice to
        % have matrix indices start at 1 has implications for
        % calculations relating to the position of a number in the
        % matrix. In particular, if you want an operation defined on
        % the sequence of numbers 1 2 3 4 that wraps around, the
        % traditional modulus operations will not work correctly.
        % The trick is to convert the index to 0 1 2 3 4, do the
        % math, and convert back.
        function out = inc(obj,in)
            % increment with wraparound over natural numbers
            out = indexAdd(obj,in,1);
        end
        function out = dec(obj,in)
            % decrement with wraparound over natural numbers
            out = indexAdd(obj,in,-1);
        end
        function out = indexAdd(obj,a,b)
            % add with wraparound over natural numbers. First number
            % “a” is "natural" meaning it >=1. Second number is signed.
            % Convert a to 0:3 and add b (which is already 0:3).
            % Convert the result back by adding 1.
            out = mod((a-1)+b,obj.numPix)+1;
        end
    end
end