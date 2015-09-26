function [x y th] = closestObject(robot)
%% Parameters
minVal = 0.06; %Meters
maxObjectRange = 1.5; % Meters
maxBearing = pi/2; %Radians

%% Get Ranges From Laser
Ranges = robot.laser.LatestMessage.Ranges;

%% Ranges less than minVal are set to 0
validRanges = (Ranges > minVal) .* Ranges;
%Ranges less than maxObjectRange are set to 0
validRanges = (validRanges < maxObjectRange) .* validRanges;
%Ranges with bearing pi/2 - -pi/2 set to 0
validRanges(92:270) = 0;

%Find index and range of smallest non zero range
[minRange, index] = min((1000*(validRanges==0))+validRanges)
%validRanges = (1000*(validRanges==0))+validRanges

figure(1);
x = 1:360
plot(x,validRanges)

[x y th] = irToXy(index, minRange)

figure(1);
plot(-y,x, 'X') , axis([-2 2 -2 2]), xlabel('Y (meters)'), ylabel('X (meters)');
end
