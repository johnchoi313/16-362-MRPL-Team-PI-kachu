function [ slope, midX, midY ] = bestFit(X, Y)
% Input X and Y as a 1-D Row vector. Ex a = [1 2 3 ...]
% outputs slope and midpoint cords (x,y) of line segment

%Parameters for plotting only
resolution = 0.01;
numPoints = size(X,2);

% Make X and Y a column vector
X = X'; 
Y = Y'; 

% Vector of ones for constant term
Const = ones(size(X)); 
% Find the coefficients
Coeffs = [X Const]\Y; 

% Slope and Y intercept of inputed data
m1 = Coeffs(1);
b1 = Coeffs(2);

% Evaluate fitted curve at many points
bestFitX = 0:0.01:1;
% Vector of ones for constant term
Const = ones(size(X)); 
% Find the coefficients
coeffs = [X Const]\Y; 

% Find slope and y-intercept of best fit line
slope = coeffs(1);
yIntercept = coeffs(2);

bestFitX = X(1):resolution:X(end); %numPoints;
bestFitY = slope*bestFitX+yIntercept;

% Find Midpoint
midXIndex = round(size(bestFitX,2)/2);
midX = bestFitX(midXIndex);
midY = slope*midX+yIntercept;

% ****** DO NOT NEED ***** FOR PLOTTING ONLY *****%
% Plot the original points and the fitted curve
figure;
hold on;
plot(X,Y,'ro') %data points
plot(midX, midY, 'c*');
plot(bestFitX, bestFitY, 'g-');
title(sprintf('Noisy data: y=%f*x+%f',slope,yIntercept))
end

