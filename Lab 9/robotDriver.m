% initialize the key press driver
figure(1); pause(0.1);
rkd = robotKeypressDriver(figure(1));

% initialize the range finder
rr = robotRanges(robot);

% loop the control infinitely
while(true)
    % Add a short pause
    pause(0.1);
    
    % Get the keypress and move the robot
    robotKeypressDriver.drive(robot,0.1,0.1);
    
    % Get one laser scan
    ranges = rr.getRanges(1);
    
    % Update the information and plot
    ri = rangeImage(ranges,0,true);   
    ri.plotXvsY(1.0);
end