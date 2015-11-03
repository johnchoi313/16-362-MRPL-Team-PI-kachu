% initialize the key press driver
figure(1); pause(0.1);
rkd = robotKeypressDriver(figure(1));

% initialize the range finder
rr = robotRanges(robot);

% initialize the map
xO = 9*0.0254;
yO = 15*.0254;
thO = pi/2;

p0 = [0 - xO ; 0 - yO];
p1 = [0 - xO ; 1 - yO];
p2 = [1 - xO ; 0 - yO];
lines_p1 = [p0 p1];
lines_p2 = [p2 p0];
wallsX = [0, 1, 0, 0] - xO;
wallsY = [0, 0, 0, 1] - yO;

% initialize the line map localizer
lml = lineMapLocalizer(lines_p1,lines_p2, 0.01,0.001,0.0005);

% initialize the pose
%15*0.0254 = .381, 9*.0254 = .2286;
robotPose = pose(0,0,0);

% loop the control infinitely
while(true)
    % Add a short pause
    pause(0.1);
    
    % Get the keypress and move the robot
    robotKeypressDriver.drive(robot,1);
    
    % Get one laser scan
    ranges = rr.getRanges(1);
    % Update the information and plot (use only evry 5th point)
    ri = rangeImage(ranges,5,true);  
    % convert ranges to xy coordinates
    modelPts = [ri.xArray ; ri.yArray; ones(1,length(ri.xArray))];
    % convert lidar to frame
    modelPts = robotPose.bToA()*modelPts;
    
    % Update pose
    [success, robotPose] = lml.refinePose(robotPose, modelPts, 10);
        
    %robot pose
    x = robotPose.poseVec(1);
    y = robotPose.poseVec(2);
    body = robotModel.bodyGraph();
    body = robotPose.bToA()*body;
    
    % world view
    worldLidarPts = robotModel.senToWorld(robotPose)*modelPts;
    
    % plot everything
    figure(1);
    plot(wallsX,wallsY, 'b', body(1,:), body(2,:), 'g*',worldLidarPts(1,:),worldLidarPts(2,:), 'r*');
    title('X/Y Ranges'),...
    axis([-.5 1.5 -.5 1.5]),...
    xlabel('X (meters)'),... 
    ylabel('Y (meters)');   

end