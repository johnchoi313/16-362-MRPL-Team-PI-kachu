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
p1 = [0 - xO ; 1.2 - yO];
p2 = [1.2 - xO ; 0 - yO];
lines_p1 = [p0 p1];
lines_p2 = [p2 p0];
wallsX = [0, 1.2, 0, 0] - xO;
wallsY = [0, 0, 0, 1.2] - yO;

% initialize the line map localizer
lml = lineMapLocalizer(lines_p1,lines_p2, 0.01,0.0015,0.0005);

% initialize the pose
robotPose = pose(0,0,0);

%intialize Odometry Pose
odomPose = pose(0,0,0);

%
succeess = false;

% loop the control infinitely
while(true)
    
    % Add a short pause
    %pause(0.1);
    
    % Get the keypress and move the robot
    robotKeypressDriver.drive(robot,1);

    % Get one laser scan
    ranges = rr.getRanges(1);
    % Update the information and plot (use only every 5th point)
    ri = rangeImage(ranges,10,true);  
    % convert ranges to xy coordinates
    modelPts = [ri.xArray ; ri.yArray; ones(1,length(ri.xArray))];        
    % convert lidar to frame
    modelPts = robotPose.bToA()*modelPts;
    
     
    
    % Update pose
    if(success)
        % Merge odomPose and robotPose for increased accuracy
        mergedPose = mergePose(odomPose, robotPose);
        %Pass in mergedPose at new "true" pose and calculate next pose
        [success, robotPose] = lml.refinePose(mergedPose, modelPts, 20);
        %Update odomPose to last known "true" pose
        odomPose = robotPose;
    else
        %Success = false, so continue to just use triangulation
        [success, robotPose] = lml.refinePose(robotPose, modelPts, 20);
        
        % Need to get these values for odometry
        %V = getV;
        %x = getW;
        %dt = getDt;
        %Computes the pose using only odometry since last "true" pose
        %and iterate(update) on it every loop until we have a success
        odomPose = updatePoseOdom(odomPose, V, w, dt);
    end
               
    
    %Render pose is slightly offset from measured pose
    renderPose = pose(robotPose.poseVec);
    % apply rendering offsets
    renderPose.poseVec(3) = renderPose.poseVec(3)*2; % adjust for sensor offset
    renderPose.poseVec(1) = renderPose.poseVec(1)*2 + .12*cos(renderPose.poseVec(3)); 
    renderPose.poseVec(2) = renderPose.poseVec(2)*2 + .12*sin(renderPose.poseVec(3)); 
    % print out render pose info
    x = renderPose.poseVec(1);
    y = renderPose.poseVec(2);
    th = renderPose.poseVec(3);
    % show robot body
    body = robotModel.bodyGraph();
    body = renderPose.bToA()*body;
    
    % adjust modelPts with updated robotPose
    modelPts = robotPose.bToA()*modelPts;
    % world view
    worldLidarPts = modelPts;
    
    % plot everything
    figure(1);
    plot(wallsX,wallsY, 'b', body(1,:), body(2,:), 'g*',worldLidarPts(1,:),worldLidarPts(2,:), 'r*');
    title('X/Y Ranges'),...
    axis([-.5 1.5 -.5 1.5]),...
    xlabel('X (meters)'),... 
    ylabel('Y (meters)');    
end
