% initialize the key press driver
figure(1); pause(0.1);
rkd = robotKeypressDriver(figure(1));

% initialize the range finder
rr = robotRanges(robot);

%% Robot Pose
x0 = 9*0.0254;
y0 = 15*.0254;
th0 = pi/2;

%% Initialize the map in world cordinates
mapLen = 1.3; %Meters

%Define Points
p0 = [0; mapLen];    
p1 = [0; 0];      
p2 = [mapLen; 0];     
p3 = [mapLen; mapLen];    

%Define the three lines
line1 = [p0 p1];
line2 = [p1 p2];
line3 = [p2 p3];

wallsX = [line1(1,:) line2(1,:) line3(1,:)]; 
wallsY = [line1(2,:) line2(2,:) line3(2,:)]; 

% initialize the line map localizer
lml = lineMapLocalizer(line1,line2, 0.01,0.0015,0.0005);
% lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)

%% Initialize the robot pose in the world frame
robotPose = pose(x0, y0, th0);
%NOTE robot pose is actual the senor pose 

while(1)
    % Get the keypress and move the robot
    robotKeypressDriver.drive(robot,1);
    
    % Get Ranges
    ranges = rr.getRanges(0);
    % Conver Pt to X, Y, TH
    ri = rangeImage(ranges, 10, true);
    
    %DO NOT USE, THIS WILL CONVERT SENSOR POSE TO ROBOT POSE
    % CONVERSION MATRIX, SENSOR=>WORLD FRAME
    % Finds the sensor pose in world given the robot pose in the world.
    %senToWorld = robotModel.senToWorld(robotPose);
    % Get Sensor Pose in Word Frame
    %sensorPoseVec = senToWorld * robotPose.getPoseVec;
    %sensorPose = pose(sensorPoseVec(1), sensorPoseVec(2), sensorPoseVec(3));
    
    % CONVERSION MATRIX, ROBOT=>WORLD FRAME
    %(world)Sensor=>Robot(world) 
    %Finds the robot pose in world given the sensor pose in the world.
    robotToWorld = robotModel.robToWorld(robotPose);
    
    %Convert ranges to xy coordinates
    modelPts = [ri.xArray ; ri.yArray; ones(1,length(ri.xArray))];        
    %Convert lidar scan to world frame ROBOT=>WORLD
    worldPts = robotToWorld*modelPts;
    worldX = worldpts(1,:);
    worldY = worldpts(2,:);
    
    % Get Robot Body in world frame
    body = robotToWorld*robotModel.bodyGraph();
    
    % Update pose
    [success, robotPose] = lml.refinePose(robotPose, modelPts, 40);
    
    %World View for plotting
    worldLidarPts = robotPose.bToA()*modelPts;
        
    %Print Robot Pose
    %robotPose.getPoseVec
    
    robPoseVec = robotPose.getPoseVec;
    
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
