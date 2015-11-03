% Set up lines
p1 = [-2 ; -2];
p2 = [ 2 ; -2];
p3 = [ 2 ; 2];
p4 = [-2 ; 2];
lines_p1 = [p1 p2 p3 p4];
lines_p2 = [p2 p3 p4 p1];

wallsX = [-2 2 2 -2];
wallsY = [-2 -2 2 2];

% Set up test points
nPts = 10;
x1 = -2.0*ones(1,nPts);
x2 = linspace(-2.0,2.0,nPts);
x3 = 2.0*ones(1,nPts);
y1 = linspace(0.0,2.0,nPts);
y2 = 2.0*ones(1,nPts);
y3 = linspace(2.0,0,nPts);
w = ones(1,3*nPts);
x1pts = [x1 x2 x3];
y1pts = [y1 y2 y3];
w1pts = w;
modelPts = [x1pts ; y1pts ; w1pts];

% pick a pose
dx = -0.15*rand();
dy = -0.15*rand();
dt = -0.15+0.2*rand();
robotPose = pose(0.0+dx,0.0+dy,0.0+dt);

% loop the control infinitely
while(true)
    % Add a short pause
    pause(0.1);
    
    % Update pose
    [success, robotPose] = lml.refinePose(robotPose, modelPts, 10);
        
    %robot pose
    x = robotPose.poseVec(1);
    y = robotPose.poseVec(2);
    body = robotModel.bodyGraph();
    body = robotPose.bToA()*body;
    
    % world view
    worldLidarPts = robotPose.bToA()*modelPts;
    worldLidarPts = robotModel.senToWorld(robotPose)*modelPts;
    
    % plot everything
    figure(1);
    plot(wallsX,wallsY, 'b', body(1,:), body(2,:), 'g*',worldLidarPts(1,:),worldLidarPts(2,:), 'r*');
    title('X/Y Ranges'),...
    axis([-2.5 2.5 -2.5 2.5]),...
    xlabel('X (meters)'),... 
    ylabel('Y (meters)');   

end