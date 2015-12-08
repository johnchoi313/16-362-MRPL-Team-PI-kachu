%% Robot Pose
x0 = 9*0.0254;
y0 = 15*.0254;
th0 = pi/2;
robotPose = pose(x0, y0, th0);

niter = 1
plot = true

%Create Laser object
ll = laserLocalize();
[updatedPose, success] = ll.localize(robot, robotPose, niter, plot);
