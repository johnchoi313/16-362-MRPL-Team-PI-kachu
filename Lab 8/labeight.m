tf = trajectoryFollower(robot);

tf.followTrajectory(0.25,0.25,0.0, 1.0);
pause(2);
tf.followTrajectory(-0.5,-0.5,-pi/2, 1.0);
pause(2);
tf.followTrajectory(-0.25,0.25,pi/2, 1.0);
