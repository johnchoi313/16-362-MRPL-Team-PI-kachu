
%tf.followTrajectory(0.25,0.25,0.0, 1.0);
%pause(2);
%tf.followTrajectory(-0.5,-0.5,-pi/2, 1.0);
%pause(2);
%tf.followTrajectory(-0.25,0.25,pi/2, 1.0);

%create range finder
rr = robotRanges(robot);
ranges = rr.getRanges();

%get ranges and plot their positions
ri = rangeImage(ranges,0,true);
ri.plotRvsTh(2.0);
ri.plotXvsY(2.0);
% find line (pallet width = 12.5; half of this is roughly 7)
[x y th] = ri.bestLineCandidate(7.0);

% create trajectory follower
tf = trajectoryFollower(robot);

% goto line
tf.followTrajectory(x,y,th, 1.0);



