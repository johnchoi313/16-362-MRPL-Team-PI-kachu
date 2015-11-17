
%%% --- LAB SEVEN TEST CODE --- %%%
%tf = trajectoryFollower(robot);
%tf.followTrajectory(0.25,0.25,0.0, 1.0);
%pause(2);
%tf.followTrajectory(-0.5,-0.5,-pi/2, 1.0);
%pause(2);
%tf.followTrajectory(-0.25,0.25,pi/2, 1.0);

% create trajectory follower
tf = trajectoryFollower(robot);

%create range finder
rr = robotRanges(robot);

%get ranges and plot their positions
ri = rangeImage(rr.getRanges(2),1,true);
% find line (pallet width = .125; half of this is roughly .7)
[x y th] = ri.bestLineCandidate(.12,1);
% goto line
tf.followTrajectory(x,y,th, 1.0);
% plot data
ri.plotXvsY(2.0);
tf.plotData();

%% -------------------- %%

%get ranges and plot their positions
ri = rangeImage(rr.getRanges(2),1,true);
% find line (pallet width = 12.5; half of this is roughly 7)
[x y th] = ri.bestLineCandidate(.12,0);
% goto line
tf.followTrajectory(x,y,th, 1.0);

%% -------------------- %%

tf.backupRotate();