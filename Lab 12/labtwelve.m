%%% --- INITIALIZATION --- %%%
% create trajectory follower
tf = trajectoryFollower(robot);
% create range finder
rr = robotRanges(robot);
% constants
sensorOffset = 0.23;
ftm = 0.3048;

%%% --- LAB SEVEN TEST CODE --- %%%
%tf.followTrajectory(0.25,0.25,0.0, 1.0);
%pause(2);
%tf.followTrajectory(-0.5,-0.5,-pi/2, 1.0);
%pause(2);
%tf.followTrajectory(-0.25,0.25,pi/2, 1.0);

%%% --- LAB TWELVE PICKUP --- %%% 

% METHOD 1: ONE BY ONE.

% %%% FIRST PALLET %%% 
% robot.forksDown();
% tf.rotate(180);
% tf.followTrajectory(2.75*ftm-sensorOffset+.03,-.25*ftm-.01, 0, 1.0); % relative distance to first pallet
% robot.forksUp();
% tf.rotate(180);
% tf.followTrajectory(2.25*ftm-sensorOffset+.01,.75*ftm-.01, 0, 1.0);
% robot.forksDown();
% tf.backup(10);
% tf.rotate(180);
%%% SECOND PALLET %%%
%%% THIRD PALLET %%% 

% METHOD 2: GET 1, PUSH 2, PUT 1, GET 3. (FORKS DON'T WORK)

%%% FIRST PALLET AND SECOND PALLET %%% 
robot.forksDown();
tf.rotate(180);
tf.followTrajectory(2.78*ftm-sensorOffset+.03,-.25*ftm-.03, 0, 1.0); % relative distance to first pallet
robot.forksUp();

tf.followTrajectory(3.5*ftm-sensorOffset,-.68*ftm, 0, 1.0); % to second pallet push pose
tf.rotate(170); %rotate towards second pallet

tf.followTrajectory(5.0*ftm-sensorOffset, 0, 0, 1.0); % push second pallet to drop pose 2 

tf.rotate(90); 
tf.followTrajectory(.70*ftm-.01,0, 0, 1.0); % go to pallet drop pose 1
tf.rotate(-90);
tf.followTrajectory(.25*ftm-.01,0, 0, 1.0); % face pallet drop pose 1
robot.forksDown(); % drop pallet

tf.backup(15); % back up a bit
tf.rotate(180); % ready to scan and get third pallet

%%% THIRD PALLET %%%
tf.followTrajectory(1.5*ftm-sensorOffset,-1.7*ftm, 0, 1.0); % aim for third pallet
tf.followTrajectory(1.8*ftm-sensorOffset, 0, 0, 1.0); % get it
robot.forksUp();
tf.rotate(-180);

tf.followTrajectory(2.5*ftm, -.1*ftm, 0, 1.0); % move it to drop position.
robot.forksDown(); % drop third pallet

tf.backup(15);
tf.rotate(180);
