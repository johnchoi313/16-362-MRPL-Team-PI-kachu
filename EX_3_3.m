global dt

% lab defined k
k = 1;
kv = 0.4;
ks = 0.5;

% lab defined total time 
tf = 22.4;
t0 = 0.0;
t = 0.0;

global INITIAL_TIME;
INITIAL_TIME = double(robot.encoders.LatestMessage.Header.Stamp.Sec) ...
+ double(robot.encoders.LatestMessage.Header.Stamp.Nsec) /1000000000.0;

%% this loop happens for 22.4 seconds
while(t <  50)

    t = double(robot.encoders.LatestMessage.Header.Stamp.Sec) ...
+ double(robot.encoders.LatestMessage.Header.Stamp.Nsec) /1000000000.0 - INITIAL_TIME
    
    %% set vl and vr according to Cornu Spiral
    %vl = 1000 * (0.1/k + 0.01174 * (t/(k^2)))
    %vr = 1000 * (0.1/k - 0.01174 * (t/(k^2)))

    %% set vl and vr according to Figure 8
    vl = (0.3 * kv + 0.14125 * kv/ks * sin(t*kv/(2*ks)));
    vr = (0.3 * kv - 0.14125 * kv/ks * sin(t*kv/(2*ks)));
    
    robot.sendVelocity(vl,vr);
    
    %pause(1);
    
    %plot trajectory
    %modelDiffSteerRobot(vl, vr, t0, tf, t);
    
end
robot.sendVelocity(0,0);

%% function converts parameters to graphable Cartesian points
%function [x y th] = modelDiffSteerRobot(vl, vr, t0, tf, t) 

%end

