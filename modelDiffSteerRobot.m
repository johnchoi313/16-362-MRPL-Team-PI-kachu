
% lab defined k
k = 2;
kv = 0.4;
ks = 0.5;

% lab defined total time 
tf = 224;
t0 = 0;
dt = 0.01;

%% this loop happens for 22.4 seconds
for t = 1:tf
    
   % t = double(i) * dt;
    
    %% set vl and vr according to Cornu Spiral
    %vl = 1000 * (0.1/k + 0.01174 * (t/(k^2)))
    %vr = 1000 * (0.1/k - 0.01174 * (t/(k^2)))

    %% set vl and vr according to Figure 8
    vl = 1000 * (0.3 * kv + 0.14125 * kv/ks * sin(t*kv/(2*ks)))
    vr = 1000 * (0.3 * kv - 0.14125 * kv/ks * sin(t*kv/(2*ks))) 
    
    robot.sendVelocity(vl*.001,vr*.001);
    
    pause(1);
    
    %plot trajectory
    %modelDiffSteerRobot(vl, vr, t0, tf, t);
    
end
robot.sendVelocity(0,0);

%% function converts parameters to graphable Cartesian points
%function [x y th] = modelDiffSteerRobot(vl, vr, t0, tf, t) 




%end

