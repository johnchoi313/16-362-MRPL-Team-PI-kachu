%% --- GLOBALS --- %%
%% EX2 Case 2
global V; 
V = []; %zeros(1,500);
global T; 
T = []; %zeros(1,500);
global INITIAL_TIME;

INITIAL_TIME = double(robot.encoders.LatestMessage.Header.Stamp.Sec) ...
+ double(robot.encoders.LatestMessage.Header.Stamp.Nsec) /1000000000.0;


tic;
   
%global myPlot;
%myPlot = line('xdata', [], 'ydata', []); %plot(T, V, 'b-');


%Begin the encoder callback
robot.encoders.NewMessageFcn = @neatoEncoderEventListener;
enc = rossubscriber('/enc', @neatoEncoderEventListener);

% Plotting
%global myPlot;

%Send velocity for a bit
for i = 1:100
    robot.sendVelocity(0.15, 0.15);
    pause(0.1);
    figure(1);
    plot(T, V), title('Robot Velocity vs Time'), xlabel('Time'), ylabel('Velocity')

end
robot.sendVelocity(0.0, 0.0);
 
