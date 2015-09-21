%% --- GLOBALS --- %%
%% EX2 Case 2
global V; 
V = [];  %zeros(1,200);
global T; 
T = []; %zeros(1,200);

tic;
   
%global myPlot;
%myPlot = line('xdata', [], 'ydata', []); %plot(T, V, 'b-');


%Begin the encoder callback
robot.encoders.NewMessageFcn=@neatoEncoderEventListener;
enc=rossubscriber('/enc', @neatoEncoderEventListener);

% Plotting
%global myPlot;

%Send velocity for a bit
for i = 1:10
    robot.sendVelocity(0.15, 0.15);
    pause(1);
end
robot.sendVelocity(0.0, 0.0);
 
