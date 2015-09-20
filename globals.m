% --- GLOBALS --- %

%Begin the encoder callback
robot.encoders.NewMessageFcn=@neatoEncoderEventListener;
enc=rossubscriber('/enc', @neatoEncoderEventListener);

%Send velocity for a bit
 for i = 1:10
    robot.sendVelocity(0.03, 0.03);
    pause(3);
    robot.sendVelocity(0.0, 0.0);
 end
 
