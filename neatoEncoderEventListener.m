function neatoEncoderEventListener(obj,msg)
    %globals
    global neatoEncoderDataTimeStamp;
    global neatoEncoderRight;
    global neatoEncoderLeft;
    
    %Get time stamp
    neatoEncoderDataTimeStamp=obj.LatestMessage.Header.Stamp.Sec +(obj.LatestMessage.Header.Stamp.Nsec/1000000000);
      
    %Check if left encoder changed
    if(neatoEncoderLeft ~= msg.Left)
        neatoEncoderLeft = msg.Left
    end
    
    %Check if right encoder changed    
    if(neatoEncoderRight ~= msg.Right)
        neatoEncoderRight = msg.Right
    end
end
