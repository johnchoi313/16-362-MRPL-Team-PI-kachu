classdef robotKeypressDriver < handle
    %robotKeypressDriver Creates a keyboard event handler and then lets the
    %user drive the robot with the arrow keys.

    properties (Constant)
    end
    
    properties (Access = private)
        fh=[];
    end
    
    properties (Access = public)
    end
    
    methods (Static = true)
        function drive(robot,moveV,rotateV)
            % Get keypress
            key = pollKeyboard();
            
            % Check key and determine velocity
            if(key ~= false)
                vl = 0.0;
                vr = 0.0;
                
                % slightly modified velocities for more intuitive control
                if(strcmp(key, 'uparrow'))
                    disp('up');
                    vl = moveV; vr = moveV;
                elseif(strcmp(key,'downarrow'))
                    disp('down');
                    vl = -moveV; vr = -moveV;
                elseif(strcmp(key,'leftarrow'))
                    disp('left');
                    vl = -rotateV; vr = rotateV;
                elseif(strcmp(key,'rightarrow'))  
                    disp('right');
                    vl = rotateV; vr = -rotateV;
                elseif(strcmp(key,'s'))  
                    disp('stop');
                    vl = 0.0; vr = 0.0;
                end;
                robot.sendVelocity(vl,vr);
            end
        end    
    end
    
    methods(Access = private)
    end
    
    methods(Access = public)

        function obj = robotKeypressDriver(fh)
            % create a robotKeypressDriver for the figure handle
            % normally you call this with gcf for fh
            obj.fh = fh;
            set(fh,'KeyPressFcn',@keyboardEventListener);
        end

        function bodyPts = bodyGraph()
            % return an array of points that can be used to plot the robot
            % body in a window.
            % angle arrays
            step = pi/20;
            q1 = 0:step:pi/2;
            q2 = pi/2:step:pi;
            cir = 0:step:2*pi;          
            % circle for laser
            lx = robotModel.laser_rad*-cos(cir) + robotModel.laser_l;
            ly = robotModel.laser_rad*sin(cir);
            % body rear
            bx = [-sin(q1)*robotModel.rad lx [-sin(q2) 1  1  0]*robotModel.rad];
            by = [-cos(q1)*robotModel.rad ly [-cos(q2) 1 -1 -1]*robotModel.rad];
            %create homogeneous point
            bodyPts = [bx ; by ; ones(1,size(bx,2))];
        end
    
    end
    
end
