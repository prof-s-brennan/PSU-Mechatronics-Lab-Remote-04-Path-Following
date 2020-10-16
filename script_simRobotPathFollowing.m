% script_simRobotPathFollowing.m
%
% This is a script that simulates a 2-wheel robot within a test course that
% is given a path to follow. The robot is equipped with one distance sensor
% whose reading emulates the behavior of an actual IR sensor (the GP2Y0A
% Sharp Sensor, specifically). The simulation loops through a time vector,
% and at each time lets the user calculate a turn and speed torque to send
% to the left and right motors respectively. It then sends these commands
% to a kinematic simulator that predicts the robot motion and the IR
% reading. If the robot hits a wall, the IR readings become NaN values
% which causes the code to stop. While driving, the position of the robot
% is tracked (the center point between the two dires). If the center is
% within a distance of user-definable "gates", the gates turn from red to
% green. If all gates turn green, the simulation ends and the run time is
% recorded.
% 
% The student objective is to program a wall-following algorithm such that
% all the gates are completed as fast as possible.
%
% Written by Sean Brennan, sbrennan@psu.edu on 09 October, 2020.
% 

% Revision history:
% 2020_10_09 - first code write by cloning from the WallFollowing repo


clear all;
close all;

% Define a path to follow - this needs to be set by students. Here we
% define just a triangle

X1 = linspace(0,150,100)';
Y1 = -25*ones(length(X1),1);

Y2 = linspace(-25,100,100)';
X2 = 150*ones(length(Y2),1);

X3 = flipud(X1);
Y3 = interp1([0 150],[-25 100],X3);

pathXY = [X1 Y1; X2 Y2; X3 Y3];
% FOR DEBUGGING: plot the path?
% figure; plot(pathXY(:,1),pathXY(:,2),'k-');

% Convert the path into s-coordinates (s = sistance traveled)
dist = sum(diff(pathXY).^2,2).^0.5;
s_dist = [0; cumsum(dist)];
pathSXY = [s_dist pathXY];

% Remove repeated entries (causes problems)
pathSXY = unique(pathSXY,'rows');

% The sensorx, sensory, and sensor_angle variables set the location of the
% virtual sensor on the robot, so that we can "move it around" virtually to
% find a position and orientation that gives a good behavior. 

% STUDENTS: change the following 3 lines of code to put your sensor where
% you want to!
sensorx      = 4;  % Units are cm from midpoint between wheels
sensory      = 4;  % Units are cm from midpoint between wheels
sensor_angle_in_degrees = -40;  % Units are degrees
% Note: for angles, 0 degrees is pointing straight forward, angles are
% measured in typical CCW direction


delta_t = 0.01; %100 Hz sampling rate
% The plotting takes a lot of time, so we can set an integer here such that
% it only updates the plot every N times. Set N = 1 if we want to see plot
% for every iteration.
show_plot_of_robot_every_N = 100; 

% This is the time vector. The code loops through this so the upper limit
% of this vector sets the duration
t = 0:delta_t:500; 

IR_distance = 100; % Start off with a long-range reading
path_distance = 0;  % Start off with a zero path distance
IR_distance_old = IR_distance; % Start off with same "old" IR reading.
error_sum = 0; % Start off with zero sum of error.

% Set the gains and setpoints
Kp = 0.005;  % The proportional gain
Ki = 0.000;  % The integral gain
Kd = 0.000;  % The derivative gain
ref = 90;   % The IR reading to use as a setpoint

for i=1:length(t)
    time = t(i); % This is the current time
    
    % STUDENTS: Put your algorithm here to drive the robot toward the wall
    % and trying to stop. You get to change the torque based on previous
    % IR_distance reading, and using this and some smarts, try to get the
    % robot to park as close as you can to the wall at x = 300.

    % turn_command = 0.5;  % Positive commands turn to the left
    speed_command = 0.3; % This is the open-loop forward torque that sets speed


    % error = ref - IR_distance; % Calculate the error
    error = path_distance;
    error_sum = error_sum + error*delta_t;
    error_dot = (IR_distance - IR_distance_old)/delta_t;
    
    % PID control on steering
    turn_command = Kp*error + Ki*error_sum + Kd*error_dot;   % Calculate the turn command
    
    % Simple control on speed
    % if error<0 % Slow the robot down if about to hit something?
    %     speed_command = 0.1;
    % end
    
    % Limit the max and min values of the speed and turn command. Only
    % change these once you get working code (above), if you are trying to
    % make the robot move faster/faster. Note that -1 and 1 are the minimum
    % and max values respectively (larger magnitudes are ignored). These
    % limits keep silly things from happening with robot due to poor
    % controller designs for turning or speed.
    
    turn_command = min(turn_command,0.5);  
    turn_command = max(turn_command,-0.5);
    
    speed_command = min(speed_command,1);  
    speed_command = max(speed_command,-1);

        
    % Mix the speed and turn commands to determine actual L and R motor
    % torques
    torque_L = speed_command + turn_command;
    torque_R = speed_command - turn_command;
    
    % Call a function to simulate robot and grab the IR distance
    [IR_distance,path_distance] = fcn_simRobot(...
        torque_L,torque_R,sensorx,sensory,sensor_angle_in_degrees,...
        show_plot_of_robot_every_N,pathSXY);   
    
    % Update the title of the plot (note: this is SLOW - comment it out if
    % you get tired of watching the plot change)
    title(sprintf('IR reading is: %.f, Path distance is: %.f',IR_distance,path_distance));
    
    % Check to see if you slam into a wall: if so, the IR reading becomes
    % not-a-number, e.g. NaN!
    if isnan(IR_distance)
        break;
    end
    


end




