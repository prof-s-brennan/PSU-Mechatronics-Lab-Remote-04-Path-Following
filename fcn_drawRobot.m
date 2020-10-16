function IR_distance = fcn_drawRobot(robot,varargin)
% fcn_drawRobot draws the robot
% Syntax:
% fcn_drawRobot(robot,varargin)
% Examples:
%      
%    % BASIC example - find all the points
%      
% 
% This function was written on 2020_04_13 by S. Brennan
% Questions or comments? sbrennan@psu.edu 
%

%% Set up for debugging
do_debug = 0; % Flag to plot the results for debugging
 
if do_debug    
    fig_num = 2; %#ok<UNRCH>
    figure(fig_num);
    flag_make_new_plot = 1;
end

%% check input arguments
if nargin < 1 || nargin > 2
    error('Incorrect number of input arguments.')
end

flag_make_new_plot = 0; % Default is not to make a new plot
if 2 == nargin
    fig_num = varargin{1};
    figure(fig_num);
else
    fig = gcf; % create new figure with next default index
    fig_num = get(fig,'Number');
    flag_make_new_plot = 1;
end


%% Draw the arena
length_arena = 400;
wall_start = [-50 -50; -50 100; 300 -50];
wall_end   = [length_arena -50; length_arena 100; 300 100];
N_walls = length(wall_start(:,1));

walls_x = [wall_start(:,1) wall_end(:,1) NaN*wall_start(:,1)];
walls_y = [wall_start(:,2) wall_end(:,2) NaN*wall_start(:,2)];
walls_x = reshape(walls_x',N_walls*3,1);
walls_y = reshape(walls_y',N_walls*3,1);


%% Calculations to determine robot dimensions
left_side  = -robot.width/2;
right_side =  robot.width/2;
front_side =  robot.wheel_center;
rear_side  =  robot.wheel_center - robot.length;

body = [left_side front_side; right_side front_side; right_side rear_side; left_side rear_side];
right_wheel = [...
    right_side-robot.wheel_width/2  robot.wheel_length/2; 
    right_side+robot.wheel_width/2  robot.wheel_length/2; 
    right_side+robot.wheel_width/2 -robot.wheel_length/2; 
    right_side-robot.wheel_width/2 -robot.wheel_length/2];
left_wheel = [...
    left_side-robot.wheel_width/2  robot.wheel_length/2; 
    left_side+robot.wheel_width/2  robot.wheel_length/2; 
    left_side+robot.wheel_width/2 -robot.wheel_length/2; 
    left_side-robot.wheel_width/2 -robot.wheel_length/2];

% Define dimensions of sensor
sensor_width = 1.5;
sensor_length = 0.5;

% Create basic sensor box
sensor_box = [...
    -sensor_width/2  sensor_length/2; 
    +sensor_width/2  sensor_length/2; 
    +sensor_width/2 -sensor_length/2; 
    -sensor_width/2 -sensor_length/2];

% Project the sensor via a unit vector
unit_sensor_vector = [0 1];


% Rotate it by the sensor angle and translate sensor into place
sensor_offset = 0;
R = [cos(robot.sensor_angle - sensor_offset) sin(robot.sensor_angle - sensor_offset);...
    -sin(robot.sensor_angle - sensor_offset)  cos(robot.sensor_angle - sensor_offset)];
sensor_box = sensor_box*R + [robot.sensorx robot.sensory];
unit_sensor_vector = unit_sensor_vector*R;
sensor_vector = [robot.sensorx robot.sensory;...
    unit_sensor_vector*robot.sensor_range+[robot.sensorx robot.sensory]];

% Draw spokes?
spoke_interval = 20*pi/180;
r_spoke_angles = (0:spoke_interval:2*pi)' - robot.r_wheelangle;
l_spoke_angles = (0:spoke_interval:2*pi)' - robot.l_wheelangle;

N_spokes = length(r_spoke_angles);

r_spoke_y = robot.wheel_length/2*cos(r_spoke_angles);
l_spoke_y = robot.wheel_length/2*cos(l_spoke_angles);

r_spokes = [(right_side-robot.wheel_width/2)*ones(N_spokes,1), r_spoke_y, ...
    (right_side+robot.wheel_width/2)*ones(N_spokes,1) r_spoke_y];
l_spokes = [(left_side-robot.wheel_width/2)*ones(N_spokes,1), l_spoke_y,...
    (left_side+robot.wheel_width/2)*ones(N_spokes,1) l_spoke_y];



%% Rotate the robot to current theta
offset = pi/2;
R = [cos(robot.theta - offset) sin(robot.theta - offset);...
    -sin(robot.theta - offset)  cos(robot.theta - offset)];

body = body*R;
right_wheel = right_wheel*R;
left_wheel  = left_wheel*R;
sensor_box = sensor_box*R;
sensor_vector = sensor_vector*R;
r_spokes = [r_spokes(:,1:2)*R, r_spokes(:,3:4)*R];
l_spokes = [l_spokes(:,1:2)*R, l_spokes(:,3:4)*R];

%% Translate the robot to current XY coordinates
body = body + [robot.position_x robot.position_y];
right_wheel = right_wheel + [robot.position_x robot.position_y];
left_wheel  = left_wheel + [robot.position_x robot.position_y];
sensor_box = sensor_box + [robot.position_x robot.position_y];
sensor_vector = sensor_vector + [robot.position_x robot.position_y];
r_spokes = r_spokes + [robot.position_x robot.position_y robot.position_x robot.position_y];
l_spokes = l_spokes + [robot.position_x robot.position_y robot.position_x robot.position_y];

% Check where sensor vector hits the walls
[distance,location] = fcn_findSensorHit(wall_start,wall_end,sensor_vector,varargin);

% Check if there was a hit
if distance>0 && distance<robot.sensor_range
    sensor_vector(2,:)=location;
else
    distance = robot.sensor_range;
end

% Calculate the IR distance
IR_distance = fcn_emulateIRSensorDistance(distance);

%% Check if any of the robot is outside of the walls
all_points = [body; right_wheel; left_wheel; sensor_box];
if any(all_points(:,1)>300)
    IR_distance = NaN;
elseif any(all_points(:,2)<-50)
    IR_distance = NaN;
elseif any(all_points(:,2)>100)
    IR_distance = NaN;
end    

%% Plot input results
if flag_make_new_plot
    figure(fig_num);
    hold on;
    axis equal;
    grid on; grid minor;
    
    % Plot arena (first time)         
    plot(walls_x,walls_y,'k','Linewidth',5);
    
    % Plot the robot parts the first time
    handles.h_robot_body = plot_box(0,0,body,'k');  % Body
    handles.h_robot_rwheel = plot_box(0,0,right_wheel,'r');  % R Wheel
    handles.h_robot_lwheel = plot_box(0,0,left_wheel,'b');  % L Wheel
    handles.h_robot_sensor = plot_sensor(0,0,sensor_box,sensor_vector,'g');  % Sensor
    handles.h_robot_rspokes = plot_spokes(0,0,r_spokes,r_spoke_angles,'r'); % Rwheel spokes
    handles.h_robot_lspokes = plot_spokes(0,0,l_spokes,l_spoke_angles,'b'); % Lwheel spokes
    set(fig_num,'UserData',handles)
else % Update the line positions
    handles = get(fig_num,'UserData');
    handles.h_robot_body = plot_box(1,handles.h_robot_body,body,'k');  % Body
    handles.h_robot_rwheel = plot_box(1,handles.h_robot_rwheel,right_wheel,'r');  % R Wheel
    handles.h_robot_lwheel = plot_box(1,handles.h_robot_lwheel,left_wheel,'b');  % L Wheel
    handles.h_robot_sensor = plot_sensor(1,handles.h_robot_sensor,sensor_box,sensor_vector,'g');  % Sensor
    handles.h_robot_rspokes = plot_spokes(1,handles.h_robot_rspokes,r_spokes,r_spoke_angles,'r'); % Rwheel spokes
    handles.h_robot_lspokes = plot_spokes(1,handles.h_robot_lspokes,l_spokes,l_spoke_angles,'b'); % Lwheel spokes
    set(fig_num,'UserData',handles)
    
end

end % Ends the function





%% Functions for plotting

function h_box = plot_box(flag_plot_exists, h_box_old, corners, varargin)    % plot all the boxes

plot_str = 'b-';
plot_type = 1;

if 4 == nargin
    plot_str = varargin{1};
    if isnumeric(plot_str)
        plot_type = 2;
    end
end

xdata = [corners(:,1); corners(1,1)];
ydata = [corners(:,2); corners(1,2)];

% Check if plot already exists
if ~flag_plot_exists  % It does not exist, create it then
    if plot_type==1
        h_box = plot(xdata,ydata,plot_str);
    elseif plot_type==2
        h_box = plot(xdata,ydata,'Color',plot_str);
    end
else % It exists already
    set(h_box_old,'XData',xdata);
    set(h_box_old,'YData',ydata);
    h_box = h_box_old;
end


end % Ends plot_box function


function h_sensor = plot_sensor(flag_plot_exists, h_box_old, corners, sensor_vector, varargin)    % plot sensor

plot_str = 'b-';
plot_type = 1;

if 5 == nargin
    plot_str = varargin{1};
    if isnumeric(plot_str)
        plot_type = 2;
    end
end

xdata = [corners(:,1); corners(1,1); NaN; sensor_vector(:,1)];
ydata = [corners(:,2); corners(1,2); NaN; sensor_vector(:,2)];

% Check if plot already exists
if ~flag_plot_exists  % It does not exist, create it then
    if plot_type==1
        h_sensor = plot(xdata,ydata,plot_str);
    elseif plot_type==2
        h_sensor = plot(xdata,ydata,'Color',plot_str);
    end
else % It exists already
    set(h_box_old,'XData',xdata);
    set(h_box_old,'YData',ydata);
    h_sensor = h_box_old;
end


end % Ends plot_box function

function h_spokes = plot_spokes(flag_plot_exists, h_spokes_old, spokes, spoke_angles, varargin)    % plot all the spokes

plot_str = 'b-';
plot_type = 1;

if 5 == nargin
    plot_str = varargin{1};
    if isnumeric(plot_str)
        plot_type = 2;
    end
end

% Tag hidden angles to hide them
spoke_angles = mod(spoke_angles,2*pi);
spokes(spoke_angles>pi,:) = NaN;


N_spokes = length(spokes(:,1));
xmatrix = [spokes(:,1), spokes(:,3), NaN*spokes(:,1)];

xdata = reshape(xmatrix',N_spokes*3,1);

ymatrix = [spokes(:,2), spokes(:,4), NaN*spokes(:,2)];
ydata = reshape(ymatrix',N_spokes*3,1);

% Check if plot already exists
if ~flag_plot_exists  % It does not exist, create it then
    if plot_type==1
        h_spokes = plot(xdata,ydata,plot_str);
    elseif plot_type==2
        h_spokes = plot(xdata,ydata,'Color',plot_str);
    end
else % It exists already
    set(h_spokes_old,'XData',xdata);
    set(h_spokes_old,'YData',ydata);
    h_spokes = h_spokes_old;
end


end % Ends plot_spokes function




