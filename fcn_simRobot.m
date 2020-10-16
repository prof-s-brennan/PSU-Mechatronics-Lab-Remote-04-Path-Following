function [IR_distance,path_distance] = fcn_simRobot(torque_L,torque_R,sensorx,sensory,sensor_angle_in_degrees,varargin)
% This is a function that performs a simulation of the robot
% Inputs:
%  torque_L: The torque on the left wheel of the robot (from -1 to 1)
%  torque_R: The torque on the right wheel of the robot (from -1 to 1)
%  sensorx:  The x-location on the robot of the sensor on the robot's body,
%            in cm. 
%            (NOTE: the origin is the midpoint between the wheels, X is
%            measured forward, Y is measured laterally relative to this
%            body-fixed coordinate system)
%  sensory:  The y-location of the sensor on the robot's body
%  sensor_angle_in_degrees: The angle of the sensor relative to straight
%            forward
%  VARIABLE ARGUMENTS:
%  plot_every: a variable that tells how often to plot, e.g. every N frames
%  pathSXY: the path a robot should follow.

%% Define initial variables
persistent robot;

if isempty(robot)
    robot.width        = 10;
    robot.length       = 12;
    robot.radius_squared = robot.width.^2 + robot.length^2;
    robot.wheel_center = 4;
    robot.wheel_length = 3;
    robot.wheel_width  = 2;
    robot.position_x   = 0;
    robot.position_y   = -20;
    robot.theta        = 0;  % Units are radians
    robot.r_wheelangle = 0;
    robot.l_wheelangle = 0;
    robot.sensorx      = sensorx;  % Units are cm from midpoint between wheels
    robot.sensory      = sensory;  % Units are cm from midpoint between wheels
    robot.sensor_angle = sensor_angle_in_degrees*pi/180;  % Units are radians
    robot.sensor_range = 100;  % Sensor range: units are cm
    robot.omega_L = 0; % Initial value of robot's left motor speed
    robot.omega_R = 0; % Initial value of robot's right motor speed
    robot.omega_Robot = 0; % Initially not moving
    robot.delta_t = 0.01; % Time step for 100 Hz sampling rate
    robot.plot_every = 1; % Plot every 1 time steps
    robot.current_plot_index = 1; % This is the current index of plotting
    robot.start_time = cputime;
    if nargin>6
        robot.pathSXY = varargin{2};
        robot.path_follow_mode = 1;
    else
        robot.path_follow_mode = 0;
    end
    robot.previous_s_coord = 0; % The total distance traveled along the path
    robot.s_estimate = 0;
    robot.lookahead_distance = 10;
    figure(1);
    clf;
    fcn_drawRobot(robot);  % Draw the first time
end

%% Check the inputs
if nargin>5
    robot.plot_every = max(round(varargin{1}),1);
end


%% Constraint the torque
torque_L = min(torque_L,1);
torque_L = max(torque_L,-1);
torque_R = min(torque_R,1);
torque_R = max(torque_R,-1);


%% Euler simulation of kinematics

% Define sum of torques
torque_coulomb_L = 0.01; % Assume 5 percent of torque is colomb
torque_coulomb_R = 0.01; % Assume 5 percent of torque is colomb
max_speed = 20; % Units are cm/second
max_rotational_velocity = (max_speed / (robot.wheel_length/2)); % Max rotational speed of motor, in rad/sec

% Left motor calculations from torque to angular acceleration
J_L = .02;
damping_L = 1 / max_rotational_velocity; % Assume 100% of torque (1) is required to get to max velocity
torque_friction_L = torque_coulomb_L * sign(robot.omega_L)   + robot.omega_L*damping_L;
torque_net_L = torque_L - torque_friction_L;
rot_accel_L = torque_net_L / J_L;

% Left motor calculations from torque to angular acceleration
J_R = .02;
damping_R = 1.05 / max_rotational_velocity; % Assume 100% of torque (1) is required to get to max velocity
torque_friction_R = torque_coulomb_R * sign(robot.omega_R)  + robot.omega_R*damping_R;
torque_net_R = torque_R - torque_friction_R;
rot_accel_R = torque_net_R / J_R;

% From sum(Torque) = rotational_inertia * angular_acceleration
robot.omega_L = robot.omega_L + rot_accel_L*robot.delta_t;  % Units are cm/s
robot.omega_R = robot.omega_R + rot_accel_R*robot.delta_t;  % Units are cm/s

% Linear velocities
robot.v_L = robot.omega_L*robot.wheel_length/2;
robot.v_R = robot.omega_R*robot.wheel_length/2;

% Total velocity
V = (robot.v_L + robot.v_R)/2;

% Add rotational inertia?
factor = 0.1;  % Set to 0.01 if want eigs of 1 second for rotation
robot.omega_Robot = factor*(robot.v_R - robot.v_L)/robot.width + (1-factor)*robot.omega_Robot;

% Update the robot based on kinematics
robot.l_wheelangle = robot.l_wheelangle + robot.omega_L*robot.delta_t; % Agular speed of left wheel (rad/sec)
robot.r_wheelangle = robot.r_wheelangle + robot.omega_R*robot.delta_t;  % Angular speed of wheel (rad/sec)
robot.theta = robot.theta + robot.omega_Robot*robot.delta_t; % Angular speed of robot (rad/sec)
robot.position_x = robot.position_x + V*cos(robot.theta)*robot.delta_t;
robot.position_y = robot.position_y + V*sin(robot.theta)*robot.delta_t;



% Update the robot's s-position
robot.s_estimate = robot.previous_s_coord + V*robot.delta_t;

% Calculate results of motion
if 0==robot.path_follow_mode
    robot.previous_s_coord = robot.s_estimate;
    IR_distance = fcn_drawRobot(robot,1);  % Redraw the robot in Figure 1
else  % Doing path-following motion
    
    % Find the path segment to query ahead    
    [path_segmentSXY,flag_outside_start, flag_outside_end] = ...
        fcn_pathtools_FindPathSXYSegmentContainingGivenSBounds(...
        robot.pathSXY,...
        robot.previous_s_coord,robot.s_estimate);
        
    % % Warn if this segment is invalid?
    % if flag_outside_start
    %     fprintf(2,'Outside start of path. Path station query is %.2f and path ends at %.2f\n',robot.previous_s_coord,robot.pathSXY(1,1));
    % end
    % if flag_outside_end
    %     fprintf(2,'Outside end of path. Path station query is %.2f and path ends at %.2f\n',robot.s_estimate,robot.pathSXY(end,1));
    % end
    
    
    %     % Find the path segment to query
    %     path_segment_start_index = find(robot.pathSXY(:,1)<robot.previous_s_coord,1,'last');
    %     path_segment_end_index   = find(robot.pathSXY(:,1)>robot.s_estimate,1,'first');
    %
    %     % Check if we've gone past the ends of the path
    %     if isempty(path_segment_start_index)
    %         path_segment_start_index = 1;
    %     end
    %     if isempty(path_segment_end_index)
    %         path_segment_end_index = length(robot.pathSXY(:,1));
    %     end
    %
    %     % Grab the path segment
    %     path_segmentSXY = robot.pathSXY(path_segment_start_index:path_segment_end_index,:);
        
    % Find the s-coordinate of the closest robot's location on the path
    % segment by snapping the robot back onto the path
    [~,robot.previous_s_coord] = ...
        fcn_pathtools_snap_point_onto_path(...
        [robot.position_x robot.position_y], path_segmentSXY);
        
    [IR_distance,path_distance] = fcn_drawRobot(robot,1);  % Redraw the robot in Figure 1
    
    %     % For debugging
    %     fprintf(1,'Previous and Current s: %.4f %.4f  Path distance: %.2f\n',robot.previous_s_coord,robot.s_estimate,path_distance);
    %     if robot.position_y > 100
    %         plot(path_segmentSXY(:,2),path_segmentSXY(:,3),'b-','Linewidth',3);
    %     end
end

% Increment the index
robot.current_plot_index = robot.current_plot_index + 1;

if isnan(IR_distance)
    return;
end

if mod(robot.current_plot_index,robot.plot_every)==0
    drawnow;
end
%pause(0.001);  % Not necessary - slows down sim


end


function [IR_distance,path_distance] = fcn_drawRobot(robot,varargin)
% fcn_drawRobot draws the robot
% Syntax:
% fcn_drawRobot(robot,varargin)
% Examples:
%           
% 
% This function was written on 2020_04_13 by S. Brennan
% Questions or comments? sbrennan@psu.edu 
%

% Revision history:
% 2020_10_10 - added variable arguments out to plot the path error

persistent arena;

if isempty(arena)
    % Specify the arena details
    %length_arena = 400;
    arena.wall_start = [-50 -50; 50 -50; 150 -150; 200 -150; 200 -200; 250 -200; 250 0; 200 0; 200 50; 250 50; 250 150; 0 250; 0 50; -50 0];
    arena.wall_end   = [arena.wall_start(2:end,:); arena.wall_start(1,:)];
    N_walls = length(arena.wall_start(:,1));
    
    arena.walls_x = [arena.wall_start(:,1) arena.wall_end(:,1) NaN*arena.wall_start(:,1)];
    arena.walls_y = [arena.wall_start(:,2) arena.wall_end(:,2) NaN*arena.wall_start(:,2)];
    arena.walls_x = reshape(arena.walls_x',N_walls*3,1);
    arena.walls_y = reshape(arena.walls_y',N_walls*3,1);
    
    
    %% Calculations to determine robot dimensions
    left_side  = -robot.width/2;
    right_side =  robot.width/2;
    front_side =  robot.wheel_center;
    rear_side  =  robot.wheel_center - robot.length;
    
    arena.body = [left_side front_side; right_side front_side; right_side rear_side; left_side rear_side];
    arena.right_wheel = [...
        right_side-robot.wheel_width/2  robot.wheel_length/2;
        right_side+robot.wheel_width/2  robot.wheel_length/2;
        right_side+robot.wheel_width/2 -robot.wheel_length/2;
        right_side-robot.wheel_width/2 -robot.wheel_length/2];
    arena.left_wheel = [...
        left_side-robot.wheel_width/2  robot.wheel_length/2;
        left_side+robot.wheel_width/2  robot.wheel_length/2;
        left_side+robot.wheel_width/2 -robot.wheel_length/2;
        left_side-robot.wheel_width/2 -robot.wheel_length/2];
    
    
    % Define dimensions of sensor
    sensor_width = 1.5;
    sensor_length = 0.5;
    
    % Create basic sensor box
    arena.sensor_box = [...
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
    arena.sensor_box = arena.sensor_box*R + [robot.sensorx robot.sensory];
    unit_sensor_vector = unit_sensor_vector*R;
    arena.sensor_vector = [robot.sensorx robot.sensory;...
        unit_sensor_vector*robot.sensor_range+[robot.sensorx robot.sensory]];
    
    % Show the preview horizon?
   
    if 1==robot.path_follow_mode
        % Define dimensions of preview sensor
        preview_box_width = 0.5;
        preview_box_length = 0.5;
    
        % Create basic sensor box
        arena.robot_preview_box = [...
        -preview_box_width/2  preview_box_length/2;
        +preview_box_width/2  preview_box_length/2;
        +preview_box_width/2 -preview_box_length/2;
        -preview_box_width/2 -preview_box_length/2];
            
        
        % Project the sensor via a unit vector
        unit_robot_preview_vector = [0 1]; % Unit x direction
        arena.robot_preview_vector = [0 0;...
            unit_robot_preview_vector*robot.lookahead_distance];
    else  % No preview, set it to zero
        arena.robot_preview_vector = [0 0; 0 0];
    end
        
    % Draw spokes?
    arena.spoke_interval = 60*pi/180;
    arena.base_spoke_angles = (0:arena.spoke_interval:2*pi)';
  
    arena.N_spokes = length(arena.base_spoke_angles);
    arena.robot_right_spoke_widths_start = (right_side-robot.wheel_width/2)*ones(arena.N_spokes,1);
    arena.robot_right_spoke_widths_end   = (right_side+robot.wheel_width/2)*ones(arena.N_spokes,1);
    arena.robot_left_spoke_widths_start  = (left_side-robot.wheel_width/2)*ones(arena.N_spokes,1);
    arena.robot_left_spoke_widths_end    = (left_side+robot.wheel_width/2)*ones(arena.N_spokes,1);
    
    % Define the target locations
    arena.targets_start = [50 -40; 140 -135; 165 -145; 205 -150; 230 -195; 245 -140; 200  -5; 195 55; 230 55; 240 145; 110 195; 10 225; 10 50;   -40 0; -10 -45];
    arena.targets_end  = [50 -20; 155 -120; 165 -125; 225 -150; 230 -180; 225 -140; 170 -20; 185 65; 230 70; 220 135;  90 175; 30 205; 30 50; -20 -15; -10 -25];
    arena.targets_midpoints = (arena.targets_start + arena.targets_end)/2;
    
    arena.N_targets = length(arena.targets_start(:,1));
    arena.target_was_hit = zeros(arena.N_targets,1);
    
    arena.targets_x = [arena.targets_start(:,1) arena.targets_end(:,1) NaN*arena.targets_start(:,1)];
    arena.targets_y = [arena.targets_start(:,2) arena.targets_end(:,2) NaN*arena.targets_start(:,2)];
    arena.targets_x = reshape(arena.targets_x',arena.N_targets*3,1);
    arena.targets_y = reshape(arena.targets_y',arena.N_targets*3,1);
    
    % Saved traces of robot position    
    arena.robot_positions = NaN*ones(50000,2);
    
end  % Ends if statement for first time in arena


%% Set up for debugging
do_debug = 0; % Flag to plot the results for debugging
 
if do_debug    
    fig_num = 2; %#ok<UNRCH>
    figure(fig_num);
    flag_make_new_plot = 1;
end

%% check input arguments
% Are the number of inputs correct?
if nargin < 1 || nargin > 3
    error('Incorrect number of input arguments.')
end

% Are we making a new plot? Check input arguments to see
flag_make_new_plot = 0; % Default is not to make a new plot

if 2<= nargin  % Check to see if plot number has been specified
    fig_num = varargin{1};
    % figure(fig_num);  % This might not be needed - plot commands go to it
    % automatically
else
    fig = gcf; % create new figure with next default index
    fig_num = get(fig,'Number');
    flag_make_new_plot = 1;
end


%% Move the spokes?
r_spoke_angles = arena.base_spoke_angles - robot.r_wheelangle;
l_spoke_angles = arena.base_spoke_angles - robot.l_wheelangle;

r_spoke_y = robot.wheel_length/2*cos(r_spoke_angles);
l_spoke_y = robot.wheel_length/2*cos(l_spoke_angles);

r_spokes = [arena.robot_right_spoke_widths_start, r_spoke_y, ...
    arena.robot_right_spoke_widths_end, r_spoke_y];
l_spokes = [ arena.robot_left_spoke_widths_start, l_spoke_y,...
    arena.robot_left_spoke_widths_end, l_spoke_y];

%% Rotate the robot to current theta
offset = pi/2;
R = [cos(robot.theta - offset) sin(robot.theta - offset);...
    -sin(robot.theta - offset)  cos(robot.theta - offset)];
   
body = arena.body*R;
right_wheel = arena.right_wheel*R;
left_wheel  = arena.left_wheel*R;
sensor_box = arena.sensor_box*R;
sensor_vector = arena.sensor_vector*R;
robot_preview_box = arena.robot_preview_box*R;
robot_preview_vector = arena.robot_preview_vector*R;
r_spokes = [r_spokes(:,1:2)*R, r_spokes(:,3:4)*R];
l_spokes = [l_spokes(:,1:2)*R, l_spokes(:,3:4)*R];

%% Translate the robot to current XY coordinates
body = body + [robot.position_x robot.position_y];
right_wheel = right_wheel + [robot.position_x robot.position_y];
left_wheel  = left_wheel + [robot.position_x robot.position_y];
sensor_box = sensor_box + [robot.position_x robot.position_y];
sensor_vector = sensor_vector + [robot.position_x robot.position_y];
robot_preview_box = robot_preview_box + [robot.position_x robot.position_y];
robot_preview_vector = robot_preview_vector + [robot.position_x robot.position_y];
r_spokes = r_spokes + [robot.position_x robot.position_y robot.position_x robot.position_y];
l_spokes = l_spokes + [robot.position_x robot.position_y robot.position_x robot.position_y];



%% Check where sensor vector hits the walls
[distance,location] = fcn_findSensorHit(arena.wall_start,arena.wall_end,sensor_vector);

% Check if there was a hit
if distance>0 && distance<robot.sensor_range
    sensor_vector(2,:)=location;
else
    distance = robot.sensor_range;
end

% Calculate the IR distance
IR_distance = fcn_emulateIRSensorDistance(distance);

%% Check if any of the robot is outside of the walls
flag_no_collision = 1;

% all_points = [body; right_wheel; left_wheel; sensor_box];
% if any(all_points(:,1)>300)
%     flag_no_collision = 0;
% elseif any(all_points(:,2)<-50)
%     flag_no_collision = 0;
% elseif any(all_points(:,2)>100)
%     flag_no_collision = 0;
% end    

% Check if any of the robot hits the walls
right_wheel_sensor = right_wheel(2:3,:);
left_wheel_sensor =  [left_wheel(1,:); left_wheel(4,:)];
body_front_sensor = body(1:2,:);
body_rear_sensor = body(3:4,:);

[distance_right_wheel,location_right_wheel] = fcn_findSensorHit(arena.wall_start,arena.wall_end,right_wheel_sensor);
[distance_left_wheel,location_left_wheel]   = fcn_findSensorHit(arena.wall_start,arena.wall_end,left_wheel_sensor);
[distance_body_front,location_body_front]   = fcn_findSensorHit(arena.wall_start,arena.wall_end,body_front_sensor);
[distance_body_rear,location_body_rear]     = fcn_findSensorHit(arena.wall_start,arena.wall_end,body_rear_sensor);

if ~isnan(distance_right_wheel)
    fprintf(1,'Impact detected on right wheel at location: x = %.2f, y = %.2f,\n',location_right_wheel(1,1),location_right_wheel(1,2));    
    flag_no_collision = 0;
end
if ~isnan(distance_left_wheel)
    fprintf(1,'Impact detected on left wheel at location: x = %.2f, y = %.2f,\n',location_left_wheel(1,1),location_left_wheel(1,2));    
    flag_no_collision = 0;
end
if ~isnan(distance_body_front)
    fprintf(1,'Impact detected on front of robot at location: x = %.2f, y = %.2f,\n',location_body_front(1,1),location_body_front(1,2));    
    flag_no_collision = 0;
end
if ~isnan(distance_body_rear)
    fprintf(1,'Impact detected on rear of robot at location: x = %.2f, y = %.2f,\n',location_body_rear(1,1),location_body_rear(1,2));    
    flag_no_collision = 0;
end


if flag_no_collision ==0
    IR_distance = NaN;
end

%% Check intersections with targets
distance_squared_to_targets = sum((arena.targets_midpoints - [robot.position_x robot.position_y]).^2,2);
[min_distance_squared,index_min] = min(distance_squared_to_targets);
if(min_distance_squared<robot.radius_squared)
    if arena.target_was_hit(index_min,1) == 0 % This is the first time it was found?
        arena.target_was_hit(index_min,1) = 1;
        plot([arena.targets_start(index_min,1) arena.targets_end(index_min,1)],...
            [arena.targets_start(index_min,2) arena.targets_end(index_min,2)],...
            'g','Linewidth',5);
    end
end

% OLD METHOD: (slow)
% for i=1:arena.N_targets
%     [distance_target,~]   = fcn_findSensorHit(arena.targets_start(i,:),arena.targets_end(i,:),body_front_sensor);
%     if ~isnan(distance_target)
%         arena.target_was_hit(i,1) = 1;
%         plot([arena.targets_start(i,1) arena.targets_end(i,1)],...
%             [arena.targets_start(i,2) arena.targets_end(i,2)],...
%             'g','Linewidth',5);
%     end
% end


%% Check distance to path?
if 1==robot.path_follow_mode
    
    % Find the path segment to query up ahead
    %     s_distance_to_search = robot.previous_s_coord + 3*robot.lookahead_distance;
    %     [path_segmentSXY,flag_outside_start, flag_outside_end] = ...
    %         fcn_pathtools_FindPathSXYSegmentContainingGivenSBounds(...
    %         robot.pathSXY,...
    %         robot.previous_s_coord,s_distance_to_search);
    s_distance_to_search = robot.previous_s_coord + robot.lookahead_distance;
    [path_segmentSXY,flag_outside_start, flag_outside_end] = ...
        fcn_pathtools_FindPathSXYSegmentContainingGivenSBounds(...
        robot.pathSXY,...
        s_distance_to_search,s_distance_to_search);
    
    % % Warn if this segment is invalid?
    % if flag_outside_start
    %     fprintf(2,'Outside start of path. Path station query is %.2f and path ends at %.2f\n',s_distance_to_search,path_segmentSXY(end,1));
    % end
    % if flag_outside_end
    %     fprintf(2,'Outside end of path. Path station query is %.2f and path ends at %.2f\n',s_distance_to_search,path_segmentSXY(end,1));
    % end
        
    
    %     path_segment_start_index = find(robot.pathSXY(:,1)<robot.previous_s_coord,1,'last');
    %     path_segment_end_index   = find(robot.pathSXY(:,1)>s_distance_to_search,1,'first');
    %
    %     % Check if we've gone past the ends of the path
    %     if isempty(path_segment_start_index)
    %         path_segment_start_index = 1;
    %     end
    %     if isempty(path_segment_end_index)
    %         path_segment_end_index = length(robot.pathSXY(:,1));
    %     end
    %
    %     % Check to see if end index is the start of path
    %     if 1 == path_segment_end_index
    %         path_segment_start_index = 1;
    %         path_segment_end_index = 2;
    %     end
    %
    %     % Check to see if start and end of segment are the same
    %     if path_segment_start_index == path_segment_end_index
    %         path_segment_start_index = path_segment_end_index - 1;
    %     end
    %
    %     % Grab the path segment that is closest to the robot's look-ahead point
    %     path_segmentSXY = robot.pathSXY(path_segment_start_index:path_segment_end_index,:);
    
    % Find the coordinates of the closest location on the path segment  
    robot_preview_point = robot_preview_vector(end,:);
    [path_snapped_point,path_station] = fcn_pathtools_snap_point_onto_path(robot_preview_point, path_segmentSXY);
    
    % FOR DEBUGGING:
    %     if robot.position_y > 100
    %         axis([120 175 80 125]);
    %         title(sprintf('Previous s, path station: %.4f %.4f  \n',robot.previous_s_coord,path_station));
    %         plot(path_segmentSXY(:,2),path_segmentSXY(:,3),'g-','Linewidth',3);
    %         plot(path_snapped_point(:,1),path_snapped_point(:,2),'c*');
    %         plot(robot_preview_point(:,1),robot_preview_point(:,2),'b*');
    %         plot(robot.position_x,robot.position_y,'ko');
    %     end
    
    % Calculate error in distance
    path_distance = sum((path_snapped_point - robot_preview_point).^2,2).^0.5;

    % Grab the segment where the robot's look-ahead point landed
    [path_segmentSXY_currently_on,flag_outside_start, flag_outside_end] = ...
        fcn_pathtools_FindPathSXYSegmentContainingGivenSBounds(...
        robot.pathSXY,...
        path_station,path_station);
    
    % % Warn if this segment is invalid?
    % if flag_outside_start
    %     fprintf(2,'Outside start of path. Path station query is %.2f and path ends at %.2f\n',path_station,path_segmentSXY_currently_on(end,1));
    % end
    % if flag_outside_end
    %     fprintf(2,'Outside end of path. Path station query is %.2f and path ends at %.2f\n',path_station,path_segmentSXY_currently_on(end,1));
    % end
    
    %     %%% TO DO - put the following into a subfunction
    %     % Find the start of path behind this lookahead point
    %     path_segment_index_behind_point = find(path_segmentSXY(:,1)<path_station,1,'last');
    %     if isempty(path_segment_index_behind_point)
    %         path_segment_index_behind_point = 1;
    %     end
    %
    %     % Find the end of path ahead of this lookahead point
    %     path_segment_index_ahead_point = find(path_segmentSXY(:,1)>path_station,1,'first');
    %     if isempty(path_segment_index_ahead_point)
    %         path_segment_index_ahead_point = length(path_segmentSXY(:,1));
    %     end
    %
    %
    %     %%% END SUBFUNCTION
    
    path_vector = ...
        path_segmentSXY_currently_on(2,2:3) - ...
        path_segmentSXY_currently_on(1,2:3);
    
        
    % Determine if error is positive or negative via cross product
    % to do this, we take the vector from the previous station to the next
    % station. This is the path vector. Then we take the vector from the
    % previous station to the robot's look-ahead point; this is the robot's
    % vector. We take the cross product of these.

    robot_vector = ...
        robot_preview_vector(end,:) - ...
        path_segmentSXY_currently_on(1,2:3);
    
    result = crossProduct(path_vector,robot_vector);    
    if result<0
        path_distance = path_distance*-1;
    end
    
    
else
    path_distance = 0;
end

%% Check to see if you won!
if all(arena.target_was_hit(:,1))
    fprintf(1,'Competition complete! Your simulated completion time is: %.2f seconds. Wall time simulation duration was: %.2f seconds.\n',(robot.current_plot_index*robot.delta_t), (cputime-robot.start_time));
    IR_distance = NaN;  % Force the sim to exit
end



%% Save the robot position
arena.robot_positions(robot.current_plot_index,:) = [robot.position_x robot.position_y];


%% Plot input results
if flag_make_new_plot 
    figure(fig_num);
    hold on;
    axis equal;
    grid on; grid minor;
    
    % Plot arena (first time)
    plot(arena.walls_x,arena.walls_y,'k','Linewidth',5);
    
    % Plot targets (first time)
    plot(arena.targets_x,arena.targets_y,'r','Linewidth',5);
    
    % Plot path? (first time)
    if 1==robot.path_follow_mode        
        plot(robot.pathSXY(:,2),robot.pathSXY(:,3),'k--','Linewidth',2);
    end
    
    % Plot the robot parts the first time
    handles.h_robot_body = plot_box(0,0,body,'k');  % body
    handles.h_robot_rwheel = plot_box(0,0,right_wheel,'r');  % R Wheel
    handles.h_robot_lwheel = plot_box(0,0,left_wheel,'b');  % L Wheel
    handles.h_robot_sensor = plot_sensor(0,0,sensor_box,sensor_vector,'g');  % Sensor
    handles.h_robot_preview_box = plot_sensor(0,0,robot_preview_box,robot_preview_vector,'m');  % Look-ahead point
    handles.h_robot_rspokes = plot_spokes(0,0,r_spokes,r_spoke_angles,'r'); % Rwheel spokes
    handles.h_robot_lspokes = plot_spokes(0,0,l_spokes,l_spoke_angles,'b'); % Lwheel spokes

    % Drop an ant-trail
    handles.h_robot_path = plot(arena.robot_positions(:,1),arena.robot_positions(:,2),'c.');

    
    set(fig_num,'UserData',handles)
else % Update the line positions
    if mod(robot.current_plot_index,robot.plot_every)==0
        handles = get(fig_num,'UserData');
        handles.h_robot_body = plot_box(1,handles.h_robot_body,body,'k');  % body
        handles.h_robot_rwheel = plot_box(1,handles.h_robot_rwheel,right_wheel,'r');  % R Wheel
        handles.h_robot_lwheel = plot_box(1,handles.h_robot_lwheel,left_wheel,'b');  % L Wheel
        handles.h_robot_sensor = plot_sensor(1,handles.h_robot_sensor,sensor_box,sensor_vector,'g');  % Sensor
        handles.h_robot_preview_box = plot_sensor(1,handles.h_robot_preview_box,robot_preview_box,robot_preview_vector,'m');  % Look-ahead point
        handles.h_robot_rspokes = plot_spokes(1,handles.h_robot_rspokes,r_spokes,r_spoke_angles,'r'); % Rwheel spokes
        handles.h_robot_lspokes = plot_spokes(1,handles.h_robot_lspokes,l_spokes,l_spoke_angles,'b'); % Lwheel spokes

        % Update the ant-trail
        set(handles.h_robot_path,'Xdata',arena.robot_positions(:,1));
        set(handles.h_robot_path,'Ydata',arena.robot_positions(:,2));
        
        set(fig_num,'UserData',handles)
    end
    
end

% if nargout>1
%     path_distance
% end

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


end % Ends plot_sensor function

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


function IR_distance = fcn_emulateIRSensorDistance(distance)   
% IR_distance = fcn_emulateIRSensorDistance(distance)
% Syntax:
% IR_distance = fcn_emulateIRSensorDistance(distance)
% Examples:
     
% 
% This function was written on 2020_04_13 by S. Brennan
% Questions or comments? sbrennan@psu.edu 
% 

persistent distance_array

if isempty(distance_array)
    % Convert distance
    numerator = 200*30;
    crossover = 6;  % Units are centimeters
    offset = 20;
    IR_distance_at_crossover = numerator./(crossover+offset);
   
    distance_array = (0:0.1:100)';
    distance_array = [distance_array zeros(length(distance_array(:,1)),1)];
    Npoints = length(distance_array(:,1));
    for i=1:Npoints
        distance_to_fill = distance_array(i,1);
        if distance_to_fill<crossover
            IR_distance = distance_to_fill/crossover * IR_distance_at_crossover;
        else
            IR_distance = numerator./(distance_to_fill+offset);
        end
        distance_array(i,2) = IR_distance;
    end
    
    for i=1:Npoints-10
        distance_array(i,2) = mean(distance_array(i:i+10,2));
    end
end
%IR_distance = interp1(distance_array(:,1),distance_array(:,2),distance) + 3*randn(length(distance(:,1)),1);
IR_index = max(min(round(distance*10),1000),1);
IR_distance =  distance_array(IR_index,2) + 3*randn(length(distance(:,1)),1);                                           


end



function [distance,location] = fcn_findSensorHit(P,wall_end,sensor_vector,varargin)   
% fcn_findSensorHit calculates hits between sensor vector and walls
% Syntax:
% fcn_findSensorHit(arena.wall_start,arena.wall_end,sensor_vector,varargin) 
% Examples:
%      
%    % BASIC example - find all the points
%      
% 
% This function was written on 2020_04_13 by S. Brennan
% Questions or comments? sbrennan@psu.edu 
% 
% Adopted from https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect

%% Set up for debugging
do_debug = 0; % Flag to plot the results for debugging
 
if do_debug    
    fig_num = 23333;  %#ok<UNRCH>
    figure(fig_num);
    clf;
    hold on;
    axis equal;
    grid on; grid minor;
    
    N_walls = length(P(:,1));
    walls_x = [P(:,1) wall_end(:,1) NaN*P(:,1)];
    walls_y = [P(:,2) wall_end(:,2) NaN*P(:,2)];
    walls_x = reshape(walls_x',N_walls*3,1);
    walls_y = reshape(walls_y',N_walls*3,1);
        
    plot(walls_x,walls_y,'k','Linewidth',1);
    plot(sensor_vector(:,1),sensor_vector(:,2),'g');
    
end
% 
% %% check input arguments
% if nargin < 1 || nargin > 2
%     error('Incorrect number of input arguments.')
% end
% 
% flag_make_new_plot = 0; % Default is not to make a new plot
% if 2 == nargin
%     fig_num = varargin{1};
%     figure(fig_num);
% else
%     fig = gcf; % create new figure with next default index
%     fig_num = get(fig,'Number');
%     flag_make_new_plot = 1;
% end



%% Calculations begin here
% Define p, q, r and s vectors
q = sensor_vector(1,:);
r = wall_end - P;
s = sensor_vector(2,:)-sensor_vector(1,:);


r_cross_s = crossProduct(r,s);
q_minus_p =  q - P;

q_minus_p_cross_s = crossProduct(q_minus_p,s);
q_minus_p_cross_r = crossProduct(q_minus_p,r);

parallel_indices = find(0==r_cross_s);
if any(parallel_indices)
    r_cross_s(parallel_indices) = 1; % They are colinear or parallel, so make dummy length
end
   
t = q_minus_p_cross_s./r_cross_s;
u = q_minus_p_cross_r./r_cross_s;

t(parallel_indices) = inf;
u(parallel_indices) = inf;

intersection = NaN*ones(length(P(:,1)),2);

good_vector = ((0<t).*(1>t).*(0<u).*(1>u));
good_indices = find(good_vector>0);
if ~isempty(good_indices)
    result = P + t.*r; 
    intersection(good_indices,:) = result(good_indices,:);
    %plot(intersection(:,1),intersection(:,2),'rx');
end

distances_squared = sum((intersection - sensor_vector(1,:)).^2,2);
[best,best_index] = min(distances_squared);

distance = best^0.5;
location = intersection(best_index,:);

% FOR BACKUP
% %% Calculations begin here
% % Define p, q, r and s vectors
% p = wall_start;
% q = sensor_vector(1,:);
% r = wall_end - wall_start;
% s = sensor_vector(2,:)-sensor_vector(1,:);
% 
% 
% r_cross_s = crossProduct(r,s);
% q_minus_p =  q - p;
% 
% q_minus_p_cross_s = crossProduct(q_minus_p,s);
% q_minus_p_cross_r = crossProduct(q_minus_p,r);
% 
% parallel_indices = find(0==r_cross_s);
% if any(parallel_indices)
%     r_cross_s(parallel_indices) = 1; % They are colinear or parallel, so make dummy length
% end
%    
% t = q_minus_p_cross_s./r_cross_s;
% u = q_minus_p_cross_r./r_cross_s;
% 
% t(parallel_indices) = inf;
% u(parallel_indices) = inf;
% 
% intersection = NaN*ones(length(p(:,1)),2);
% 
% good_vector = ((0<t).*(1>t).*(0<u).*(1>u));
% good_indices = find(good_vector>0);
% if ~isempty(good_indices)
%     result = p + t.*r; 
%     intersection(good_indices,:) = result(good_indices,:);
%     %plot(intersection(:,1),intersection(:,2),'rx');
% end
% 
% distances_squared = sum((intersection - sensor_vector(1,:)).^2,2);
% [best,best_index] = min(distances_squared);
% 
% distance = best^0.5;
% location = intersection(best_index,:);

end




function [closest_path_point,s_coordinate] = fcn_pathtools_snap_point_onto_path(point, path,varargin)
% fcn_pathtools_snap_point_onto_path
% Finds location on a path that is closest to a given point, e.g. snapping
% the point onto the path
% 
% Format: 
% [closest_path_point,s_coordinate] = fcn_pathtools_snap_point_onto_path(point, path,varargin)
%
% INPUTS:
%      point: a 1x2 vector containing the [X Y] location of the point
%      path: a Nx2 vector of [X Y] path points, where N is the number of points the points on the path, N >= 2. 
%      (optional) figure_number: number where results are plotted
%
% OUTPUTS:
%      closest_path_point: a 1x2 vector containing the [X Y] location of
%      the nearest point on the path
%      s_coordinate: a scalar (1x1) representing the s-coordinate distance
%      along the path
%
% Examples:
%      
%      % BASIC example
%      point = [0.5 0.2];
%      path = [0 0 0; 1 1 0; 2 2 0; 3 2 1];
%      fignum = 222;
%      [closest_path_point,s_coordinate] = ...
%      fcn_pathtools_snap_point_onto_path(point, path,fignum)
% 
% See the script: script_test_fcn_pathtools_snap_point_onto_path
% for a full test suite.
%
% This function was written on 2020_10_10 by S. Brennan
% Questions or comments? sbrennan@psu.edu 

% Revision history:
% 2020_10_10 - first write of the code
%

% TO-DO:
% Allow multiple points, e.g.
%      point: a Nx2 vector where N is the number of points, but at least 1. 

flag_do_debug = 0; % Flag to plot the results for debugging
flag_check_inputs = 1; % Flag to perform input checking

%% check input arguments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _       
%  |_   _|                 | |      
%    | |  _ __  _ __  _   _| |_ ___ 
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |                  
%              |_| 
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Are the input vectors the right shape?
Npoints = length(path(:,1));

if flag_check_inputs == 1
    % Are there the right number of inputs?
    if nargin < 2 || nargin > 3
        error('Incorrect number of input arguments')
    end
    
    if Npoints<2
        error('The path vector must have at least 2 rows, with each row representing a different (x y) point');
    end
    if length(point(1,:))~=2
        error('The point vector must have 2 columns, with column 1 representing the x portions of the points, column 2 representing the y portions.');
    end
end

% Does user want to show the plots?
if 3 == nargin
    fig_num = varargin{1};
    figure(fig_num);
    flag_do_debug = 1;
else
    if flag_do_debug
        fig = figure;  %#ok<UNRCH>
        fig_num = fig.Number;
    end
end

%% Find the closest point
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
% The solution method is as follows:
%  1. Find the closest point on the path to the query point
%  2. Find next closest point to that one to the query point
%  3. Sort these to find which is first/second
%  4. Find percentage of travel using dot products


% Find distance from point to path
distances_point_to_path = sum((path(:,2:3)-point).^2,2).^0.5; % Calculate distances
[~,closest_path_point_index] = min(distances_point_to_path); % Grab closest point

% Find next closest point - be sure to check end cases as well
if 1 == closest_path_point_index
    next_closest_path_point_index = 2;
elseif Npoints == closest_path_point_index
    next_closest_path_point_index = Npoints-1;
elseif distances_point_to_path(closest_path_point_index+1)<distances_point_to_path(closest_path_point_index-1)
    next_closest_path_point_index = closest_path_point_index+1;
else
    next_closest_path_point_index = closest_path_point_index-1;
end
first_path_point_index = min(closest_path_point_index,next_closest_path_point_index);
second_path_point_index = max(closest_path_point_index,next_closest_path_point_index);



% Do the dot products - define the vectors first
% See: https://mathinsight.org/dot_product for explanation
% Basically, we are seeing what amount the point_vector points in the
% direction of the path_vector
path_vector = path(second_path_point_index,2:3)-path(first_path_point_index,2:3);
path_segment_length = sum(path_vector.^2,2).^0.5;
point_vector = point-path(first_path_point_index,2:3);
projection_distance = dot(path_vector,point_vector)/path_segment_length; % Do dot product

% URHERE
% % If dot product is negative, then going the wrong way! Not sure what to
% % do, but an obvious fix is to flip the sign
% if projection_distance<0
%     projection_distance = -1*projection_distance;
% end

% Calculate the percentage distance
percent_along_length = projection_distance/path_segment_length;

% Keep percent along length to positive values only
percent_along_length = max(percent_along_length,0);

% Calculate the outputs
closest_path_point = path(first_path_point_index,2:3) + path_vector*percent_along_length;
s_coordinate = path(first_path_point_index,1) + path_segment_length*percent_along_length;

% PLOT result?
if 1==0
    plot(point(:,1), point(:,2),'ks');
    plot([path(first_path_point_index,2) path(second_path_point_index,2)],...
        [path(first_path_point_index,3) path(second_path_point_index,3)],'r',...
        'Linewidth',10);
end

%% Plot the results (for debugging)?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _                 
%  |  __ \     | |                
%  | |  | | ___| |__  _   _  __ _ 
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/ 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag_do_debug
    figure(fig_num);
    hold on;
    grid on;
    % Plot the path
    plot(path(:,2),path(:,3),'r-');       
    plot(path(:,2),path(:,3),'ro');       
    
    axis equal;
    
    % Plot the query point
    plot(point(:,1),point(:,2),'ko');
    
    % Plot the closest path points;
    plot(...
        path(first_path_point_index:second_path_point_index,2),...
        path(first_path_point_index:second_path_point_index,3),'r*');       
    
    % Label the points with distances
    for i_point = 1:length(path(:,1))
        text(path(i_point,2),path(i_point,3),sprintf('%.2f',distances_point_to_path(i_point)));
    end
    
    % Plot the closest point on path
    plot(closest_path_point(:,1),closest_path_point(:,2),'go');
    
    % Connect closest point on path to query point
    plot(...
        [point(:,1) closest_path_point(:,1)],...
        [point(:,2) closest_path_point(:,2)],'g-');
    
    
end % Ends the flag_do_debug if statement



end % Ends the function




%% Calculate cross products
function result = crossProduct(v,w)
result = v(:,1).*w(:,2)-v(:,2).*w(:,1);
end





function [pathSXY_segment,flag_outside_start, flag_outside_end] = ...
    fcn_pathtools_FindPathSXYSegmentContainingGivenSBounds(...
    pathSXY, s_coord_start,s_coord_end, varargin)
% fcn_pathtools_FindPathSegmentWithinSBounds
% Finds portion of a SXY-type path that contains the given s_coordinates, starting
% from s_coord_start to s_coord_end
% 
% Format: 
% [closest_path_point,s_coordinate] = fcn_pathtools_snap_point_onto_path(point, path,varargin)
%
% INPUTS:
%      path: a Nx3 vector of [S X Y] path points, where N is the number of
%      points the points on the path, N >= 2. S is the station distance,
%      and X and Y are the XY points of the path coordinates
%
%      s_coord_start: a 1x1 (scalar) indicating the s-coordinate location
%      at which the query starts. The path segment output will start at
%      previous s-value to this station.
%
%      s_coord_end: a 1x1 (scalar) indicating the s-coordinate location
%      at which the query ends. The path segment output will end at
%      subsequent s-value to this station.
%
%      If the query includes s-values outside the path, then the flag_
%      variables are set to 1. Note that this function always returns at
%      least 2 points representing the closest path segment, even if both
%      s-point queries are outside the given path.
%
%      (optional_input) figure_number: plots the results into the given
%      figure
%
% OUTPUTS:
%      path_segment: a Mx3 vector of [S X Y] path points, where M is the
%      number of points the points on the path segment, M >= 2. S is the
%      station distance, and X and Y are the XY points of the path
%      coordinates
%
%      flag_outside_start, flag_outside_end: flags that are set equal to 1
%      if the query is outside the s-distance within the given path at
%      either the start, the end, or both
%
% Examples:
%      
%      % BASIC example
%      path = [0 0 0; 1 1 0; 2 2 0; 3 2 1];
%      fignum = 222;
%      [closest_path_point,s_coordinate] = ...
%      fcn_pathtools_snap_point_onto_path(point, path,fignum)
% 
% See the script: 
% SCRIPT_test_fcn_pathtools_FindPathSXYSegmentContainingGivenSBounds.m
% for a full test suite.
%
% This function was written on 2020_10_14 by S. Brennan
% Questions or comments? sbrennan@psu.edu 

% Revision history:
% 2020_10_14 - first write of the code
%

flag_do_debug = 0; % Flag to plot the results for debugging
flag_check_inputs = 1; % Flag to perform input checking

%% check input arguments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _       
%  |_   _|                 | |      
%    | |  _ __  _ __  _   _| |_ ___ 
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |                  
%              |_| 
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Are the input vectors the right shape?
Npoints_in_path = length(pathSXY(:,1));

if flag_check_inputs == 1
    % Are there the right number of inputs?
    if nargin < 3 || nargin > 4
        error('Incorrect number of input arguments')
    end
    
    if Npoints_in_path<2
        error('The path vector must have at least 2 rows, with each row representing a different (x y) point');
    end
    if length(pathSXY(1,:))~=3
        error('The path vector must have 3 columns, with column 1 representing the s-distance, column 2 representing the x portions of the points, column 3 representing the y portions.');
    end
    
    if s_coord_start>s_coord_end
        warning('S coordinates of start and end seem out of order. These will be automatically sorted but the results may be incorrect.');
        s_coord_start_new = s_coord_end;
        s_coord_end = s_coord_start;
        s_coord_start = s_coord_start_new;
    end
end

% Does user have special variable inputs?
if 4 == nargin
    fig_num = varargin{1};
    figure(fig_num);
    flag_do_debug = 1;
else
    if flag_do_debug
        fig = figure;  %#ok<UNRCH>
        fig_num = fig.Number;
    end
end

%% Find the closest point
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set default outputs
flag_outside_start = 0;
flag_outside_end   = 0;

% Use find function to grab values
path_segment_start_index = find(pathSXY(:,1)<s_coord_start,1,'last');
path_segment_end_index   = find(pathSXY(:,1)>s_coord_end,1,'first');

% Check if we've gone past the ends of the path
if isempty(path_segment_start_index) % There is no s-coordinate in the path smaller than the start
    path_segment_start_index = 1;
    flag_outside_start = 1;
end
if isempty(path_segment_end_index) % There is no s-coordinate in the path larger than the end
    path_segment_end_index = Npoints_in_path;
    flag_outside_end   = 1;
end

% Check to see if start index is the end of path
if Npoints_in_path == path_segment_start_index  % All path s-coordinates are smaller than the start
    path_segment_start_index = Npoints_in_path - 1;
    path_segment_end_index = Npoints_in_path;
end

% Check to see if end index is the start of path
if 1 == path_segment_end_index  % All path s-coordinates are larger than the end
    path_segment_start_index = 1;
    path_segment_end_index = 2;
end

% Check to see if start and end of segment are the same (degenerate case)
if path_segment_start_index == path_segment_end_index
    path_segment_start_index = path_segment_end_index - 1;
end

% Grab the path segment that is closest to the robot's look-ahead point
pathSXY_segment = pathSXY(path_segment_start_index:path_segment_end_index,:);


%% Plot the results (for debugging)?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _                 
%  |  __ \     | |                
%  | |  | | ___| |__  _   _  __ _ 
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/ 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag_do_debug
    figure(fig_num);
    hold on;
    grid on;
    % Plot the path
    plot(pathSXY(:,2),pathSXY(:,3),'r-','Linewidth',5);       
    plot(pathSXY(:,2),pathSXY(:,3),'ro','Linewidth',5);       
    
    axis equal;
    
    % Plot the results
    plot(pathSXY_segment(:,2),pathSXY_segment(:,3),'b-','Linewidth',3);
    plot(pathSXY_segment(1,2),pathSXY_segment(1,3),'b*');   
    plot(pathSXY_segment(end,2),pathSXY_segment(end,3),'b*');   
    text(pathSXY_segment(1,2),pathSXY_segment(1,3),'Start');   
    text(pathSXY_segment(end,2),pathSXY_segment(end,3),'End');     
    
end % Ends the flag_do_debug if statement



end % Ends the function



