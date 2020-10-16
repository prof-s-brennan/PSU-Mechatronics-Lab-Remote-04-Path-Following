% script_test_fcn_drawRobot
% Exercises the function to make sure it works


robot.width        = 10;
robot.length       = 12; 
robot.wheel_center = 4;
robot.wheel_length = 3;
robot.wheel_width  = 2;
robot.position_x   = 0;
robot.position_y   = 0;
robot.theta        = 0;  % Units are radians
robot.r_wheelangle = 0;
robot.l_wheelangle = 0; 
robot.sensorx      = 4;  % Units are cm from midpoint between wheels
robot.sensory      = 0;  % Units are cm from midpoint between wheels
robot.sensor_angle = -15*pi/180;  % Units are radians
robot.sensor_range = 100;  % Sensor range: units are cm


figure(1);
clf;
fcn_drawRobot(robot)  % Draw the first time


% 
% % Test the right wheel
% for theta = 0.1:0.01:pi/2
%     robot.r_wheelangle = theta;
%     fcn_drawRobot(robot,1)  % Redraw the robot in Figure 1
%     pause(0.015);
% end
% 
% % Test the left wheel
% for theta = 0.1:0.01:pi/2
%     robot.l_wheelangle = theta;
%     fcn_drawRobot(robot,1)  % Redraw the robot in Figure 1
%     pause(0.015);
% end
% 
% 
% % Test y translation
% for y = 0.1:0.1:10
%     robot.position_y = y;
%     fcn_drawRobot(robot,1)  % Redraw the robot in Figure 1
%     pause(0.015);
% end
% 
% % Test theta translation
% for theta = 0.1:0.01:pi/2
%     robot.theta = theta;
%     fcn_drawRobot(robot,1)  % Redraw the robot in Figure 1
%     pause(0.015);
% end

%% Test basic kinematics
delta_t = 0.01; %100 Hz sampling rate
t = 0:delta_t:20;

omega_L = 0; % Initial value of robot's left motor speed
omega_R = 0; % Initial value of robot's right motor speed

for i=1:length(t)
    time = t(i); % This is the current time

    % Take torque for left and right motors, and run it through Newtonian
    % dynamics
    torque_L = 0.1*sin(time) + 0.9;
    torque_R = 0.1*sin(time) + 0.9;
    
    % Define sum of torques
    torque_coulomb_L = 0.01; % Assume 5 percent of torque is colomb
    torque_coulomb_R = 0.01; % Assume 5 percent of torque is colomb
    max_speed = 20; % Units are cm/second
    max_rotational_velocity = (max_speed / (robot.wheel_length/2)); % Max rotational speed of motor, in rad/sec

    % Left motor calculations from torque to angular acceleration
    %J_L = 0.2*0.015*0.015 ; % Units are kg*m^2 ... so assume 0.2 kg acting at 0.015m
    J_L = .02;
    damping_L = 1 / max_rotational_velocity; % Assume 100% of torque (1) is required to get to max velocity
    torque_friction_L = torque_coulomb_L * sign(omega_L)   + omega_L*damping_L;
    torque_net_L = torque_L - torque_friction_L;
    rot_accel_L = torque_net_L / J_L;

    % Left motor calculations from torque to angular acceleration
    %J_R = 0.2*0.015*0.015 ; % Units are kg*m^2 ... so assume 0.2 kg acting at 0.015m
    J_R = .02;
    damping_R = 1.05 / max_rotational_velocity; % Assume 100% of torque (1) is required to get to max velocity
    torque_friction_R = torque_coulomb_R * sign(omega_R)  + omega_R*damping_R;
    torque_net_R = torque_R - torque_friction_R;
    rot_accel_R = torque_net_R / J_R;

    % From sum(Torque) = rotational_inertia * angular_acceleration    
    omega_L = omega_L + rot_accel_L*delta_t;  % Units are cm/s
    omega_R = omega_R + rot_accel_R*delta_t;  % Units are cm/s
    
    %     %     % Left and right wheel velocities
    %     omega_L = sin(time) + 15 + exp(time/10);  % Units are cm/s
    %     omega_R = cos(time) + 15;  % Units are cm/s
    
    % Linear velocities
    v_L = omega_L*robot.wheel_length/2;
    v_R = omega_R*robot.wheel_length/2;
    
    % Total velocity
    V = (v_L + v_R)/2;
    omega_robot = (v_R - v_L)/robot.width;
    
    % Update the robot based on kinematics
    robot.l_wheelangle = robot.l_wheelangle + omega_L*delta_t; % Agular speed of left wheel (rad/sec)
    robot.r_wheelangle = robot.r_wheelangle + omega_R*delta_t;  % Angular speed of wheel (rad/sec)
    robot.theta = robot.theta + omega_robot*delta_t; % Angular speed of robot (rad/sec)
    robot.position_x = robot.position_x + V*cos(robot.theta)*delta_t;
    robot.position_y = robot.position_y + V*sin(robot.theta)*delta_t;

    % Plot result
    IR_distance = fcn_drawRobot(robot,1);  % Redraw the robot in Figure 1
    if isnan(IR_distance)
        return;
    end
    
    pause(0.015);

end




    