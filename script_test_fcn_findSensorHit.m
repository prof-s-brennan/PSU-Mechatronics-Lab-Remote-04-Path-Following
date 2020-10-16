% Script_test_fcn_findSensorHit

clc

%% Simple test 1 - a simple intersection
fprintf(1,'Simple intersection result: \n');
wall_start = [0 10];
wall_end   = [10 10];
sensor_vector = [0 0; 5 12];
[distance,location] = fcn_findSensorHit(wall_start,wall_end,sensor_vector) 

%% Simple test 2 - no intersections - 
fprintf(1,'No intersection result: \n');
wall_start = [0 10];
wall_end   = [2 10];
sensor_vector = [0 0; 5 12];
[distance,location] = fcn_findSensorHit(wall_start,wall_end,sensor_vector) 

%% Simple test 3 - multiple intersections
fprintf(1,'Multiple intersections result: \n');
wall_start = [0 10; 0 5; 0 2];
wall_end   = [10 10; 10 5; 10 2];
sensor_vector = [0 0; 5 12];
[distance,location] = fcn_findSensorHit(wall_start,wall_end,sensor_vector) 

%% Simple test 4 - some hits and some misses
fprintf(1,'Some hits, some misses result: \n');
wall_start = [0 10; 6 8; 0 5; 0 2; 0 8; 6 4];
wall_end   = [10 10; 10 8; 10 5; 10 2; 2 8; 10 4];
sensor_vector = [0 0; 5 12];
[distance,location] = fcn_findSensorHit(wall_start,wall_end,sensor_vector) 



