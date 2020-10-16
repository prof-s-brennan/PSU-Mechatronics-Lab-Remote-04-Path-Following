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
IR_distance = interp1(distance_array(:,1),distance_array(:,2),distance) + 3*randn(length(distance(:,1)),1);

