function [distance,location] = fcn_findSensorHit(wall_start,wall_end,sensor_vector,varargin)   
% fcn_findSensorHit calculates hits between sensor vector and walls
% Syntax:
% fcn_findSensorHit(wall_start,wall_end,sensor_vector,varargin) 
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
    fig_num = 23333; 
    figure(fig_num);
    clf;
    hold on;
    axis equal;
    grid on; grid minor;
    
    N_walls = length(wall_start(:,1));
    walls_x = [wall_start(:,1) wall_end(:,1) NaN*wall_start(:,1)];
    walls_y = [wall_start(:,2) wall_end(:,2) NaN*wall_start(:,2)];
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
% Define r and s vectors
p = wall_start;
q = sensor_vector(1,:);
r = wall_end - wall_start;
s = sensor_vector(2,:)-sensor_vector(1,:);
r_cross_s = crossProduct(r,s);
q_minus_p =  q - p;

q_minus_p_cross_s = crossProduct(q_minus_p,s);
q_minus_p_cross_r = crossProduct(q_minus_p,r);

parallel_indices = find(0==r_cross_s);
if any(parallel_indices)
    r_cross_s(indices) = 1; % They are colinear or parallel, so make dummy length
end
   
t = q_minus_p_cross_s./r_cross_s;
u = q_minus_p_cross_r./r_cross_s;

t(parallel_indices) = inf;
u(parallel_indices) = inf;

intersection = NaN*ones(length(p(:,1)),2);

good_vector = ((0<t).*(1>t).*(0<u).*(1>u));
good_indices = find(good_vector>0);
if ~isempty(good_indices)
    result = p + t.*r; 
    intersection(good_indices,:) = result(good_indices,:);
    %plot(intersection(:,1),intersection(:,2),'rx');
end

distances_squared = sum((intersection - sensor_vector(1,:)).^2,2);
[best,best_index] = min(distances_squared);

distance = best^0.5;
location = intersection(best_index,:);





%% Calculate cross products
function result = crossProduct(v,w)
result = v(:,1).*w(:,2)-v(:,2).*w(:,1);




