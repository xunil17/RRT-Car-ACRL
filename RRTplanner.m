%% -----------------------------------------------------------------------
close all;
clear all;
clc;

% load map_1.mat;
% load map_2.mat;
load map_3.mat;

load_sim_params;

% scale is used to blow up the environment for display purposes only. Set
% to whatever looks good on your screen
scale = 9;

% determines the size of the map and creates a meshgrid for display
% purposes
[N,M] = size(map_struct.seed_map);
[x,y] = meshgrid(1:N, 1:M);

DISPLAY_ON = 1; % 1 - turns display on, 0 - turns display off
DISPLAY_TYPE = 0; % 0 - displays map as dots, 1 - displays map as blocks

initialize_state;
num_nodes = 20;
possible_actions = [-1:0.2:1];
number_of_timesteps_RRT = 4;

real_map = map_struct.map_samples{1};
x_max = size(real_map,1)*10;
y_max = size(real_map,2)*10;

q_start.coord = map_struct.start;
q_start.state = state;
q_start.cost = 0;
q_start.parent = 0;

q_goal.coord = map_struct.goal;
q_goal.cost = 0;

nodes(1) = q_start;

%% -----------------------------------------------------------------------
for i = 1:num_nodes
    action = randsample(possible_actions,1);
    q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
    hold on;
    plot(q_rand(1), q_rand(2), 'xb');
    
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end

    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    
    for t = 1:number_of_timesteps_RRT
        % 
        [state, observed_map, flags] = motionModel(params, state, action, observed_map, real_map, goal);
        if flags == 2
            break;
        end
    end
    if flags == 2
        break;
    end
    if (DISPLAY_ON)
        display_environment;
    end

end