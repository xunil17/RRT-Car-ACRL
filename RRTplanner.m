%% -----------------------------------------------------------------------
close all;
clear all;
clc;

load map_1.mat;
% load map_2.mat;
% load map_3.mat;

map = map_struct.seed_map;
hold on;
for x = 1:size(map,1)
   for y = 1:size(map,2)
       if map(x,y) == 0
          obstacle_plot = plot(y, x, 'r*','MarkerSize',3); 
       end
   end
end

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
num_nodes = 5000;
possible_actions = [-1:0.2:1];
number_of_timesteps_RRT = 20;

real_map = map_struct.map_samples{1};
x_max = size(real_map,1);
y_max = size(real_map,2);

% q_start.coord = map_struct.start;
q_start.state = state;
q_start.action = 0;
% q_start.cost = 0;
q_start.parent = 0;

goal_state.x = map_struct.goal.x;
goal_state.y = map_struct.goal.y;
goal_state.theta = 0;
goal_state.H = state.H;
goal_state.border = state.H;
goal_state.moveCount = 0;

plot(goal_state.x, goal_state.y, 'g*','MarkerSize',4);

% q_goal.state = goal_state;
% q_goal.coord = map_struct.goal;
% q_goal.cost = 0;

nodes(1) = q_start;

%% -----------------------------------------------------------------------
for i = 1:num_nodes
%     action = randsample(possible_actions,1);
    q_rand_coord.x = floor(rand(1)*x_max); 
    q_rand_coord.y = floor(rand(1)*y_max);
    hold on;
    plot(q_rand_coord.x, q_rand_coord.y, 'xb');
    
    % Break if goal node is already reached
%     for j = 1:1:length(nodes)
%         if nodes(j).state.x == q_goal.state.x && nodes(j).state.y == q_goal.state.y
%             break
%         end
%     end

    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.state, q_rand_coord);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    
    
    q_near_node = nodes(idx);
    action = action_select(q_near_node.state, q_rand_coord);
    action = randsample([randsample(possible_actions,1), action], 1);
    disp(action);
%     disp(q_near_node.state)
    [q_new_node.state, flags] = steerRRT(q_near_node.state, action, number_of_timesteps_RRT, params, observed_map, observed_map, goal);
    if flags ~= 2
        q_new_node.state.H = state.H;
        q_new_node.state.border = state.border;
        q_new_node.state.moveCount = 0;
        q_new_node.parent = idx;
        q_new_node.action = action;
%         q_new_node.cost = 0;
        nodes = [nodes q_new_node];
    end
    
    if flags == 1
        q_goal = q_new_node;
        break;
    end
    
%     line([200,400] , [200,400], 'Color', 'k', 'LineWidth', 2);
%     drawnow
%     disp([q_near_node.state.x, q_near_node.state.y, q_new_node.state.x, q_new_node.state.y]);
    line([q_new_node.state.x, q_near_node.state.x], [q_new_node.state.y, q_near_node.state.y], 'LineWidth', 1);
    drawnow
    
%         hold on
%         q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
%         
%         % Within a radius of r, find all existing nodes
%         q_nearest = [];
%         r = 60;
%         neighbor_count = 1;
%         for j = 1:1:length(nodes)
%             if noCollision(nodes(j).coord, q_new.coord, obstacle) && dist(nodes(j).coord, q_new.coord) <= r
%                 q_nearest(neighbor_count).coord = nodes(j).coord;
%                 q_nearest(neighbor_count).cost = nodes(j).cost;
%                 neighbor_count = neighbor_count+1;
%             end
%         end
%         
%         % Initialize cost to currently known value
%         q_min = q_near;
%         C_min = q_new.cost;
%         
%         % Iterate through all nearest neighbors to find alternate lower
%         % cost paths
%         
%         for k = 1:1:length(q_nearest)
%             if noCollision(q_nearest(k).coord, q_new.coord, obstacle) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
%                 q_min = q_nearest(k);
%                 C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
%                 line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');                
%                 hold on
%             end
%         end
%         
%         % Update parent to least cost-from node
%         for j = 1:1:length(nodes)
%             if nodes(j).coord == q_min.coord
%                 q_new.parent = j;
%             end
%         end
%         
%         % Append to nodes
%         nodes = [nodes q_new];
%     end
%     if (DISPLAY_ON)
%         display_environment;
%     end
end

%% return path

current_node = q_goal;
save_commands = [];
while current_node.parent ~= 0
   save_commands = [ones(1,number_of_timesteps_RRT)*[current_node.action],save_commands]; 
   current_node = nodes(current_node.parent);
end

disp(save_commands);