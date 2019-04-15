%% -----------------------------------------------------------------------
% close all;
% clear all;
% clc;

% load map_1.mat;
% load map_2.mat;
% load map_3.mat;
% 
% load_sim_params;
% 
% % scale is used to blow up the environment for display purposes only. Set
% % to whatever looks good on your screen
% scale = 9;
% 
% % determines the size of the map and creates a meshgrid for display
% % purposes
% [N,M] = size(map_struct.seed_map);
% [x,y] = meshgrid(1:N, 1:M);
% 
% DISPLAY_ON = 1; % 1 - turns display on, 0 - turns display off
% DISPLAY_TYPE = 0; % 0 - displays map as dots, 1 - displays map as blocks
M_CVF = 5;
initialize_state;
num_nodes = 5000;
possible_actions = [-1:0.2:1];
number_of_timesteps_RRT = 20;

% RRT_map = observed_map;
% RRT_map = map_struct.map_samples{1};

%Display map
hold on;
for x = 1:size(RRT_map,1)
   for y = 1:size(RRT_map,2)
       if RRT_map(x,y) == 0
          obstacle_plot = plot(y, x, 'r*','MarkerSize',3); 
       end
   end
end

x_max = size(RRT_map,1);
y_max = size(RRT_map,2);

% q_start.coord = map_struct.start;
q_start.state = state;
q_start.action = 0;
q_start.CVF = 0;
q_start.parent = 0;

goal_state.x = map_struct.goal.x;
goal_state.y = map_struct.goal.y;
goal_state.theta = 0;
goal_state.H = state.H;
goal_state.border = state.H;
goal_state.moveCount = 0;

plot(goal_state.x, goal_state.y, 'g*','MarkerSize',4);

nodes(1) = q_start;

%% -----------------------------------------------------------------------
for i = 1:num_nodes
%     action = randsample(possible_actions,1);
    %choose goal target 20 percent of the time
    if rand(1) >= 0.80
        q_rand_coord.x = goal_state.x;
        q_rand_coord.y = goal_state.y;
    else
        q_rand_coord.x = floor(rand(1)*x_max); 
        q_rand_coord.y = floor(rand(1)*y_max);
    end

    hold on;

    % Pick the closest node from existing list to branch out from
    ndist = [];
    flags = 0;
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.state, q_rand_coord);
        ndist = [ndist tmp];
    end
    sorted_dists = sort(ndist);
    
    %For every node in tree, try extending to closest one
    for index = 1:1:length(sorted_dists)
        idx = find(ndist==sorted_dists(index));

        q_near_node_id = idx(1);
        q_near_node = nodes(q_near_node_id);
        rand_skip = rand;
%         disp(q_near_node.CVF);
        %skip expanding nodes if they have too many children nodes
        if rand_skip > q_near_node.CVF/M_CVF
%             if dist(q_near_node.state,goal) < 10
%                 number_of_timesteps_RRT = 6;
%             else
%                 number_of_timesteps_RRT = 12;
% %                 
%             end
%         
%             action = action_select(q_near_node.state, q_rand_coord, goal, number_of_timesteps_RRT);
            
            %select action that brings us closest to manhattan distance of
            %goal
            [action, goal_reached] = action_select(q_near_node.state, goal, q_rand_coord, params, observed_map, RRT_map, number_of_timesteps_RRT);
            %         action = randsample(possible_actions,1);
            if ~goal_reached
            	action = randsample([(rand(1)*2) - 1, action], 1);
%                 action = randsample([randsample(possible_actions,1), action], 1);
            end

            [q_new_node.state, flags] = steerRRT(q_near_node.state, action, number_of_timesteps_RRT, params, RRT_map, RRT_map, goal);
            if flags ~= 2
                q_new_node.state.H = state.H;
                q_new_node.state.border = state.border;
                q_new_node.state.moveCount = 0;
                q_new_node.parent = q_near_node_id;
                q_new_node.action = [action,number_of_timesteps_RRT];
                q_new_node.CVF = 0;

                nodes = [nodes q_new_node];
                line([q_new_node.state.x, q_near_node.state.x], [q_new_node.state.y, q_near_node.state.y], 'LineWidth', 1);
                drawnow
                if flags == 1
                    disp("REACHED GOAL INNER");
                    q_goal = q_new_node;
                    break;
                end
                break;
            else %%hit into a wall - update CVF
                current_node_id = q_near_node_id;
                current_node = q_near_node;
                count = 0;
                while current_node.parent ~= 0
                    current_node.CVF = current_node.CVF + 1/(M_CVF^count);
                    nodes(current_node_id) = current_node;
                    count = count + 1;
                    current_node_id = current_node.parent;
                    current_node = nodes(current_node_id);
                end
            end
        end
    end
    if flags == 1
        disp("REACHED GOAL")
        q_goal = q_new_node;
        break;
    end
end

%% return path

current_node = q_goal;
save_commands = [];
while current_node.parent ~= 0
    act = current_node.action(1);
    timestep = current_node.action(2);
    save_commands = [ones(1,timestep)*act,save_commands];
    parent_node = nodes(current_node.parent);
    line([current_node.state.x, current_node.state.x], [parent_node.state.y, parent_node.state.y], 'LineWidth', 2, 'Color', 'green');
    drawnow
    current_node = parent_node;
    
end
saveas(gcf,'RRT_planner.png')
disp("Number Of Nodes: ");
disp(i);
% disp(save_commands);