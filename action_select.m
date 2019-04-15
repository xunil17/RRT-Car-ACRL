function [action_choose, goal_reached] = action_select(og_state, goal, random_point, params, observed_map, real_map, number_of_timesteps_RRT)
% Want to select action based on heading of state and location of desired
% target
% denom = desired_target.x - state.x;
%     if denom == 0
%         denom = 0.01;
%     end
%     input_head = (desired_target.y - state.y) / denom;
%     desired_heading = atan(input_head);
% 
% 
%     if (desired_target.x < state.x)
%         desired_heading = desired_heading - pi;
%     end
% 
%     if (desired_heading >= 2 * pi)
%         desired_heading = desired_heading - 2 * pi;
%     end
% 
%     if (desired_heading < 0)
%         desired_heading = desired_heading + 2 * pi;
%     end
% 
% 	diff = desired_heading - state.theta;
% 	absDiff = abs(diff);
% 
%     if (absDiff <= pi)
%         if absDiff == pi
%            retValue = pi; 
%         else
%             retValue = diff;
%         end
%     elseif (desired_heading > state.theta)
%         retValue = ( absDiff - 2 * pi);
%     else
%         retValue = ( 2 * pi - absDiff);
%     end
% 
%     if (abs(retValue) < 0.001)
%         retValue = 0;
%     end
%     
%     if retValue <= -pi/2
%        action = -1;
%     elseif retValue >= pi/2
%         action = 1;
%     else
%         action = retValue/(pi/2);
%     end
    possible_actions = -1:0.1:1;
    dist_save_goal = intmax;
    dist_random_point = intmax;
    action_best = 0;
    action_random_goal = 0;
    goal_reached = 0;
    for a_index = 1:1:length(possible_actions)
        action = possible_actions(a_index);
        state = og_state;
        
        % simulate next state
        for t = 1:number_of_timesteps_RRT
            % 
            [state, observed_map, flags] = motionModel(params, state, action, observed_map, real_map, goal);
            if flags == 2 || flags == 1
                break;
            end
        end
        
        if flags == 1 %goal reached
            goal_reached = 1;
            action_best = action;
            action_choose = action;
            return
        elseif flags == 0 %didn't hit obstacle
           if dist_manhattan(state, goal) < dist_save_goal
                dist_save_goal = dist_manhattan(state,goal);
                action_best = action;
           end
           if dist_manhattan(state, random_point) < dist_random_point
              dist_random_point = dist_manhattan(state, random_point);
              action_random_goal = action; 
           end
        end
    end
    action_choose = action_best;

    action_choose = action_random_goal;
    action_choose = action_choose + (rand(1)*0.2) - 0.1;
    if action_choose < -1
        action_choose = -1;
    elseif action_choose > 1
        action_choose = 1;
    end
%     rand_num = rand;
%     if rand_num > 0.5
%        action_choose = action_best;
%     else
%         action_choose=action_best;
% %         action_choose = action_random_goal;
%     end
%     
%     
    


end