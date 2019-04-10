function [state, flags] = steerRRT(q_near_state, action, number_of_timesteps_RRT, params, observed_map, real_map, goal)    
    for t = 1:number_of_timesteps_RRT
        % 
        [state, observed_map, flags] = motionModel(params, q_near_state, action, observed_map, real_map, goal);
        if flags == 2
            break;
        end
    end 
    
end