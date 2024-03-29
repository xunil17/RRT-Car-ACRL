function [state, flags] = steerRRT(q_near_state, action, number_of_timesteps_RRT, params, observed_map, real_map, goal)    
    state = q_near_state;
    for t = 1:number_of_timesteps_RRT
        % 
        [state, observed_map, flags] = motionModel(params, state, action, observed_map, real_map, goal);
        if flags == 2 || flags == 1
            break;
        end
    end 
    
end