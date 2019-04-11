function action = action_select(state,desired_target)
% Want to select action based on heading of state and location of desired
% target
denom = desired_target.x - state.x;
    if denom == 0
        denom = 0.01;
    end
    input_head = (desired_target.y - state.y) / denom;
    desired_heading = atan(input_head);


    if (desired_target.x < state.x)
        desired_heading = desired_heading - pi;
    end

    if (desired_heading >= 2 * pi)
        desired_heading = desired_heading - 2 * pi;
    end

    if (desired_heading < 0)
        desired_heading = desired_heading + 2 * pi;
    end

	diff = desired_heading - state.theta;
	absDiff = abs(diff);

    if (absDiff <= pi)
        if absDiff == pi
           retValue = pi; 
        else
            retValue = diff;
        end
    elseif (desired_heading > state.theta)
        retValue = ( absDiff - 2 * pi);
    else
        retValue = ( 2 * pi - absDiff);
    end

    if (abs(retValue) < 0.001)
        retValue = 0;
    end
    
    if retValue <= -pi/2
       action = -1;
    elseif retValue >= pi/2
        action = 1;
    else
        action = retValue/(pi/2);
    end
    
    action = action + (rand(1)*0.2) - 0.1;
    
    if action < -1
        action = -1;
    elseif action > 1
        action = 1;
    end

end