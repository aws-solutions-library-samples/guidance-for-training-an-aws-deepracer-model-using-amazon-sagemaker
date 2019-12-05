""" Instead of the bubble penalty, we consider both the  
distance between the learner car and the closest bot car,  
as well as if the learner car is within the wedge area 
apexed at the bot car. The assumption is within the wedge
is more possible to crash. """

def reward_function(params):
    
    reward = 1e-3

    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    bot_car_progress_delta = params['bot_car_progress']
    bot_car_lane_match = params['bot_car_lane_match']
    speed = params['speed']
    steering = abs(params['steering_angle'])
    flag_unsafe = params['flag_unsafe']
    dist_closest_bot_car = params['dist_closest_bot']
    is_bot_in_camera = params['is_bot_in_camera']
    
    reward = 1e-3  # likely crashed / close to off track        
    if distance_from_center <= (0.3 * track_width): 
        reward_lane = 1.0
        if distance_from_center <= (0.2 * track_width):
            # geting close to the center
            reward_lane *= 0.8
        elif distance_from_center <= (0.1 * track_width):
            # getting closer
            reward_lane *= 0.2
        elif distance_from_center <= (0.05 * track_width):
            # too close
            reward_lane = 1e-3       
    else:
        reward_lane = 1e-3
        
    # avoid closest bot car
    reward_avoid = 1.0
    # penalize if distance too close
    if 0.8 <= dist_closest_bot_car < 1.0:
        reward_avoid *= 0.8
    elif 0.5 <= dist_closest_bot_car < 0.8:
        reward_avoid *= 0.5
    elif 0.3 < dist_closest_bot_car < 0.5:
        reward_avoid *= 0.01
        
    
    # on the different lane of the closest ahead bot car
    if bot_car_lane_match and is_bot_in_camera:
        reward = 0.0
    else:
        reward = 2.0*reward_avoid + 2.0*reward_lane
        
                          
    # speed penalty
    if speed < 3.0:
        reward *= 0.5
        

    return float(reward)