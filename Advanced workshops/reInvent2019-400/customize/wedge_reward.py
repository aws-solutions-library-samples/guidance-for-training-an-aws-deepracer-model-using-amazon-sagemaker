""" Instead of the bubble penalty, we consider both the  
distance between the learner car and the closest bot car,  
as well as if the learner car is within the wedge area 
apexed at the bot car. The assumption is within the wedge
is more possible to crash. """

def reward_function(params):

    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    flag_unsafe = params['flag_unsafe']
    dist_closest_bot_car = params['dist_closest_bot']

    reward = 1e-3  # likely crashed / close to off track

#     # wide centerline
#     marker_1 = 0.4 * track_width
#     if distance_from_center <= marker_1:
#         reward = 1.0
        
    if distance_from_center <= (0.4 * track_width): 
        reward_lane = 1.0
        if distance_from_center <= (0.2 * track_width):
            # geting close to the center
            reward_lane *= 0.5
        elif distance_from_center <= (0.1 * track_width):
            # getting closer
            reward_lane *= 0.2
        elif distance_from_center <= (0.05 * track_width):
            # too close
            reward_lane = 1e-3       
    else:
        reward_lane = 1e-3
       
    reward += 1.0*reward_lane

                
    reward_avoid = 1.0
    # penalize if distance too close
    if 0.5 <= dist_closest_bot_car < 0.8 and flag_unsafe:
        reward_avoid *= 0.5
    elif 0.3 < dist_closest_bot_car < 0.5 and flag_unsafe:
        reward_avoid *= 0.01
        
    # yunzhe tried 3 and 4
    reward += 10.0*reward_avoid
    
    # speed penalty
    if params['speed'] < 3.0:
        reward *= 0.5

    return float(reward)