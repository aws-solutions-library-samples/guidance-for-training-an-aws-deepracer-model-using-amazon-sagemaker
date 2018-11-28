# Lab 2: MGM Speedway Model

#### Time breakdown

- Presenter provides overview of Lab 2                      :  2 minutes
- Individual create reward function and start training model: 20 minutes
- Total                                                     : 22 minutes


## Overview

In this lab you will get 20 minutes to create the model you will be racing with at the MGM Speedway. You can always train another model afterwards, but make good use of the time and resources to come up with a great model. From the [AWS DeepRacer](https://console.aws.amazon.com/deepracer/home?region=us-east-1) page choose **Create model**. Proceed to name your model, leave all hyperparameter settings as they are, and focus your time on the reward function. While you can train your model for 60 minutes we recommend 150 minutes. Thus before you start training make sure to set the stopping time to 150 minutes. After you have started training we will continue with the workshop going under the hood of AWS DeepRacer.

While you don't have to start training your model after 20 minutes, it is advisable to try get to a point where you can start it as training will take the better part of an hour and you want to ensure you get to the MGM Speedway with your model on your USB stick.

Once your model training is completed, create a folder called "models" on your USB stick, download the model from the AWS DeepRacer console, and copy it to the models folder. You are then ready to race at the MGM Speedway.

What you want to see during training is a TrainingRewardScore graph where the average monotonically increases. Once your car can complete one-and-a-half to two loops in the simulator your model should be good to take to the real car. The risk is that if you train too long you could just overfit and have a model that struggles to generalize in the real world. Using the provided reward functions and hyperparameters this should happen at around 2 to 2.5hours. The following image shows what a converging model TrainingRewardScore graph looks like. Note if you provide significant rewards,i.e. > 2 per step then your y-axis scale may be much larger.


![Converge_Training](img/good_sim_training2.png)

### Reward Function Tips
Tip 1: Start by looking at the advanced reward functions for inspiration. We provide a few examples below.
Tip 2: Think carefully through the driving behavior you want to incentivize and consider the trade-offs. For example, you can penalize your car for going slow, but if all your car can do is go fast, it may not be the best at taking turns.
Tip 3: Use Python3 syntax for your reward function.

Here are the variables you can use in your logic. 
Furthermore, the following three cells show examples of advanced reward functions.

![Reward Function Variables](img/reward_vars.png)


**Advanced Reward Function 1**


'''python

    def reward_function (on_track, x, y, distance_from_center, car_orientation, progress, steps, throttle, steering, track_width, waypoints, closest_waypoint):

        import math

        marker_1 = 0.1 * track_width
        marker_2 = 0.25 * track_width
        marker_3 = 0.5 * track_width

        reward = 1e-3
        if distance_from_center >= 0.0 and distance_from_center <= marker_1:
            reward = 1
        elif distance_from_center <= marker_2:
            reward = 0.5
        elif distance_from_center <= marker_3:
            reward = 0.1
        else:
            reward = 1e-3  # likely crashed/ close to off track

        # penalize reward if the car is steering way too much
        ABS_STEERING_THRESHOLD = 0.5
        if abs(steering) > ABS_STEERING_THRESHOLD:
            reward *= 0.8

        return float(reward)
'''


**Advanced Reward Function 2**


'''python

    def reward_function (on_track, x, y, distance_from_center, car_orientation, progress, steps, throttle, steering, track_width, waypoints, closest_waypoint):

        import math

        marker_1 = 0.1 * track_width
        marker_2 = 0.25 * track_width
        marker_3 = 0.5 * track_width

        reward = 1e-3
        if distance_from_center >= 0.0 and distance_from_center <= marker_1:
            reward = 1
        elif distance_from_center <= marker_2:
            reward = 0.5
        elif distance_from_center <= marker_3:
            reward = 0.1
        else:
            reward = 1e-3  # likely crashed/ close to off track

        # penalize reward for the car taking slow actions
        THROTTLE_THRESHOLD = 0.5
        if throttle < THROTTLE_THRESHOLD:
            reward *= 0.8

        return float(reward)
'''


**Advanced Reward Function 3**


'''python

    def reward_function (on_track, x, y, distance_from_center, car_orientation, progress, steps, throttle, steering, track_width, waypoints, closest_waypoint):

        reward = 1e-3
        if distance_from_center >= 0.0 and distance_from_center <= 0.03:
            reward = 1.0

        # add steering penalty
        if abs(steering) > 0.5:
            reward *= 0.80

        # add throttle penalty
        if throttle < 0.5:
            reward *= 0.80

        return reward
'''


[Go back to Workshop](https://github.com/aws-samples/aws-deepracer-workshops/blob/master/README.md)
