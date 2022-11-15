---
title: "Create a reward function"
chapter: true
weight: 25
description: "The reward function describes immediate feedback (as a score for reward or penalty) when the vehicle takes an action to move from a given position on the track to a new position. Its purpose is to encourage the vehicle to make moves along the track to reach its destination quickly. The model training process will attempt to find a policy which maximizes the average total reward the vehicle experiences."
---

# Create a reward function

In general, you design your reward function to act like an incentive plan. You can customize your reward function with relevant input parameters passed into the reward function. 

## Reward function files

`DeepRacer_400_Workshop/src/artifacts/rewards/`

![Image](/images/400workshop/rewardfunctionfiles.png)

There are several example reward functions for you to get started from.


#### Follow the Center Line in Time Trials ####
`follow_center_line.py`

This example determines how far away the agent is from the center line, and gives higher reward if it is closer to the center of the track, encouraging the agent to closely follow the center line.

#### Stay Inside the Two Borders in Time Trials ####
`stay_inside_two_border.py`

This example simply gives high rewards if the agent stays inside the borders, and let the agent figure out what is the best path to finish a lap. It is easy to program and understand, but likely takes longer to converge.

#### Prevent Zig-Zag in Time Trials ####
`prevent_zig_zag.py`

This example incentivizes the agent to follow the center line but penalizes with lower reward if it steers too much, which helps prevent zig-zag behavior. The agent learns to drive smoothly in the simulator and likely keeps the same behavior when deployed in the physical vehicle.

#### Stay On One Lane without Crashing into Stationary Obstacles or Moving Vehicles ####
`object_avoidance_head_to_head.py`

This reward function rewards the agent to stay between the track borders and penalizes the agent for getting too close to the next object in the front. The agent can move from lane to lane to avoid crashes. The total reward is a weighted sum of the reward and penalty. The example gives more weight to the penalty term to focus more on safety by avoiding crashes. You can play with different averaging weights to train the agent with different driving behaviors and to achieve different driving performances.



## Create your own reward function (optional)


If you wish to create your own reward function there is a pattern to the function that you must use.

```python
def reward_function(params) :
    
    reward = ...

    return float(reward)
```

#### A list of parameters and their definition is located [here](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-reward-function-input.html?icmpid=docs_deepracer_console)

---

**NOTE**

Since you are using a Jupyter Notebook in an Amazon SageMaker Notebook Instance you can import modules to create robust reward functions or add functionality to the examples.

---

| ![Open SageMaker Notebook](/images/400workshop/aws-sagemaker-notebooks.png) | **Section: Upload the actions and rewards artifact files** |
|---|---|

1. Return to the Jupyter Notebook ``400_deepracer_rl.ipynb``

2. Complete the **Section: Upload the actions and rewards artifact files** in the notebook.


**[Proceed to the next activity](../starttraining/)**
