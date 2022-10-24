---
title: "Train the RL Model"
chapter: true
weight: 30
description: "Train it!"
---



# Train the RL model

The training process involves using AWS RoboMaker to emulate driving experiences in the environment, relaying the experiences at fixed intervals to Amazon SageMaker as input to train the deep neural network, and updating the network weights to an S3 location.

## Build and push Docker image

Part of this training process uses a customized Docker image.
The Dockerfile is located in the Amazon SageMaker Notebook Instance filesystem here:

```DeepRacer_400_Workshop/Dockerfile```

[More information on how Amazon SageMaker creates containers for custom training](https://docs.aws.amazon.com/sagemaker/latest/dg/your-algorithms.html)

The type of container whether the training is done via CPU or if the training takes advantage of a GPU is set via the **instance_type** parameter.
If you selected a GPU instance type then be aware that training may go faster but the cost for a GPU container is higher than CPU.

Another action is code is copied from the ```DeepRacer_400_Workshop/src``` folder in the Amazon SageMaker Notebook Instance to the Docker container.

---

***NOTE*** 

If you modify files after the Build and Push Docker image step that you want to include in the Amazon SageMaker container you must re-run the step.


---

## Set algorithm metrics for CloudWatch

Next, we define the following algorithm metrics that we want to capture from CloudWatch logs to monitor the training progress. These are algorithm specific parameters and might change for different algorithm. We use Clipped PPO for this example.

## Train the RL model using the Python SDK Script mode

We use Clipped PPO (as provided by Coach) as our reinformcent learning algorithm to train our network. 
To edit the hyperparameters of the Clipped PPO RL agent edit the cell in the Jupyter Notebook.

The areas to edit are highlighted below.

![Image](/images/400workshop/hyperparams.png)

---

### Hyperparameter definitions

**"batch_size": "64"**

This item is short for Gradient descent batch size which is defined as the number of recent vehicle experiences sampled at random from an experience buffer and used for updating the underlying deep-learning neural network weights. Random sampling helps reduce correlations inherent in the input data. Use a larger batch size to promote more stable and smooth updates to the neural network weights, but be aware of the possibility that the training may be longer or slower.

The batch is a subset of an experience buffer that is composed of images captured by the camera mounted on the AWS DeepRacer vehicle and actions taken by the vehicle.

**"num_epochs": "10"**

The number of passes through the training data to update the neural network weights during gradient descent. The training data corresponds to random samples from the experience buffer. Use a larger number of epochs to promote more stable updates, but expect a slower training. When the batch size is small, you can use a smaller number of epochs.

**"stack_size": "1"**

Used for the observation space, default value is 1.

**"lr": "0.0003"**

During each update, a portion of the new weight can be from the gradient-descent (or ascent) contribution and the rest from the existing weight value. The learning rate controls how much a gradient-descent (or ascent) update contributes to the network weights. Use a higher learning rate to include more gradient-descent contributions for faster training, but be aware of the possibility that the expected reward may not converge if the learning rate is too large.

**"exploration_type": "Categorical"**

This is used to determine the desired trade-off between exploration and exploitation. Since AWS DeepRacer uses a discrete action space where you set the speed and steering angles the value should be set to "Categorical".

**"e_greedy_value": "1"**

e-greedy is Epsilon Greedy which is an exploration policy that is intended for both discrete and continuous action spaces.
For discrete action spaces, it assumes that each action is assigned a value, and it selects the action with the
highest value with probability 1 - epsilon. Otherwise, it selects a action sampled uniformly out of all the
possible actions. The epsilon value is given by the user and can be given as a schedule.

Unless you wish to explore advanced scenarios the default value of "1" is appropriate.
Additionally since the value for exploration_type is set to "Categorical" e_greedy_value is not used.

**"epsilon_steps": "10000"**

The number of experience gathering steps used in a training run.

**"beta_entropy": "0.01"**

The degree of uncertainty used to determine when to add randomness to the policy distribution. The added uncertainty helps the AWS DeepRacer vehicle explore the action space more broadly. A larger entropy value encourages the vehicle to explore the action space more thoroughly.

**"discount_factor": "0.999"**

The discount factor determines how much of future rewards are discounted in calculating the reward at a given state as the averaged reward over all the future states. The discount factor of 0 means the current state is independent of future steps, whereas the discount factor 1 means that contributions from all of the future steps are included. With the discount factor of 0.9, the expected reward at a given step includes rewards from an order of 10 future steps. With the discount factor of 0.999, the expected reward includes rewards from an order of 1000 future steps.

**"loss_type": "Huber"**

The type of the objective function to update the network weights. A good training algorithm should make incremental changes to the vehicle’s strategy so that it gradually transitions from taking random actions to taking strategic actions to increase reward. But if it makes too big a change then the training becomes unstable and the agent ends up not learning. The Huber and Mean squared error loss types behave similarly for small updates. But as the updates become larger, the Huber loss takes smaller increments compared to the Mean squared error loss. When you have convergence problems, use the Huber loss type. When convergence is good and you want to train faster, use the Mean squared error loss type.

**"num_episodes_between_training": "20"**

The size of the experience buffer used to draw training data from for learning policy network weights. An episode is a period in which the vehicle starts from a given starting point and ends up completing the track or going off the track. Different episodes can have different lengths. For simple reinforcement-learning problems, a small experience buffer may be sufficient and learning will be fast. For more complex problems which have more local maxima, a larger experience buffer is necessary to provide more uncorrelated data points. In this case, training will be slower but more stable. The recommended values are 10, 20 and 40.

**"max_sample_count": "0"**

default do not change

**"sampling_frequency": "1"**

default do not change


| ![Open SageMaker Notebook](/images/400workshop/aws-sagemaker-notebooks.png) | **Section: Train the RL Model** |
|---|---|

1. Return to the Jupyter Notebook ``400_deepracer_rl.ipynb``

2. Complete the **Section: Train the RL Model** in the notebook.


**[Proceed to the next activity](../startrollouts)**

