---
title: "Evaluate your models"
chapter: true
weight: 40
description: ""
---

# Evaluate your models

After training your model using multiple rollouts you can evaluate the current state of the training by using an evaluation simulation. 
When evaluating your model only a single AWS RoboMaker Simulation Job will be created as opposed to multiple Simulation Jobs if you created multiple rollouts.
Additionally, you have the opportunity to add custom labels to the media overlay for the simulation.

### Set the AWS RoboMaker Simulation Job parameters and start the evaluation Simulation Job

There are several similarities between the training Simulation Job parameters, including the **world_name** and **race_type**.
Since you are evaluating a trained model you can run an evaluation against the same world name and race type.
You can also run an evaluation using a different world name and race type.

![Evaluation World and Race Type](/images/400workshop/evalworldandrace.png)

There are additional parameters that you can change to customize the Kinetic Video Streams output.

[Kinesis Video Streams](https://console.aws.amazon.com/kinesisvideo/home?region=us-east-1#/streams)

![Evaluation YAML](/images/400workshop/evalyaml.png)

| key | value |
|---|---|
| yaml_config['NUMBER_OF_TRIALS'] | Set the number of laps for evaluation |
| yaml_config['DISPLAY_NAME'] | Displayed in the upper left corner to identify the current racer|
| yaml_config['LEADERBOARD_TYPE'] | **Leave as "LEAGUE"**|
|yaml_config['LEADERBOARD_NAME'] | Displayed on the bottom area of the media output|
|yaml_config['CAR_COLOR']| Controls the color of the racecar|
|yaml_config['NUMBER_OF_RESETS']| The number of resets allowed per lap|
|yaml_config['PENALTY_SECONDS']| **Leave as "5"** |
|yaml_config['OFF_TRACK_PENALTY']| Number of seconds to add to the race time when the race car leaves the track|
|yaml_config['COLLISION_PENALTY']| Number of seconds to add to the race time when the race car collides with an obstacle like a box in the OBJECT_AVOIDANCE race type|



Below is an example of the evaluation job media output using the parameters set in the notebook

![Evaluation Video](/images/400workshop/evalvideo.png)

### Plot metrics for evaluation job

For evaluation jobs the metrics you plot are based on the time your race car takes to go around the track including the penalties.
This is different than the training job because the evaluation jobs are evaluating the model training not the rewards returned during training.

![Evaluation YAML](/images/400workshop/evalplot.png)

| ![Open SageMaker Notebook](/images/400workshop/aws-sagemaker-notebooks.png) | **Section: Start the AWS Robomaker Simulation for Model Evaluation** |
|---|---|

1. Return to the Jupyter Notebook ``400_deepracer_rl.ipynb``

2. Complete the **Section: Start the AWS Robomaker Simulation for Model Evaluation** in the notebook.


**[Proceed to the next activity](../cleanup/)**
