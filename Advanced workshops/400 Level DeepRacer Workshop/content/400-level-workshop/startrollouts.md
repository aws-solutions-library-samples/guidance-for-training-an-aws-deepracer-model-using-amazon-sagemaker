---
title: "Create multiple Rollouts"
chapter: true
weight: 35
description: "We create AWS RoboMaker simulation jobs that simulates the environment and shares this data with SageMaker for training. Each roll-out uses a central model to independently collect experience in the form of episodes, where each episode consist of (state, action, next state, reward) tuples"
---

# Create multiple Rollouts 

We create AWS RoboMaker simulation jobs that simulates the environment and shares this data with SageMaker for training. Each roll-out uses a central model to independently collect experience in the form of episodes, where each episode consist of (state, action, next state, reward) tuples

![Image](/images/400workshop/fourrollouts.png)

We use horizontal scaling where the neural network model files are synchronized between the Amazon Sagemaker training job and AWS RoboMaker simulation workers.

### Create the Kinesis video stream (optional)

Like the AWS DeepRacer Console Kinesis video streams is used to display the video feed and additional overlay information. If you wish to view the video feed then you can access the media playback in AWS Console.
Performing this step is optional, but it is a great way to view all of your rollouts and their progress.
If you set multiple rollouts with a value over 1 for the **num_simulation_workers** you will see Kinesis Video Streams cycle through each rollout in the media playback.

[Kinesis Video Streams](https://console.aws.amazon.com/kinesisvideo/home?region=us-east-1#/streams)


| ![Rollout 1](/images/400workshop/rollout1.png) | ![Rollout 2](/images/400workshop/rollout2.png) |
|---|---|



|![Rollout 3](/images/400workshop/rollout3.png)|![Rollout 4](/images/400workshop/rollout4.png)|
|---|---|

You should see a near real-time media playback of your agents traversing the track in training and evaluating phases.

### Create the AWS RoboMaker Simulation Application

AWS DeepRacer uses AWS RoboMaker as the simulation engine to train your model using a world created in Gazebo.
To create this simulation we need to configure AWS RoboMaker by first creating an AWS RoboMaker Simulation Application.
More information on AWS RoboMaker and [Managing Simulation Applications](https://docs.aws.amazon.com/robomaker/latest/dg/managing-simulation-applications.html)

```
robomaker_s3_key = 'robomaker/simulation_ws.tar.gz'
robomaker_source = {'s3Bucket': s3_bucket,
                    's3Key': robomaker_s3_key,
                    'architecture': "X86_64"}
simulation_software_suite={'name': 'Gazebo',
                           'version': '7'}
robot_software_suite={'name': 'ROS',
                      'version': 'Kinetic'}
rendering_engine={'name': 'OGRE',
                  'version': '1.x'}

```
**robomaker_s3_key** sets the location that the AWS RoboMaker Simulation Application will look for the bundled SimApp. `robomaker/simulation_ws.tar.gz`

**robomaker_source** sets the S3 bucket location to AWS RoboMaker to look for the SimApp bundle.
Additionally the architecture is set for "X86_64" which should not be changed for this workshop.

**simulation_software_suite** sets Gazebo as the simulation engine, and version 7 of Gazebo.

**robot_software_suite** sets ROS and ROS version Kinetic which is the first version of ROS.

**rendering_engine** sets the graphical rendering engine, OGRE stands for Object-Oriented Graphics Rendering Engine

[Additional AWS RoboMaker Simulation FAQs](https://aws.amazon.com/robomaker/faqs/#Simulation)


### Bundle the AWS RoboMaker Simulation Application

The Simulation Application was downloaded and you expanded the SimApp into the `DeepRacer_400_Workshop/build` folder.
If you make changes to any files in the `DeepRacer_400_Workshop/src/markov` folder then you will need to copy those changes to the SimApp so the simulation works with your changes then bundle the SimApp.

If you did not make any changes to files in the `DeepRacer_400_Workshop/src/markov` folder then only bundle the SimApp, copying changes is not required.

### Upload the AWS RoboMaker Simulation Application to an Amazon S3 bucket

After bundling the SimApp locally in your notebook filesystem you need to upload the SimApp to S3.
The location for upload is set by the **robomaker_s3_key** parameter.


### Create ARN for the AWS RoboMaker Simulation Application

Using boto3.client("robomaker"), create the Amazon Resource Name which will return an ARN for use in creating AWS RoboMaker Simulation Jobs.
This ARN is used as a reference to which Simulation Application the rollouts will use. 

### Set the AWS RoboMaker Simulation Job parameters and start the training Simulation Jobs

This task contains multiple steps to set Simulation parameters and start the training Simulation jobs.

#### Set SimApp Parameters ####

Since the Simulation Application contains multiple tracks and multiple race types there is a need to tell the SimApp which to run in the Simulation Job. 

Like the Action Space and Reward Function artifact file templates there are example SimApp configuration files available for you to review.

The files are located in the `DeepRacer_400_Workshop/src/artifacts/yaml` folder.
There are four files:

**Training**
`DeepRacer_400_Workshop/src/artifacts/yaml/training_yaml_template.yaml`

**Head to Head**
`DeepRacer_400_Workshop/src/artifacts/yaml/head2head_yaml_template.yaml`

**Tournament**
`DeepRacer_400_Workshop/src/artifacts/yaml/tournament_yaml_template.yaml`

**Evaluation**
`DeepRacer_400_Workshop/src/artifacts/yaml/evaluation_yaml_template.yaml`


For this section we will focus on **Training**

In the `training_yaml_template.yaml` there are parameters that will set the track and the type of race.

For this workshop try TIME_TRIAL, OBJECT_AVOIDANCE, HEAD_TO_BOT as the **race_type**.

Also select the track you wish to use by setting **world_name**. 

![Rollout params](/images/400workshop/rolloutnotebook.png)



***Set the world_name using one of the below values*** 

| Track | world_name |
|---|---|
|![Bowtie](/images/400workshop/Bowtie_track.png)|world_name = "Bowtie_track"|
|![Canada](/images/400workshop/Canada_Training.png)|world_name = "Canada_Training"|
|![China](/images/400workshop/China_track.png)| world_name = "China_track"|
|![LGSWide](/images/400workshop/LGSWide.png)| world_name = "LGSWide"|
|![Mexico](/images/400workshop/Mexico_track.png)| world_name = "Mexico_track"|
|![New York](/images/400workshop/New_York_Track.png)| world_name = "New_York_Track"|
|![Oval](/images/400workshop/Oval_track.png)| world_name = "Oval_track"|
|![ReInvent base](/images/400workshop/reinvent_base.png)| world_name = "reinvent_base"|
|![ReInvent 2019](/images/400workshop/reInvent2019_track.png)| world_name = "reInvent2019_track"|
|![ReInvent 2019 Wide](/images/400workshop/reInvent2019_wide.png)| world_name = "reInvent2019_wide"|
|![Spain](/images/400workshop/Spain_track.png)| world_name = "Spain_track"|
|![Tokyo](/images/400workshop/Tokyo_Training_track.png)| world_name = "Tokyo_Training_track"|

After setting the values in the notebook a new file will be created called `training_params.yaml`
This file will be uploaded to Amazon S3 for the SimApp to use when creating the simulation.


#### Set Simulation Job environment variables ####

Additional variables need to be set for the Simulation Job.

**vpcConfig** is needed so AWS RoboMaker can communicate with Amazon SageMaker.
This variable is set earlier in the notebook and the VPC is created in the setup portion of this workshop.

**envriron_vars** is a series of variables that are created for the Simulation Job to use at run-time.
The AWS DeepRacer SimApp uses these variables for internal functions.

**simulation_application** this sets the framework for the Simulation Job, it sets the Simulation Application that the Simulation Job is based on and the launch configuration. 

---

**Note**

Defaults are ok as all variables are set from previous cells in the notebook.

---


#### Create the Simulation Jobs ####

Earlier in the notebook you set the variable **num_simulation_workers**
Using this variable a loop is created that traverses a for in statement.
If you set **num_simulation_workers** > 1 then the loop will create the set number of Simulation Jobs.

```python
responses = []
for job_no in range(num_simulation_workers):
    client_request_token = strftime("%Y-%m-%d-%H-%M-%S", gmtime())
    envriron_vars = {
        "S3_YAML_NAME": s3_yaml_name,
        "SAGEMAKER_SHARED_S3_PREFIX": s3_prefix,
        "SAGEMAKER_SHARED_S3_BUCKET": s3_bucket,
        "WORLD_NAME": world_name,
        "KINESIS_VIDEO_STREAM_NAME": kvs_stream_name,
        "APP_REGION": aws_region,
        "MODEL_METADATA_FILE_S3_KEY": "%s/model/model_metadata.json" % s3_prefix,
        "ROLLOUT_IDX": str(job_no)
    }

    simulation_application = {"application":simulation_app_arn,
                              "launchConfig": {"packageName": "deepracer_simulation_environment",
                                               "launchFile": "distributed_training.launch",
                                               "environmentVariables": envriron_vars}
                             }
    response =  robomaker.create_simulation_job(iamRole=sagemaker_role,
                                            clientRequestToken=client_request_token,
                                            maxJobDurationInSeconds=job_duration_in_seconds,
                                            failureBehavior="Fail",
                                            simulationApplications=[simulation_application],
                                            vpcConfig=vpcConfig
                                            )
    responses.append(response)
    time.sleep(5)

```


### Plot metrics for training job ###

Once the AWS RoboMaker Simulation Jobs are running and Amazon SageMaker training jobs are running you will be able to plot the metrics for training rewards.

The logs that the plotting function use the file training_metrics.json that is uploaded to Amazon S3.

![Plot metrics](/images/400workshop/rewardgraph.png)

As time goes on you can re-run the notebook cell to update the plot with the new training rewards.

![Plot metrics](/images/400workshop/rewardgraphupdated.png)



| ![Open SageMaker Notebook](/images/400workshop/aws-sagemaker-notebooks.png) | **Section: Start the AWS Robomaker Simulation for Model Training** |
|---|---|

1. Return to the Jupyter Notebook ``400_deepracer_rl.ipynb``

2. Complete the **Section: Start the AWS Robomaker Simulation for Model Training** in the notebook.



**[Proceed to the next activity](../evaluation/)**
