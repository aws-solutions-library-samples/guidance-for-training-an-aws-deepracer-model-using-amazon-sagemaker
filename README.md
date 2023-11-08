## ReadMe - Guidance for training an AWS DeepRacer model using Amazon SageMaker  

Table of contents

* Overview
  * Cost
* Prerequisites
  * Operating system
* Deployment steps
* Deployment validation
* Running the guidance
* Next steps
* Cleanup

### Overview ###

The AWS DeepRacer console is optimized to provide a user-friendly introduction to reinforcement learning to developers new to machine learning. As developers go deeper in their machine learning journey, they need more control and more options for further tuning and refining their reinforcement learning models for racing with AWS DeepRacer. This guidance is intended to provide developers with a deep dive on how they can use an Amazon SageMaker Notebook instance to directly train and evaluate DeepRacer models with full control, including: augmenting the simulation environment, manipulating inputs to the neural network, modifying neural network architecture, running distributed rollouts, debugging their model.

#### Architecture overview ####

<img width="676" alt="architecture-for-training-an-aws-deepracer-model-using-amazon-sagemaker" src="https://github.com/aws-solutions-library-samples/guidance-for-training-an-aws-deepracer-model-using-amazon-sagemaker/assets/58491864/86a36774-5b60-4352-bd76-57634ec38c88">

#### Cost ####

You are responsible for the cost of the AWS services used while running this Guidance.

As of 10/30/2023, the cost for running this guidance with the default settings in the US East (N. Virginia) is approximately $31.27 per month for training  5 models, 1 hour each, with training spread across 5 days. Remember to shut down your SageMaker Notebook instance each day.


| Service | Assumptions | Cost Per Month |
| ------------- | ------------- | ------------- | 
| Amazon SageMaker Studio Notebook |	1 Notebook Instance used for 25 hours	| $9.97 |
| Amazon SageMaker Training | 5 jobs per month x 1 instance per job x 1 hour per job, 32 GB SSD storage |	$6.87 |
| AWS Robo Maker	25 Simulation Unit Hours (SU-Hours) |	$10 |
| Amazon CloudWatch |	5GB logs storage |	$2.52 |
| Amazon Simple Storage Service	| 10GB data, 1,000 PUT, 1000 GET requests |	$0.26 |
| Amazon Kinesis Video Streams |	5 hours data Ingestion per day, 5 days storage |	$1.65 |
| VPC |	All traffic is flowing through a Gateway VPC Endpoint |	0 |
|	| | 31.27 |

### Prerequisites (required) ###

This guidance is targeted towards those familiar with the AWS Console and AWS DeepRacer Service. The users are expected to have a basic understanding of AWS DeepRacer, SageMaker, RoboMaker services, and general Machine Learning concepts. It guides users to utilize these services directly to train, and tune their models to a higher level of performance. It should be run in US East N.Virginia region.

#### Operating system ####
Since the guidance runs in the AWS cloud, on an Amazon Sagemaker notebook instance, you should to run it through Mac or Windows instances. Linux is not recommended.

### Deployment steps (required) ###
To deploy the dpr401 AWS CloudFormation stack in order to run the DPR401-notebook instance on Amazon SageMaker:

1. Log in to your personal AWS account in the AWS Console.
1. Search for `AWS CloudFormation` in the search bar on the top of the page. 
1. On the AWS CloudFormation home page, select **Create stack** and choose **With new resources (standard)** from the drop-down menu.
1. On the **Create stack** page, under **Specify template**, select **Amazon S3 URL** under **Template source** and paste `http://dpr401.s3.amazonaws.com/dpr401.yaml` under **Amazon S3 URL**.
1. Select **Next**.
1. On the **Stack description** page, under **Stack name**, enter `dpr401`.
1. Select **Next**.
1. On the **Configure stack options** page, use the default setttings and select **Next**.   
1. On the **Review dpr401**, review your stack and under **Capabilities**, check the box to acknowledge that AWS CloudFormation might create IAM resources with custom names.
1. Choose **Submit**. 

### Deployment validation ###

To validate that your AWS CloudFormation stack and Amazon Sagemaker notebook instance were created successfully:

1. To validate that your AWS CloudFormation stack, log in to your personal AWS account in the AWS Console.
1. Search for `AWS CloudFormation` in the search bar on the top of the AWS Console page.
1. On the **Stacks** page, under **Stack name**, verify you have a stack titled **dpr401** with a **Status** of **CREATE_COMPLETE**.
1. Select the stack **dpr401**.
1. On the **dpr401** page, confirm under the **Resources** tab that **DPR401Notebook** and **DPR401Role** are listed. 
1. Next, to validate your Amazon Sagemaker notebook instance, search for `Amazon Sagemaker` in the search bar at the top of the page.
1. On the Amazon Sagemaker console homepage, on the left navigation menu, expand the **Notebook** section and select **Notebook instances**.
1. On the **Notebook instances** page, confirm **DPR401-Notebook** is listed under **Name**. 


### Running the guidance ###

To run the Guidance for training an AWS DeepRacer model using Amazon SageMaker:

1. On the Amazon Sagemaker console homepage, on the left navigation menu, expand the **Notebook** section and select **Notebook instances**.
1. On the **Notebook instances** page, find **DPR401-Notebook** under **Name** and select the **Open Jupyter** link for the notebook. This will open a new browser tab and place you in the Jupyter notebook interface.
1. Select **guidance-for-training-an-aws-deepracer-model-using-amazon-sagemaker** under the **Files** tab.
1. Under the **files** tab, open the **dpr401** folder.
1. Choose the **deepracer_rl.ipynb** file to open the notebook in a new tab.
1. To initialize and build the environments, scroll down to the **Workshop Checkpoint #1** cell and select it.
1. Select the **Cell** in the menu bar at the top of the notebook and choose **Run All** from the dropdown menu. This will execute all the cells up to this point in the notebook loading in all the modules needed by the notebook, seting up the environment, and downloading and building the docker containers.  Initializing and building the environments will take up to 20 minutes. 

#### Train the RL model ####

The training process involves using AWS RoboMaker to emulate driving experiences in the environment, relaying the experiences at fixed intervals to Amazon SageMaker as input to train the deep neural network, and updating the network weights to an S3 location.

Select the Workshop Checkpoint #1 cell, and click the ▶ Run button until you reach Workshop Checkpoint #2. This will execute the following steps:


1. Configure presets: One can modify the presets to define initial agent parameters. It is commented out by default so the default parameters are used. One can use this to manually "pre-train" the neural network.
2. Copy custom files to S3 bucket so that Amazon SageMaker and AWS RoboMaker can pick it up: This is where the chosen reward function and model_metadata.json file are copied to S3 where Amazon SageMaker and AWS Robomaker can pick them up. These files are discussed in the later section "Customize Your Training."
3. Train the RL model using the Python SDK Script mode: This section first defines the metric_definitions and the custom_hyperparameters, then starts the sagemaker training job. Visit the DeepRacer documentation for detailed information on the hyperparameters.
4. Configure the AWS Robomaker Job: This creates the simulation application from the docker container, and sets up the kinesis video streams.
5. Launch the Simulation job(s) on AWS RoboMaker: This starts the simulation job.
6. Visualizing the simulations in AWS RoboMaker: This provides a link for accessing the AWS RoboMaker environment, and another for watching the Kinesis Video Stream.
7. Creating temporary folder top plot metrics: This creates a local temp folder used for storing metrics from the training job.

#### Allow the job to run ####

1. It will take a few minutes for the Amazon SageMaker job and the AWS RoboMaker jobs to boot up and start training. One can see this in the Amazon Sagemaker training and AWS RoboMaker simulation jobs consoles. A status of **Preparing** indicates the environment is still starting.
2. At this point, allow your Amazon SageMaker and AWS RoboMaker jobs to perform training. The default is 600 seconds (10 minutes), set by job_duration_in_seconds = 600 earlier in the notebook.
3. Feel free to visit the AWS RoboMaker job and the Kinesis Video stream in the meantime to watch the training progress.

#### After training is complete ####

After training is complete, click on the Workshop Checkpoint #2 cell and select the ▶ Run button until you have reached Workshop Checkpoint #3.

If you would like to import your trained model into the AWS DeepRacer console, visit Import Model and paste in the S3 path provided in the Upload Your Model into the DeepRacer console cell.


#### Evaluate your models ####

After training your model you can evaluate the current state of the training by using an evaluation simulation.

Select the Workshop Checkpoint #3 cell and click the ▶ Run button until you have reached Workshop Checkpoint #4. This will start an evaluation job.

There are several similarities between the training Simulation Job parameters, including the world_name and race_type. Since you are evaluating a trained model you can run an evaluation against the same world name and race type. You can also run an evaluation using a different world name and race type.

![spain-track](https://github.com/aws-solutions-library-samples/guidance-for-training-an-aws-deepracer-model-using-amazon-sagemaker/assets/58491864/b0963936-d7b0-45ab-a984-24a9b1b65958)

  
There are additional parameters that you can change to customize the evaluation.

![yaml_config](https://github.com/aws-solutions-library-samples/guidance-for-training-an-aws-deepracer-model-using-amazon-sagemaker/assets/58491864/c4e05ecc-8538-4079-8cc9-8fbe0c10fc59)

#### Key	value pairs ####

| Key | Value |
| ------------- | ------------- |
| yaml_config['NUMBER_OF_TRIALS']	| Set the number of laps for evaluation |
| yaml_config['DISPLAY_NAME']	| Displayed in the upper left corner to identify the current racer |
| yaml_config['LEADERBOARD_TYPE']	| **Leave as "LEAGUE"** |
| yaml_config['LEADERBOARD_NAME']	| Displayed on the bottom area of the media output |
| yaml_config['CAR_COLOR']	| Controls the color of the racecar |
| yaml_config['NUMBER_OF_RESETS']	| The number of resets allowed per lap |
| yaml_config['PENALTY_SECONDS']	| **Leave as "5"** |
| yaml_config['OFF_TRACK_PENALTY']	| Number of seconds to add to the race time when the race car leaves the track |
| yaml_config['COLLISION_PENALTY']	| Number of seconds to add to the race time when the race car collides with an obstacle like a box in the OBJECT_AVOIDANCE race type |

Below is an example of the evaluation job media output using the parameters set in the notebook.
![image (13)](https://github.com/aws-solutions-library-samples/guidance-for-training-an-aws-deepracer-model-using-amazon-sagemaker/assets/58491864/8dcb92df-cea3-43e7-8bb4-db760240cfb4)

#### Plot metrics for evaluation job ####

For evaluation jobs the metrics you plot are based on the time your race car takes to go around the track including the penalties. This is different than the training job because the evaluation jobs are evaluating the model training not the rewards returned during training.

![image (13)](https://github.com/aws-solutions-library-samples/guidance-for-training-an-aws-deepracer-model-using-amazon-sagemaker/assets/58491864/03a47258-5a83-4a5e-badc-4dfc68dc572f)

#### Head to head evaluation ####

One will note after Workshop Checkpoint #4 that the notebook contains a Head-to-head evaluation. This is out of scope for the DPR401 workshop, but if you have two models you want to train, you can configure the s3 path to the second model and perform a head-to-head evaluation.


#### View your output logs and videos ####

Open up the Amazon S3 Console and select your **sagemaker-us-east-1-<ACCOUNTID>** bucket.
Select the **deepracer-notebook-sagemaker-<DATE>-<TIME>** prefix and explore:

1. The **training_metrics.json** and **evaluation_metrics.json** files contain metrics on the behavior of the model, mainly suitable for showing progress during training or evaluation.
2. The **sim-<jobid>** folders contain the simulation logs for the AWS RoboMaker environment.
3. The **iteration-data** folder contains videos of the training and evaluation jobs and the simulation trace logs. The logs are much more detailed than the json files mentioned above, and are suitable for analysis. There are three video vides: a top-down view, a 45 degree from behind view, and the same 45 degree from behind view but with a track overlay.
4. The **model** folder contains the unfrozen tensorflow graph. This is what is trained further or imported into the DeepRacer console.
5. The **train-output** folder (a few folders deep) contains the **model.tar.gz** file, appropriate for loading onto a physical AWS DeepRacer vehicle and optimization with the Intel OpenVino toolkit.



### Next steps ###
Now that one has successfully trained and evaluated a AWS DeepRacer model using the notebook, one can explore how to customize training in a variety of areas in this section.

This section will show one what areas to modify for customizations; once one makes the modifications one will need to go back and re-run the appropriate sections of the notebook to apply the modifications. It is intended as a general guidebook for one to pursue their own path for customization, not a prescriptive set of steps. Feel free to "think big" and brainstorm on the possibilities.

#### Create a reward function ####

In general, you design your reward function to act like an incentive plan. You can customize your reward function with relevant input parameters passed into the reward function.
Reward function files

1. Explore to **src/artifacts/rewards/** in the notebook to see example reward functions.

   ![image (14)](https://github.com/aws-solutions-library-samples/guidance-for-training-an-aws-deepracer-model-using-amazon-sagemaker/assets/58491864/a1b9419b-385a-4c6b-9c76-b93f50f0e7fe)


1. Once you pick or modify one, locate the cell in the notebook labeled **Copy custom files to S3 bucket** so that Amazon SageMaker and AWS RoboMaker can pick it up and modify this line as appropriate to copy your new reward function appropriately.
2. `!aws s3 cp ./src/artifacts/rewards/default.py {s3_location}/customer_reward_function.py`

#### Example reward functions ####
```
Follow the Center Line in Time Trials
follow_center_line.py
```

This example determines how far away the agent is from the center line, and gives higher reward if it is closer to the center of the track, encouraging the agent to closely follow the center line.


**Stay inside the two borders in time trials**
`stay_inside_two_border.py`
This example simply gives high rewards if the agent stays inside the borders, and let the agent figure out what is the best path to finish a lap. It is easy to program and understand, but likely takes longer to converge.

**Prevent zig-zag in time trials**
`prevent_zig_zag.py`
This example incentivizes the agent to follow the center line but penalizes with lower reward if it steers too much, which helps prevent zig-zag behavior. The agent learns to drive smoothly in the simulator and likely keeps the same behavior when deployed in the physical vehicle.

**Stay On One Lane without Crashing into Stationary Obstacles or Moving Vehicles**
`object_avoidance_head_to_head.py`
This reward function rewards the agent to stay between the track borders and penalizes the agent for getting too close to the next object in the front. The agent can move from lane to lane to avoid crashes. The total reward is a weighted sum of the reward and penalty. The example gives more weight to the penalty term to focus more on safety by avoiding crashes. You can play with different averaging weights to train the agent with different driving behaviors and to achieve different driving performances.

#### Create your own reward function ####

If you wish to create your own reward function there is a pattern to the function that you must use:

```
12345def reward_function(params) :

    reward = ...

    return float(reward)
```

A list of parameters and their definitions is located here: [https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-reward-function-input.html]

#### Add additional python modules ####

Several python modules are included in the AWS RoboMaker simapp, but if you want to add more, locate the cell labeled Run these commands if you wish to modify the Amazon SageMaker and AWS Robomaker code and add in additional `!docker cp` commands to copy the modules you want into the container.

#### Use another programming language ####

If one wants to use another programming language for their reward function, one can the python boto3 library to invoke an Amazon Lambda function. Such a method may look like the following:

```
import boto3,jsonlambdaservice = boto3.client('lambda')
def reward_function(params):
    response = lambdaservice.invoke(FunctionName='YourFunctionHere',
                      Payload=json.dumps(params))
        return(float(response["Payload"]))
```

One will need to modify the IAM role for the notebook to have Lambda invoke permissions.

Alternatively, one could load the alternate program and any required interpreters and libraries into the docker container, then call out from the python reward function with an os.system() or subprocesses.run() call. In such a case, one needs to consider how to pass the parameters and receive the return value, perhaps by writing temp files to disk or by assigning environmental variables. Note that the reward function is run 10 to 15 times a second during training, so the overhead introduced by calling another executable may be an issue. Due to this overhead, most reinforcement learning researchers stick to Python, as this is the language the rl_coach framework is written in.


#### Modify the training algorithm ####

Training adjusts the weights and biases of the neural network so that the correct decisions are made. There are many methods, or algorithms, for how to determine which weights and biases should be adjusted and by how much.

The default algorithm for DeepRacer training is PPO, or Proximal Policy Optimization. This algorithm works with both discrete and continuous action spaces, and tends to be stable but data hungry.

The SAC, or Soft Actor Critic, algorithm is also available. This algorithm only works with a continuous action spaces, and is less stable but also requires less training to learn.

Read more about PPO versus SAC at [https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-how-it-works-reinforcement-learning-algorithm.html] 


#### How to set an algorithm ####

#### PPO policy ####

The default algorithm is PPO; if no training algorithm is set in the model_metadata.json file, this is the algorithm used. The metric_definitions and customer_hyperparameter in the notebook in the Train the RL model using the Python SDK Script mode cells are coded for PPO.

#### SAC policy ####

If you want to change the training algorithm to SAC, first modify the model_metadata.json file with "training_algorithm" : "sac" and a continuous action space, such as:

```
{  "action_space" : {
    "steering_angle" : {
      "high" : 30.0,
      "low" : -30.0
    },
    "speed" : {
      "high" : 1.0,
      "low" : 0.5
    }
  },
  "sensor" : [ "FRONT_FACING_CAMERA" ],
  "neural_network" : "DEEP_CONVOLUTIONAL_NETWORK_SHALLOW",
  "version" : "4",
  "training_algorithm" : "sac",
  "action_space_type" : "continuous",
  "preprocess_type" : null,
  "regional_parameters" : null
}
```

Find example model_metadata.json files in **src/artifacts/actions**, such as the **front-shallow-continuous-sac.json** file. After choosing an example, modifying it, or creating a new one, locate the cell labeled **Copy custom files to S3 bucket** so that Amazon SageMaker and AWS RoboMaker can pick it up and modify the following line to instead copy the file you intend:

`!aws s3 cp ./src/artifacts/actions/default.json {s3_location}/model/model_metadata.json`

Additionally, modify the hyperparameters in the notebook (Look two cells below the label **Train the RL model using the Python SDK Script** mode) to include the SAC hyperparameters instead:

```
custom_hyperparameter = {    "s3_bucket": s3_bucket,
    "s3_prefix": s3_prefix,
    "aws_region": aws_region,
    "model_metadata_s3_key": "%s/model/model_metadata.json" % s3_prefix,
    "reward_function_s3_source": "%s/customer_reward_function.py" % s3_prefix,
    "batch_size": "64",
    "lr": "0.0003",
    "exploration_type": "Additive_noise",
    "e_greedy_value": "0.05",
    "epsilon_steps": "10000",
    "discount_factor": "0.999",
    "sac_alpha": "0.2",
    "stack_size": "1",
    "loss_type": "Mean squared error",
    "num_episodes_between_training": "20",
    "term_cond_avg_score": "100000.0",
    "term_cond_max_episodes": "100000"
}
```

### Cleanup ###
Deprovision resources so your account does not continue to be charged after completing the workshop.
In the notebook, scroll down and select the cell for **Workshop Checkpoint #5**. Click the **▶ Run** button to execute the rest of the cells in the notebook. This will cancel the AWS RoboMaker and Amazon SageMaker jobs (if still running), delete the Amazon Kinesis video streams, delete the Amazon Elastic Container (ECR) repositories, and delete the AWS RoboMaker simapp.

#### Clean up the Amazon S3 bucket ####

Consider uncommenting the **Clean your S3 bucket** cell and executing it if you want to empty the Amazon S3 bucket of generated logs and data, including the trained model. You may also choose to visit the  S3 Console  [https://s3.console.aws.amazon.com/s3/buckets] and delete the bucket.

If you choose not to do this, you may incur S3 storage costs.

#### Delete any imported DeepRacer models ####

If you imported a model into the DeepRacer console, delete it by visiting **DeepRacer** > **Your Models**, selecting the model, and choosing **Delete** under the **Actions** menu. If you choose not to do this, you may incur AWS DeepRacer model storage costs.

#### Terminate and Delete the Role and Notebook ####

Visit **CloudFormation Stacks** and select the radio button for the **dpr401** stack. Select the **Delete button**. This will terminate and delete the Amazon Sagemaker notebook and delete the IAM role.

## Developer tools

[Log Analyzer and Visualizations](https://github.com/aws-samples/aws-deepracer-workshops/tree/master/log-analysis/)

## License summary

This sample code is made available under a modified MIT license. See the LICENSE file.
