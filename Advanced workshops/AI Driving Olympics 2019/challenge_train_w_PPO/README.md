# Customize AWS DeepRacer using Amazon SageMaker RL and AWS RoboMaker services

This folder contains a notebook that demonstrates how to customize AWS DeepRacer RL assets for domain adaptation to various track shapes and textures.


## Contents

* `deepracer_rl_customized_training_notebook.ipynb`: Notebook for training autonomous race car.

* `Dockerfile`: Custom docker based of an Amazon SageMaker default docker. Make sure the Amazon Sagemaker docker file is in the same region as your notebook. Otherwise, you will get an error message. 

* `src/`: 
  * `training_worker.py`: Main entrypoint for starting distributed training job
  * `markov/`: Helper files for S3 upload/download
   * `presets/default.py`: Preset (configuration) for neural network and agent hyperparameters
   * `rewards/default.py`: Custom reward function
   * `environments/deepracer_racetrack_env.py`: Gym environment file for DeepRacer
   * `actions/model_metadata_10_state.json`: JSON file to customize your action space & the speed
  * `lib/`: redis configuration file and ppo_head.py customized tensorflow file copied to sagemaker container.

* `common/`: Helper function to build docker files.

## How to use the notebook

1. Login to your AWS account - SageMaker service ([SageMaker Link](https://us-west-2.console.aws.amazon.com/sagemaker/home?region=us-west-2#/dashboard))
2. On the left tab select `Notebook instances`
3. Select `Create notebook instance`
4. Fill up the notebook instance name. In the `Additional Configuration` dropdown menu, select at least 25GB. This is because docker gets installed and takes up space. If running into space constraints, please delete all the dockers and re-try.
5. Create a new IAM role. Give root permission.
6. Select the git repository and clone this repository.
7. Then, click create notebook instance button at the button
8. This takes like 2 min to create your notebook instance. Then, click on the newly created instance and click on the juypter notebook.
9. You will see all the github files and now run `deepracer_rl_customized_training_notebook.ipynb`
10. Run clean AWS Robomaker & Amazon Sagemaker commands in the script only when you are done with training.
