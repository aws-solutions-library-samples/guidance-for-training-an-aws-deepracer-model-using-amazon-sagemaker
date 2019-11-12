# Customizing AWS DeepRacer using Amazon SageMaker RL and AWS RoboMaker services


This folder contains example notebooks and simulation assets on how to customize AWS DeepRacer by expanding to train new tracks with varying textures, add noise to the vision-based observations or actions, modify neural networks and define custom network layers. 

While these examples are prepared to assist the participants of the AIDO 3 Challenge at NeurIPS'2019, the notebooks are generic for providing guidelines on customization of the AWS DeepRacer using Amazon SageMaker and AWS RoboMaker.



## Contents

* `challenge_train_w_PPO`: Amazon SageMaker notebook and simulation application resources for training with customized simulation environments in AWS RoboMaker and using Clipped PPO RL algorithm.

* `challenge_train_DQO`: Amazon SageMaker notebook for training with DQN RL agent and instructions on how to modify the notebook from using the Clipped PPO RL agent to DQN agent. Refer to these instruction to switch to another algorithm in Amazon SageMaker RL libraires.

* `challenge_eval_submit`: Amazon SageMaker notebook to run evaluation on trained models and iPython notebooks for analyzing the evaluation logs. 


## Prepare submission for AIDO 3 Challenge at NeurIPS'2019

To prepare you submission, run the following notebooks:

* `challenge_eval_submit\0_run_evaluation-NeurIPS.ipynb` to run evaluation on your trained models.

* `challenge_eval_submit\1_logs_evaluation-NeurIPS.ipynb` to run analysis on your evaluation logs.

* `challenge_eval_submit\2_submit-NeurIPS.ipynb` to prepare your submission.









