## Contents

The notebook, `deepracer-lidar-tf12-adv.ipynb`, provides training and evaluations an autonomous deepracer with multiple sensor types for head-to-head and object avoidance experience.

The link for the simulation application: https://neurips-aido3-awsdeepracer.s3.us-east-2.amazonaws.com/build_REINVENT400_MULTICAR.tar.gz

The notebook, `log_analysis_EVAL.ipynb`, provides instructions on choosing candidates for successful simulation to real-world transfer.


## How to use the notebook

1. Login to your AWS account - SageMaker service ([SageMaker Link](https://us-west-2.console.aws.amazon.com/sagemaker/home?region=us-west-2#/dashboard))
2. On the left tab select `Notebook instances`
3. Select `Create notebook instance`
4. Fill up the notebook instance name. In the Additional configuration select atleast 25 GB. This is because docker gets installed and takes up space.
5. Create a new IAM role. Give root permission.
6. Select the git repository and clone this repository.
7. Then click create notebook instance button at the button
8. This takes like 2 min to create your notebook instance. Then click on the newly created instance and click on the juypter notebook.
9. You will see all the github files and now run `deepracer-lidar-tf12-adv.ipynb`
10. Run clean robomaker & sagemaker commands in the script only when you are done with training.



