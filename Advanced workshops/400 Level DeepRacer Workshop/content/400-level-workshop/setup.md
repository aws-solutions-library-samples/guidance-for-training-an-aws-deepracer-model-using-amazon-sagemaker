---
title: "Prep: Workshop Setup"
chapter: true
weight: 1
description: "We will start by provisioning an Amazon SageMaker notebook instance."
---

# Workshop Setup

Before we get started, we need to configure a few things.  

<div id="no_code">

### Log  in to the AWS Console and set the AWS Region

When using your own AWS account to complete this workshop, your user need read and write permissions to several AWS services. The AWS CloudFormation stack will create an Amazon SageMaker notebook instance with the required permissions applied.

### Select the AWS Region 
This workshop uses the US East (N. Virginia) region. 
Select the region in the upper right corner of the AWS Console.

### Launch CloudFormation Stack 
AWS CloudFormation provides a common language to describe and provision infrastructure resources in your cloud environment. CloudFormation allows you to use a simple text file to model and provision the resources needed for the workshop.  For this workshop, we've pre-created a template that simplifies some of the setup.  The infrastructure it creates is needed to run the activities.  

Once you have successfully changed the AWS Region to US East (N. Virginia), click the button below to launch a CloudFormation stack to create the required resources.

[![Launch Stack](../../images/launch-stack.svg)](https://console.aws.amazon.com/cloudformation/home#/stacks/new?templateURL=https://temp-400-dr-workshop.s3.us-east-1.amazonaws.com/drnotebookeast1.json&region=us-east-1)

1. On the *Create stack* page, accept the defaults and click **Next**.
2. On the *Specify stack details* page, set *Stack name* to a value that will help you identify this stack, such as "deepracer-wrkshp-resources".
3. In the *Parameters* section, enter values for the following parameters and they will be applied to your AWS environment.
4. For the *DeepRacerS3BucketName* field, the value must be globally-unique, and it must be lower case.  This is because it will be used in the domain name for the S3 bucket that gets created.  For today's workshop, namespace your bucket with your initials or a user name to improve its uniqueness.  For example, if your name is Jane Penelope Smith, you might name the bucket, "jps-deepracer-workshop".  
5. For the *NotebookName* field, the value does not have to be globally unique but it cannot be a duplicate of an Amazon SageMaker notebook in your individual AWS account. Click **Next**.
6. On the *Configure stack options* page, use the default values and click **Next**.
6. On the *Review* page, review the choices, and check the box at the bottom of the page to "acknowledge that AWS CloudFormation might create IAM resources with custom names".
7. Click **Create stack**.


This will create:

- a **VPC** with a pair of **subnets** and a **default security group**.
- an **S3 bucket** to store your AWS SageMaker notebook assets.

The stack creation should only take a minute or two.  Once the status has changed to CREATE_COMPLETE, click on the stack's  **Outputs** tab. It will provide you with several Key/Value pairs that we will use later in the workshop.  Specifically, you should copy and paste several Key/Values to a notepad application.  This is not required, but be prepared to navigate back to the CloudFormation **Outputs** tab when you're asked for these values later.  

You will need the values for the following:

- S3BucketName
- Notebook Name



| ![Open SageMaker Notebook](/images/400workshop/aws-sagemaker-notebooks.png) | **Section: Workshop Setup** |
|---|---|

1. Launch a new web browser window and navigate to the SageMaker Notebook Instance by selecting Amazon Sagemaker in the Services drop down in the AWS Console

2. In the left menu select Notebook Instances

3. Verify the region in the AWS Console is US East (N. Virginia).

4. Select Open Jupyter or Open JupyterLab.

5. Open the Jupyter Notebook ``400_deepracer_rl.ipynb``

6. Complete the **Section: Workshop Setup** in the notebook.

**Congratulations!** You have completed the setup portion of the workshop.

**[Continue to the next module.](../modifyactionspace/)**


</div>
