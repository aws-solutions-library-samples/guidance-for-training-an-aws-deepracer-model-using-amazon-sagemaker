# How to Create Child Accounts so Multiple Enterprise Customer Employees Can Race in Virtual AWS DeepRacer Community Competitions

[**PDF**](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-child-accounts.html)

If your company doesn’t use a single AWS account per employee, you can create individual child accounts for them within an AWS Organization, so each employee has their own account to train and race with. 

## Prerequisites
- Work with your AWS account team to determine if adding child accounts is best the fit for your company's account structure.
- Have [AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-install.html) installed on your computer. Familiarity with AWS CLI is recommended.
- Install [jq](https://stedolan.github.io/jq/download/) on your computer.
- Have [Python](https://www.python.org/downloads/) installed on your computer. 
- Have the necessary permissions to [create an organization](https://docs.aws.amazon.com/organizations/latest/userguide/orgs_manage_create.html) using AWS Organizations. Familiarity with AWS Organizations is recommended.
- In the AWS DeepRacer console, training and racing activities are managed at the AWS account level. This means every individual who has access to a specific AWS account will see all AWS DeepRacer models in that account and will share a single racer alias for all race submissions. In this topic, enterprise customers, whose employees share one account ID using individual IAM roles, can learn to enter multiple racers into virtual AWS DeepRacer community competitions.
- If each employee has their own AWS account ID, you do not need to set up child accounts. Individuals with their own AWS DeepRacer account ID can submit a model to any type of leaderboard.
- If all employees share one primary AWS account ID using different IAM roles, all employees share a single leaderboard submission for all leaderboards, including community leaderboards. Using AWS Organizations and the provided scripts to create individual accounts as a child to the primary account may be the best way to allow multiple employees to submit separate models to a leaderboard at the same time.
- Download and extract the scripts from ChildAccounts.zip

**Important**
The provided scripts only works with the entity, *IAM role*. Some enterprise customers set up their accounts using the *IAM user* entity. For help converting *IAM users* to *IAM roles* contact your AWS account team.

**To create child accounts for a DeepRacer engagement**
1. We have included a trust_policy_template.json file in the templates subdirectory. You may need to edit this policy for your environment. For example, if you're using a Security Assertion Markup Language (SAML) provider or another system, add it to the trust policy. 
1. You may wish to consult the developer guide topic for AWS Organizations on [Creating an Organization](https://docs.aws.amazon.com/organizations/latest/userguide/orgs_manage_create.html).
1. If you are using AWS CLI or AWS API, choose the **AWS CLI, AWS API** tab and follow the steps to create an organization. If you are using the console, follow the steps in *To create an organization*, the tab for which is selected by default.
1. Next, find steps to create new accounts for your enterprise customer employees for AWS CLI, AWS API, and in the management console in [Creating an AWS account in your organization](https://docs.aws.amazon.com/organizations/latest/userguide/orgs_manage_accounts_create.html#orgs_manage_accounts_create-new). By default, you are limited to four child accounts. To raise the account limit:
   1. Log into to the AWS Management Console and navigate to [Service Quotas](https://console.aws.amazon.com/servicequotas/#!/services/organizations/quotas).
   1. Choose Default maximum number of accounts.
   1. Next, choose Request quota increase.
   1. Enter a number that's a more appropriate limit for the number of accounts you want to create into **Change quota value**.
   1. Choose **Request**.
   1. Once you have been granted the service quota increase, you may continue with the rest of the setup steps.
1. Using the CLI via a terminal window, execute the following commands:
```
curl "https://aws.amazon.com/deepracer/accounts-v3.zip" -o "deepracer-accounts-v3.zip"
unzip deepracer-accounts-v3.zip
chmod +x make_child_accounts.sh
chomd +x update_child_accounts.sh
chmod +x cleanup_maker.sh
```
11. Create the following environment variables. For Linux or Mac, execute the following and ensure you replace the text including the <> with the appropriate values. 
> **Note:** Setting 'USE_SAGEMAKER=false' will provide basic  read-only permission to AWS DeepRacer and Amazon Simple Storage Service (Amazon S3), and only in us-east-1. Amazon S3 can be removed from the policy template if you don't want users to have the ability to import pre-existing models. 
>
>If you also want to have your Racers train directly with Sagemaker (see https://aws.amazon.com/blogs/machine-learning/custom-deep-reinforcement-learning-and-multi-track-training-for-aws-deepracer-with-amazon-sagemaker-rl-notebook/), then set 'USE_SAGEMAKER=true'. This will provides additional permissions to access SageMaker and Robomaker beyond what is provided in the basic policy. 
```
export ACCOUNT_NAME=<your account name>
export DEEPRACER_ROLE=OrganizationAccountAccessRole
export EMAIL_ADDRESS_PREFIX=<your email address>
export NAMED_PROFILE=<named profile>
export NUMBER_OF_CHILD_ACCOUNTS=<count of child accounts e.g. 25>
export OU_ID=<ou id - starts with 'ou-...'>
export USE_SAGEMAKER=<Use only 'true' without the single quotes if you wish to permit SageMaker and Robomaker  use. Otherwise use 'false'>
```
12. Once you have  exported the environment variables, you are now ready to execute the following scripts. If you wish to create new child accounts or add additional child accounts, run the following command. **Note:**  This script creates the appropriate policy, and a role for each child account. And, if the script did not complete the correct number of child accounts, the export above can be modified and this script will add new child accounts until the count equals 'NUMBER_OF_CHILD_ACCOUNTS'.
```
./make_child_accounts.sh
```
13. Now that you have created the appropriate numberof child accounts, you will need to move all of these accounts into the OU underneath root in the Organizations window in the AWS console. 
14. If you want to update permissions on the existing child accounts, run the following command. It will delete the deepracer policy and role and replace them, depending upon the settings above. 
```
./update_child_accounts.sh
```
### Cleanup
Once your races are complete and if you set 'USE_SAGEMAKER=true' in the enviornment, run the following script to delete all the notebook instances and training jobs. Note that this cleanup script relies on the region set in the CLI configuration (e.g. `aws configure`) and will only delete notebooks and jobs for that region.

```
./cleanup_sagemaker.sh
```