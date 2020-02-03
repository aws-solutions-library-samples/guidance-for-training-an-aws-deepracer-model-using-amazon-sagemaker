# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License"). You
# may not use this file except in compliance with the License. A copy of
# the License is located at
#
#     http://aws.amazon.com/apache2.0/
#
# or in the "license" file accompanying this file. This file is
# distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF
# ANY KIND, either express or implied. See the License for the specific
# language governing permissions and limitations under the License.

def generate_s3_write_permission_for_sagemaker_role(role, iam_policy_name):
    role_name = role.split("/")[-1]
    url = "https://console.aws.amazon.com/iam/home#/roles/%s" % role_name
    text = "1. Go to IAM console to edit current SageMaker role: [%s](%s).\n" % (role_name, url)
    text += "2. Next, go to the `Permissions tab` and click on `Attach Policy.` \n"
    text += "3. Search and select `{}` policy\n".format(iam_policy_name)
    return text

