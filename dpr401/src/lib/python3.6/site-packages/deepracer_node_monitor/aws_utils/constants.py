#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################
""" Constant module """

# https://boto3.amazonaws.com/v1/documentation/api/latest/guide/retries.html for more retries info
# retry attempts with default 5 for legacy and 3 for standard
BOTO_MAX_RETRY_ATTEMPTS = 5
# retry connect timeout with default as 60
BOTO_RETRY_CONNECT_TIMEOUT = 60


class Boto3Client(object):
    """
    Boto3Clients like s3, cloudwatch
    """
    S3 = "s3"
    CLOUDWATCH = "cloudwatch"


CW_METRIC_NAMESPACE = "AwsSilverstoneSimApp"
CW_METRIC_NAME = "AwsSilverstoneEvalJobs"
