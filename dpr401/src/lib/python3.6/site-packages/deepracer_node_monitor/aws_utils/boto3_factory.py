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
"""A class for Boto3Factory."""

import botocore
import boto3

from deepracer_node_monitor.aws_utils.constants import (
    BOTO_MAX_RETRY_ATTEMPTS, BOTO_RETRY_CONNECT_TIMEOUT)
from deepracer_node_monitor.aws_utils.robomaker_utils import RoboMakerUtils


class Boto3Factory(object):
    """
    This class implements a sensor config factory
    """
    @staticmethod
    def create_boto3_client(client_name: str,
                            max_attempts: int = BOTO_MAX_RETRY_ATTEMPTS,
                            connect_timeout: int = BOTO_RETRY_CONNECT_TIMEOUT) -> botocore:
        """
        Create boto3 client for appropriate passed service name

        Args:
            client_name (str): Name of the boto3 client, like s3, cloudwatch etc.
            max_attempts (int): Maximum retry attempts (Default: BOTO_MAX_RETRY_ATTEMPTS)
            connect_timeout (int): Maximum timeout (Default: BOTO_RETRY_CONNECT_TIMEOUT)

        Returns:
            botocore: client session of botocore
        """
        # aws-cli/1.18.69 does not have mode="standard" option for retries
        # TODO - when awscli is upgraded to 2.0, make sure to add the mode="standard"
        return boto3.Session().client(client_name,
                                      region_name=RoboMakerUtils.get_aws_region(),
                                      config=botocore.config.Config(retries=dict(max_attempts=BOTO_MAX_RETRY_ATTEMPTS),
                                                                    connect_timeout=BOTO_RETRY_CONNECT_TIMEOUT))
