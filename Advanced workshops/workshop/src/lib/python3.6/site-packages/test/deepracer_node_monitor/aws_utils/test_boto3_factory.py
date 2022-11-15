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
from unittest import TestCase
from unittest.mock import MagicMock, patch, call

# rosnode is imported in NodeMonitor class, but this package is not a ROS environment.
# Since this is missing the rosnode, mocking the rosnode module.
# https://stackoverflow.com/questions/8658043/how-to-mock-an-import
import sys
sys.modules['rosnode'] = MagicMock()
sys.modules['botocore'] = MagicMock()
sys.modules['boto3'] = MagicMock()

from deepracer_node_monitor.aws_utils.boto3_factory import Boto3Factory
from deepracer_node_monitor.aws_utils.constants import (
    BOTO_MAX_RETRY_ATTEMPTS, BOTO_RETRY_CONNECT_TIMEOUT)


@patch("deepracer_node_monitor.aws_utils.boto3_factory.RoboMakerUtils.get_aws_region")
@patch("deepracer_node_monitor.aws_utils.boto3_factory.boto3")
@patch("deepracer_node_monitor.aws_utils.boto3_factory.botocore")
class Boto3FactoryTest(TestCase):
    def test_create_boto3_client(self, botocore_mock, boto3_mock, get_aws_region_mock):
        get_aws_region_mock.return_value = "us-east-1"
        Boto3Factory.create_boto3_client("test")
        boto3_mock.Session().client.assert_has_calls([
            call("test", region_name="us-east-1",
                 config=botocore_mock.config.Config(retries=dict(max_attempts=BOTO_MAX_RETRY_ATTEMPTS),
                                                    connect_timeout=BOTO_RETRY_CONNECT_TIMEOUT))])
