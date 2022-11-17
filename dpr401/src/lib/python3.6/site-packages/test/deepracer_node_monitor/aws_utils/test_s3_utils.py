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
from unittest.mock import MagicMock, patch

# rosnode is imported in NodeMonitor class, but this package is not a ROS environment.
# Since this is missing the rosnode, mocking the rosnode module.
# https://stackoverflow.com/questions/8658043/how-to-mock-an-import
import sys
sys.modules['rosnode'] = MagicMock()
sys.modules['botocore.config'] = MagicMock()
sys.modules['boto3'] = MagicMock()

from deepracer_node_monitor.aws_utils.s3_utils import S3Utils


@patch("deepracer_node_monitor.aws_utils.s3_utils.os")
class S3UtilsTest(TestCase):
    def test_get_s3_heartbeat_location_path(self, os_mock):
        os_mock.environ.get.return_value = "JOB_STATUS_S3_LOCATION"
        self.assertEqual("JOB_STATUS_S3_LOCATION", S3Utils.get_s3_heartbeat_location_path())

    def test_get_s3_job_location_path_empty(self, os_mock):
        os_mock.environ.get.return_value = ''
        self.assertEqual("", S3Utils.get_s3_heartbeat_location_path())

    @patch("deepracer_node_monitor.aws_utils.s3_utils.S3Utils.get_s3_heartbeat_location_path")
    def test_get_heartbeat_s3_info(self, get_s3_heartbeat_location_path_mock, os_mock):
        get_s3_heartbeat_location_path_mock.return_value = "s3://aws-deepracer-bba2e912-6ef0-4c3c-a072-ce17e254bcf2/node_monitor/job_status.txt"
        bucket = "aws-deepracer-bba2e912-6ef0-4c3c-a072-ce17e254bcf2"
        key = "node_monitor/job_status.txt"
        self.assertEqual((bucket, key), S3Utils.get_heartbeat_s3_info())

    @patch("deepracer_node_monitor.aws_utils.s3_utils.json")
    def test_get_s3_heartbeat_file_content(self, json_mock, os_mock):
        json_mock.dumps = MagicMock()
        json_mock.dumps.return_value = "hello"
        json_data = S3Utils.get_s3_heartbeat_file_content("SUCCESS", "SUCCESS_MSG")
        json_mock.dumps.assert_called_once()
        self.assertEqual(json_data, "hello")
