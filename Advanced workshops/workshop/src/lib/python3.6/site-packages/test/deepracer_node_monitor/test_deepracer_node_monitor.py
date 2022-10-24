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
from unittest.mock import patch, MagicMock, call

# rosnode is imported in NodeMonitor class, but this package is not a ROS environment.
# Since this is missing the rosnode, mocking the rosnode module.
# https://stackoverflow.com/questions/8658043/how-to-mock-an-import
import sys
sys.modules['rosnode'] = MagicMock()
sys.modules['botocore.config'] = MagicMock()
sys.modules['boto3'] = MagicMock()


from deepracer_node_monitor import DeepRacerNodeMonitor
from deepracer_node_monitor.constants import (
    JobStatus, JobStatusMsg)
from deepracer_node_monitor.aws_utils.constants import (
    Boto3Client, CW_METRIC_NAMESPACE)


@patch("deepracer_node_monitor.deepracer_node_monitor.RLock")
@patch("deepracer_node_monitor.deepracer_node_monitor.Boto3Factory")
class DeepRacerNodeMonitorTest(TestCase):
    def setUp(self) -> None:
        self.monitor_nodes = ['/gazebo', '/deepracer', '/ude_ros_server', '*/controller_manager',
                              '/agent0/robot_state_publisher']

    @patch("deepracer_node_monitor.deepracer_node_monitor.S3Utils.get_s3_heartbeat_location_path")
    @patch("deepracer_node_monitor.deepracer_node_monitor.CloudWatchJobStatusMetricDimensionData.is_cloudwatch_heartbeat_enabled")
    def test_initialize(self, is_cloudwatch_heartbeat_enabled, get_s3_heartbeat_location_path,
                        Boto3Factory_mock, rlock_mock):
        Boto3Factory_mock.create_boto3_client = MagicMock()
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        get_s3_heartbeat_location_path.assert_called_once()
        is_cloudwatch_heartbeat_enabled.assert_called_once()
        rlock_mock.assert_called_once()
        self.assertEqual(deepracer_node_monitor._monitor_nodes, self.monitor_nodes)
        self.assertFalse(deepracer_node_monitor._is_dead_node_detected)

    @patch("deepracer_node_monitor.deepracer_node_monitor.S3Utils.get_s3_heartbeat_file_content")
    @patch("deepracer_node_monitor.deepracer_node_monitor.S3Utils.get_heartbeat_s3_info")
    def test_upload_s3_job_status(self, get_heartbeat_s3_info_mock, get_s3_heartbeat_file_content_mock,
                                  Boto3Factory_mock, rlock_mock):
        get_s3_heartbeat_file_content_mock.return_value = "test_json"
        get_heartbeat_s3_info_mock.return_value = ("s3_bucket", "s3_prefix")
        Boto3Factory_mock.create_boto3_client = MagicMock()
        Boto3Factory_mock.create_boto3_client(Boto3Client.S3).put_object = MagicMock()
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._is_heartbeat_s3_upload_enabled = True
        deepracer_node_monitor._upload_s3_job_status("SUCCESS", "SUCCESS_MSG")
        Boto3Factory_mock.create_boto3_client.assert_has_calls([
            call(Boto3Client.S3)
        ])
        Boto3Factory_mock.create_boto3_client(Boto3Client.S3).put_object.assert_called_once_with(
            Bucket="s3_bucket",
            Key="s3_prefix", Body=bytes(str("test_json"), encoding="utf-8"))

    @patch("deepracer_node_monitor.deepracer_node_monitor.S3Utils.get_s3_heartbeat_file_content")
    @patch("deepracer_node_monitor.deepracer_node_monitor.S3Utils.get_heartbeat_s3_info")
    def test_upload_s3_job_status_empty_s3_bucket(self, get_heartbeat_s3_info_mock, get_s3_heartbeat_file_content_mock,
                                                  Boto3Factory_mock, rlock_mock):
        get_s3_heartbeat_file_content_mock.return_value = "test_json"
        get_heartbeat_s3_info_mock.return_value = ("", "s3_key")
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._is_heartbeat_s3_upload_enabled = True
        deepracer_node_monitor._upload_s3_job_status("SUCCESS", "SUCCESS_MSG")
        Boto3Factory_mock.create_boto3_client.assert_not_called()
        Boto3Factory_mock.create_boto3_client(Boto3Client.S3).put_object.assert_not_called()

    @patch("deepracer_node_monitor.deepracer_node_monitor.S3Utils.get_s3_heartbeat_file_content")
    @patch("deepracer_node_monitor.deepracer_node_monitor.S3Utils.get_heartbeat_s3_info")
    def test_upload_s3_job_status_empty_s3_key(self, get_heartbeat_s3_info_mock, get_s3_heartbeat_file_content_mock,
                                               Boto3Factory_mock, rlock_mock):
        get_s3_heartbeat_file_content_mock.return_value = "test_json"
        get_heartbeat_s3_info_mock.return_value = ("s3_bucket", "")
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._is_heartbeat_s3_upload_enabled = True
        deepracer_node_monitor._upload_s3_job_status("SUCCESS", "SUCCESS_MSG")
        Boto3Factory_mock.create_boto3_client.assert_not_called()
        Boto3Factory_mock.create_boto3_client(Boto3Client.S3).put_object.assert_not_called()

    @patch("deepracer_node_monitor.deepracer_node_monitor.S3Utils.get_s3_heartbeat_file_content")
    @patch("deepracer_node_monitor.deepracer_node_monitor.S3Utils.get_heartbeat_s3_info")
    def test_upload_s3_job_status_empty_s3_bucket_key(self, get_heartbeat_s3_info_mock, get_s3_heartbeat_file_content_mock,
                                                      Boto3Factory_mock, rlock_mock):
        get_s3_heartbeat_file_content_mock.return_value = "test_json"
        get_heartbeat_s3_info_mock.return_value = ("", "")
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._is_heartbeat_s3_upload_enabled = True
        deepracer_node_monitor._upload_s3_job_status("SUCCESS", "SUCCESS_MSG")
        Boto3Factory_mock.create_boto3_client.assert_not_called()
        Boto3Factory_mock.create_boto3_client(Boto3Client.S3).put_object.assert_not_called()

    @patch("deepracer_node_monitor.deepracer_node_monitor.S3Utils.get_s3_heartbeat_file_content")
    @patch("deepracer_node_monitor.deepracer_node_monitor.S3Utils.get_heartbeat_s3_info")
    def test_upload_s3_job_status_heartbeat_s3_disabled(
            self, get_heartbeat_s3_info_mock,
            get_s3_heartbeat_file_content_mock, Boto3Factory_mock, rlock_mock):
        get_s3_heartbeat_file_content_mock.return_value = "test_json"
        get_heartbeat_s3_info_mock.return_value = ("", "")
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._is_heartbeat_s3_upload_enabled = False
        deepracer_node_monitor._upload_s3_job_status("SUCCESS", "SUCCESS_MSG")
        Boto3Factory_mock.create_boto3_client.assert_not_called()
        Boto3Factory_mock.create_boto3_client(Boto3Client.S3).put_object.assert_not_called()

    @patch("deepracer_node_monitor.deepracer_node_monitor.RoboMakerUtils.get_deepracer_owner_id")
    @patch("deepracer_node_monitor.deepracer_node_monitor.RoboMakerUtils.get_robomaker_simapp_id")
    @patch("deepracer_node_monitor.deepracer_node_monitor.CloudWatchJobStatusMetricDimensionData")
    def test_write_cw_metrics(self, CloudWatchJobStatusMetricDimensionData_mock,
                              get_robomaker_simapp_id_mock, get_deepracer_owner_id_mock,
                              Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._is_heartbeat_cw_publisher_enabled = True
        get_deepracer_owner_id_mock.return_value = "test_owner_id"
        get_robomaker_simapp_id_mock.return_value = "test_simapp_id"
        Boto3Factory_mock.create_boto3_client = MagicMock()
        Boto3Factory_mock.create_boto3_client(Boto3Client.CLOUDWATCH).put_metric_data = MagicMock()
        CloudWatchJobStatusMetricDimensionData_mock().to_cloudwatch_dict.side_effect = [
            {"cw_metric_data_jobstatus": "hi"},
            {"cw_metric_data_jobstatus_owner_simapp_id_mock": "hello"}
        ]
        deepracer_node_monitor._write_cw_metrics("SUCCESS")
        Boto3Factory_mock.create_boto3_client(Boto3Client.CLOUDWATCH).put_metric_data.assert_has_calls([
            call(Namespace=CW_METRIC_NAMESPACE, MetricData=[{"cw_metric_data_jobstatus": "hi"}]),
            call(Namespace=CW_METRIC_NAMESPACE, MetricData=[{"cw_metric_data_jobstatus_owner_simapp_id_mock": "hello"}])
        ])

    @patch("deepracer_node_monitor.deepracer_node_monitor.RoboMakerUtils.get_deepracer_owner_id")
    @patch("deepracer_node_monitor.deepracer_node_monitor.RoboMakerUtils.get_robomaker_simapp_id")
    @patch("deepracer_node_monitor.deepracer_node_monitor.CloudWatchJobStatusMetricDimensionData")
    def test_write_cw_metrics_empty_owner_id(self, CloudWatchJobStatusMetricDimensionData_mock,
                                             get_robomaker_simapp_id_mock, get_deepracer_owner_id_mock,
                                             Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._is_heartbeat_cw_publisher_enabled = True
        get_deepracer_owner_id_mock.return_value = ""
        get_robomaker_simapp_id_mock.return_value = "test_simapp_id"
        CloudWatchJobStatusMetricDimensionData_mock().to_cloudwatch_dict.side_effect = [
            {"cw_metric_data_jobstatus": "hi"},
            {"cw_metric_data_jobstatus_owner_simapp_id_mock": "hello"}
        ]
        Boto3Factory_mock.create_boto3_client = MagicMock()
        Boto3Factory_mock.create_boto3_client(Boto3Client.CLOUDWATCH).put_metric_data = MagicMock()
        deepracer_node_monitor._write_cw_metrics("SUCCESS")
        Boto3Factory_mock.create_boto3_client(Boto3Client.CLOUDWATCH).put_metric_data.assert_has_calls([
            call(Namespace=CW_METRIC_NAMESPACE, MetricData=[{"cw_metric_data_jobstatus": "hi"}])
        ])

    @patch("deepracer_node_monitor.deepracer_node_monitor.RoboMakerUtils.get_deepracer_owner_id")
    @patch("deepracer_node_monitor.deepracer_node_monitor.RoboMakerUtils.get_robomaker_simapp_id")
    @patch("deepracer_node_monitor.deepracer_node_monitor.CloudWatchJobStatusMetricDimensionData")
    def test_write_cw_metrics_empty_simapp_id(self, CloudWatchJobStatusMetricDimensionData_mock,
                                              get_robomaker_simapp_id_mock, get_deepracer_owner_id_mock,
                                              Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._is_heartbeat_cw_publisher_enabled = True
        get_deepracer_owner_id_mock.return_value = "test_owner_id"
        get_robomaker_simapp_id_mock.return_value = ""
        CloudWatchJobStatusMetricDimensionData_mock().to_cloudwatch_dict.side_effect = [
            {"cw_metric_data_jobstatus": "hi"},
            {"cw_metric_data_jobstatus_owner_simapp_id_mock": "hello"}
        ]
        Boto3Factory_mock.create_boto3_client = MagicMock()
        Boto3Factory_mock.create_boto3_client(Boto3Client.CLOUDWATCH).put_metric_data = MagicMock()
        deepracer_node_monitor._write_cw_metrics("SUCCESS")
        Boto3Factory_mock.create_boto3_client(Boto3Client.CLOUDWATCH).put_metric_data.assert_has_calls([
            call(Namespace=CW_METRIC_NAMESPACE, MetricData=[{"cw_metric_data_jobstatus": "hi"}])
        ])

    @patch("deepracer_node_monitor.deepracer_node_monitor.RoboMakerUtils.get_deepracer_owner_id")
    @patch("deepracer_node_monitor.deepracer_node_monitor.RoboMakerUtils.get_robomaker_simapp_id")
    @patch("deepracer_node_monitor.deepracer_node_monitor.CloudWatchJobStatusMetricDimensionData")
    def test_write_cw_metrics_empty_owner_simapp_id(self, CloudWatchJobStatusMetricDimensionData_mock,
                                                    get_robomaker_simapp_id_mock, get_deepracer_owner_id_mock,
                                                    Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._is_heartbeat_cw_publisher_enabled = True
        get_deepracer_owner_id_mock.return_value = ""
        get_robomaker_simapp_id_mock.return_value = ""
        CloudWatchJobStatusMetricDimensionData_mock().to_cloudwatch_dict.side_effect = [
            {"cw_metric_data_jobstatus": "hi"},
            {"cw_metric_data_jobstatus_owner_simapp_id_mock": "hello"}
        ]
        Boto3Factory_mock.create_boto3_client = MagicMock()
        Boto3Factory_mock.create_boto3_client(Boto3Client.CLOUDWATCH).put_metric_data = MagicMock()
        deepracer_node_monitor._write_cw_metrics("SUCCESS")
        Boto3Factory_mock.create_boto3_client(Boto3Client.CLOUDWATCH).put_metric_data.assert_has_calls([
            call(Namespace=CW_METRIC_NAMESPACE, MetricData=[{"cw_metric_data_jobstatus": "hi"}])
        ])

    @patch("deepracer_node_monitor.deepracer_node_monitor.RoboMakerUtils.get_deepracer_owner_id")
    @patch("deepracer_node_monitor.deepracer_node_monitor.RoboMakerUtils.get_robomaker_simapp_id")
    @patch("deepracer_node_monitor.deepracer_node_monitor.CloudWatchJobStatusMetricDimensionData")
    def test_write_cw_metrics_empty_all(self, CloudWatchJobStatusMetricDimensionData_mock,
                                        get_robomaker_simapp_id_mock, get_deepracer_owner_id_mock,
                                        Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._is_heartbeat_cw_publisher_enabled = True
        get_deepracer_owner_id_mock.return_value = ""
        get_robomaker_simapp_id_mock.return_value = ""
        CloudWatchJobStatusMetricDimensionData_mock = MagicMock()
        CloudWatchJobStatusMetricDimensionData_mock().to_cloudwatch_dict.side_effect = [
            {"cw_metric_data_jobstatus": "hi"},
            {"cw_metric_data_jobstatus_owner_simapp_id_mock": "hello"}
        ]
        Boto3Factory_mock.create_boto3_client = MagicMock()
        Boto3Factory_mock.create_boto3_client(Boto3Client.CLOUDWATCH).put_metric_data = MagicMock()
        deepracer_node_monitor._write_cw_metrics("")
        Boto3Factory_mock.create_boto3_client(Boto3Client.CLOUDWATCH).put_metric_data.assert_not_called()

    @patch("deepracer_node_monitor.deepracer_node_monitor.RoboMakerUtils.get_deepracer_owner_id")
    @patch("deepracer_node_monitor.deepracer_node_monitor.RoboMakerUtils.get_robomaker_simapp_id")
    @patch("deepracer_node_monitor.deepracer_node_monitor.CloudWatchJobStatusMetricDimensionData")
    def test_write_cw_metrics_heartbeat_disabled(self, CloudWatchJobStatusMetricDimensionData_mock,
                                                 get_robomaker_simapp_id_mock, get_deepracer_owner_id_mock,
                                                 Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._is_heartbeat_cw_publisher_enabled = False
        get_deepracer_owner_id_mock.return_value = ""
        get_robomaker_simapp_id_mock.return_value = ""
        CloudWatchJobStatusMetricDimensionData_mock = MagicMock()
        CloudWatchJobStatusMetricDimensionData_mock().to_cloudwatch_dict.side_effect = [
            {"cw_metric_data_jobstatus": "hi"},
            {"cw_metric_data_jobstatus_owner_simapp_id_mock": "hello"}
        ]
        Boto3Factory_mock.create_boto3_client = MagicMock()
        Boto3Factory_mock.create_boto3_client(Boto3Client.CLOUDWATCH).put_metric_data = MagicMock()
        deepracer_node_monitor._write_cw_metrics("")
        Boto3Factory_mock.create_boto3_client(Boto3Client.CLOUDWATCH).put_metric_data.assert_not_called()

    @patch("deepracer_node_monitor.deepracer_node_monitor.NodeMonitor")
    def test_on_dead_node_update(self, node_monitor_mock,
                                 Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._upload_s3_job_status = MagicMock()
        deepracer_node_monitor._write_cw_metrics = MagicMock()
        deepracer_node_monitor.on_dead_node_update(node_monitor_mock, set({"dead_node"}))
        deepracer_node_monitor._upload_s3_job_status.assert_called_once_with(
            JobStatus.FAILED, JobStatusMsg.FAILED)
        deepracer_node_monitor._write_cw_metrics.assert_called_once_with(JobStatus.FAILED)

    @patch("deepracer_node_monitor.deepracer_node_monitor.NodeMonitor")
    def test_on_dead_node_update_dead_node_previously_detected(self, node_monitor_mock,
                                                               Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._upload_s3_job_status = MagicMock()
        deepracer_node_monitor._write_cw_metrics = MagicMock()
        deepracer_node_monitor._is_dead_node_detected = True
        deepracer_node_monitor.on_dead_node_update(node_monitor_mock, set({"dead_node"}))
        deepracer_node_monitor._upload_s3_job_status.assert_not_called()
        deepracer_node_monitor._write_cw_metrics.assert_not_called()

    @patch("deepracer_node_monitor.deepracer_node_monitor.NodeMonitor")
    def test_on_running_node_update_all_nodes_running(self, node_monitor_mock,
                                                      Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._upload_s3_job_status = MagicMock()
        deepracer_node_monitor._write_cw_metrics = MagicMock()
        deepracer_node_monitor.on_running_node_update(
            node_monitor_mock,
            set({'/gazebo', '/deepracer', '/ude_ros_server', '*/controller_manager', '/agent0/robot_state_publisher'}))
        deepracer_node_monitor._upload_s3_job_status.assert_called_once_with(
            JobStatus.RUNNING, JobStatusMsg.RUNNING)
        deepracer_node_monitor._write_cw_metrics.assert_called_once_with(JobStatus.RUNNING)

    @patch("deepracer_node_monitor.deepracer_node_monitor.NodeMonitor")
    def test_on_running_node_update_all_nodes_running_with_extra(self, node_monitor_mock,
                                                                 Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._upload_s3_job_status = MagicMock()
        deepracer_node_monitor._write_cw_metrics = MagicMock()
        deepracer_node_monitor.on_running_node_update(
            node_monitor_mock,
            set({'/gazebo', '/deepracer', '/ude_ros_server', '*/controller_manager',
                 '/agent0/robot_state_publisher', '/dummy/extra'}))
        deepracer_node_monitor._upload_s3_job_status.assert_called_once_with(
            JobStatus.RUNNING, JobStatusMsg.RUNNING)
        deepracer_node_monitor._write_cw_metrics.assert_called_once_with(JobStatus.RUNNING)

    @patch("deepracer_node_monitor.deepracer_node_monitor.NodeMonitor")
    def test_on_running_node_update_all_nodes_not_running(self, node_monitor_mock,
                                                          Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._upload_s3_job_status = MagicMock()
        deepracer_node_monitor._write_cw_metrics = MagicMock()
        deepracer_node_monitor.on_running_node_update(node_monitor_mock, set({'/gazebo'}))
        deepracer_node_monitor._upload_s3_job_status.assert_not_called()
        deepracer_node_monitor._write_cw_metrics.assert_not_called()

    @patch("deepracer_node_monitor.deepracer_node_monitor.NodeMonitor")
    def test_on_start(self, node_monitor_mock,
                      Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._upload_s3_job_status = MagicMock()
        deepracer_node_monitor._write_cw_metrics = MagicMock()
        deepracer_node_monitor.on_start(node_monitor_mock)
        deepracer_node_monitor._upload_s3_job_status.assert_called_once_with(
            JobStatus.INITIALIZING, JobStatusMsg.INITIALIZING)
        deepracer_node_monitor._write_cw_metrics.assert_called_once_with(JobStatus.INITIALIZING)

    def test_monitor_nodes_Property(self, Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        self.assertEqual(deepracer_node_monitor.monitor_nodes, self.monitor_nodes)

    @patch("deepracer_node_monitor.deepracer_node_monitor.NodeMonitor")
    def test_on_no_status_change(self, node_monitor_mock,
                                 Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._upload_s3_job_status = MagicMock()
        deepracer_node_monitor._write_cw_metrics = MagicMock()
        deepracer_node_monitor._is_dead_node_detected = False
        deepracer_node_monitor._job_status = JobStatus.RUNNING
        deepracer_node_monitor._job_status_msg = JobStatusMsg.RUNNING
        deepracer_node_monitor.on_no_status_change(node_monitor_mock)
        deepracer_node_monitor._upload_s3_job_status.assert_called_once_with(
            JobStatus.RUNNING, JobStatusMsg.RUNNING)
        deepracer_node_monitor._write_cw_metrics.assert_called_once_with(JobStatus.RUNNING)

    @patch("deepracer_node_monitor.deepracer_node_monitor.NodeMonitor")
    def test_on_no_status_change_dead_node(self, node_monitor_mock,
                                           Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        deepracer_node_monitor._upload_s3_job_status = MagicMock()
        deepracer_node_monitor._write_cw_metrics = MagicMock()
        deepracer_node_monitor._is_dead_node_detected = True
        deepracer_node_monitor._job_status = JobStatus.RUNNING
        deepracer_node_monitor._job_status_msg = JobStatusMsg.RUNNING
        deepracer_node_monitor.on_no_status_change(node_monitor_mock)
        deepracer_node_monitor._upload_s3_job_status.assert_not_called()
        deepracer_node_monitor._write_cw_metrics.assert_not_called()

    @patch("deepracer_node_monitor.deepracer_node_monitor.NodeMonitor")
    def test_check_all_monitor_nodes_running(self, node_monitor_mock,
                                             Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        return_val = deepracer_node_monitor._check_all_monitor_nodes_running(
            running_nodes=['/gazebo', '/deepracer', '/ude_ros_server', 'racecar/controller_manager',
                           '/agent0/robot_state_publisher'])
        self.assertEqual(return_val, True)

    @patch("deepracer_node_monitor.deepracer_node_monitor.NodeMonitor")
    def test_check_all_monitor_nodes_running_false(self, node_monitor_mock,
                                             Boto3Factory_mock, rlock_mock):
        deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=self.monitor_nodes)
        return_val = deepracer_node_monitor._check_all_monitor_nodes_running(
            running_nodes=['/gazebo', '/deepracer', '/ude_ros_server', 'racecar',
                           '/agent0/robot_state_publisher'])
        self.assertEqual(return_val, False)
