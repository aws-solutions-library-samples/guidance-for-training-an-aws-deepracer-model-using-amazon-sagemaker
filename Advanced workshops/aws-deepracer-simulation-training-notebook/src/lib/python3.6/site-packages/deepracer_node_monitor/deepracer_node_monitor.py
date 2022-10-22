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
"""A class for DeepRacerNodeMonitor."""
import fnmatch
import logging

from threading import RLock
from typing import List, Set, Optional

from node_monitor import NodeMonitor
from node_monitor.node_monitor_observer_interface import NodeMonitorObserverInterface
from deepracer_node_monitor.constants import (JobStatus,
                                              JobStatusMsg)
from deepracer_node_monitor.aws_utils.constants import (Boto3Client, CW_METRIC_NAMESPACE)
from deepracer_node_monitor.aws_utils.robomaker_utils import RoboMakerUtils
from deepracer_node_monitor.aws_utils.cloudwatch_utils import CloudWatchJobStatusMetricDimensionData
from deepracer_node_monitor.aws_utils.s3_utils import S3Utils
from deepracer_node_monitor.aws_utils.boto3_factory import Boto3Factory


class DeepRacerNodeMonitor(NodeMonitorObserverInterface):
    """
    DeepRacerNodeMonitor Class.
    """

    def __init__(self, monitor_nodes: Optional[List[str]] = None) -> None:
        """
        Initialize DeepRacerNodeMonitor.

        Args:
            monitor_nodes (Optional[List[str]]): List of nodes to be monitored
        """
        self._monitor_nodes = monitor_nodes

        self._lock = RLock()
        self._is_dead_node_detected = False
        self._job_status = JobStatus.INITIALIZING
        self._job_status_msg = JobStatusMsg.INITIALIZING
        self._is_heartbeat_s3_upload_enabled = S3Utils.get_s3_heartbeat_location_path()
        self._is_heartbeat_cw_publisher_enabled = CloudWatchJobStatusMetricDimensionData.is_cloudwatch_heartbeat_enabled()

    def _check_all_monitor_nodes_running(self, running_nodes: List[str]) -> bool:
        """
        Iterate over each nodes in the monitor list, first checks if running node
        has an exact match. If not iterate over all the running nodes that matches monitor_node pattern
        Returns boolean value if all monitor_node matches the running nodes.

        Args:
            running_nodes (list[str]): Running node name that has to be checked with the monitor node list

        Returns
            bool: Returns true value if all monitor_node matches the running nodes.
        """
        for monitor_node in self._monitor_nodes:
            if monitor_node in running_nodes:
                continue
            is_monitor_node_in_running_nodes = any([fnmatch.fnmatch(node, monitor_node) for node in running_nodes])
            if not is_monitor_node_in_running_nodes:
                return False
        return True

    def _upload_s3_job_status(self, job_status: str, job_status_msg: str) -> None:
        """
        Helper function to upload job status to S3

        Args:
            job_status (str): Job status string is mostly INITIALIZING, RUNNING, FAILED
            job_status_msg (str): Job status message for appropriate job status
        """
        if not self._is_heartbeat_s3_upload_enabled:
            return
        job_status_json = S3Utils.get_s3_heartbeat_file_content(job_status, job_status_msg)
        s3_bucket, s3_key = S3Utils.get_heartbeat_s3_info()
        # Write to S3 only if JOB_STATUS_S3_LOCATION env is passed
        if s3_bucket and s3_key:
            s3_client = Boto3Factory.create_boto3_client(Boto3Client.S3)
            s3_client.put_object(Bucket=s3_bucket, Key=s3_key,
                                 Body=bytes(str(job_status_json), encoding="utf-8"))
            logging.debug("[DeepRacerNodeMonitor]: Successfully uploaded file to s3 for job status: {}".format(job_status))

    def _write_cw_metrics(self, job_status: str) -> None:
        """
        Helper function to write job status to cloudwatch metrics

        Args:
            job_status (str): Job status string is mostly INITIALIZING, RUNNING, FAILED
        """
        if not self._is_heartbeat_cw_publisher_enabled:
            return
        owner_id = RoboMakerUtils.get_deepracer_owner_id()
        simapp_id = RoboMakerUtils.get_robomaker_simapp_id()
        cw_client = Boto3Factory.create_boto3_client(Boto3Client.CLOUDWATCH)
        # Write cloudwatch metrics only if its running inside RoboMaker simulator and OWNER_ID is passed
        if owner_id and simapp_id:
            # Metric with job status, owner_id, simapp_id
            dimenstion_data = CloudWatchJobStatusMetricDimensionData(job_status, owner_id, simapp_id).to_cloudwatch_dict()
            cw_client.put_metric_data(Namespace=CW_METRIC_NAMESPACE, MetricData=[dimenstion_data])
        if job_status:
            # Metric with only job status
            dimenstion_data = CloudWatchJobStatusMetricDimensionData(job_status).to_cloudwatch_dict()
            cw_client.put_metric_data(Namespace=CW_METRIC_NAMESPACE, MetricData=[dimenstion_data])
            logging.debug("[DeepRacerNodeMonitor]: Successfully written cloudwatch metrics for job status: {}".format(job_status))

    def on_dead_node_update(self, node_monitor: NodeMonitor, dead_nodes: Set[str]) -> None:
        """
        Callback function when set of dead nodes are updated. The first time dead node is called
        then alone S3 upload and cloudwatch metrics are written.

        Args:
            node_monitor (NodeMonitor): Instance of NodeMonitor class
            dead_nodes (Set[str]): Set of all the dead nodes
        """
        # To avoid double counting of FAILED metrics
        with self._lock:
            # Updating s3 and cloudwatch metrics only first time a dead node is detected
            if not self._is_dead_node_detected:
                logging.info("[DeepRacerNodeMonitor]: Dead nodes are {}".format(dead_nodes))
                self._job_status = JobStatus.FAILED
                self._job_status_msg = JobStatusMsg.FAILED
                self._is_dead_node_detected = True
                self._upload_s3_job_status(JobStatus.FAILED, JobStatusMsg.FAILED)
                self._write_cw_metrics(JobStatus.FAILED)

    def on_running_node_update(self, node_monitor: NodeMonitor, running_nodes: Set[str]) -> None:
        """
        Callback function when set of running nodes are updated. Updating S3 and cloudwatch
        metrics only when all the monitored nodes are running.

        Args:
            node_monitor (NodeMonitor): Instance of NodeMonitor class
            running_nodes (Set[str]): Set of all the dead nodes
        """
        # To avoid double counting of RUNNING metrics
        with self._lock:
            logging.info("[DeepRacerNodeMonitor]: Running nodes are {}".format(running_nodes))
            # This also takes care of the case where self._monitor_nodes & running nodes are empty
            is_all_monitor_node_running = self._check_all_monitor_nodes_running(running_nodes)
            if is_all_monitor_node_running:
                self._job_status = JobStatus.RUNNING
                self._job_status_msg = JobStatusMsg.RUNNING
                self._upload_s3_job_status(JobStatus.RUNNING, JobStatusMsg.RUNNING)
                self._write_cw_metrics(JobStatus.RUNNING)

    def on_start(self, node_monitor: NodeMonitor) -> None:
        """
        Callback function when node monitoring is started

        Args:
            node_monitor (NodeMonitor): Instance of NodeMonitor class
        """
        # To avoid double counting of INITIALIZING metrics
        with self._lock:
            logging.info("[DeepRacerNodeMonitor]: NodeMonitor started running")
            self._upload_s3_job_status(JobStatus.INITIALIZING, JobStatusMsg.INITIALIZING)
            self._write_cw_metrics(JobStatus.INITIALIZING)

    def on_no_status_change(self, node_monitor: NodeMonitor) -> None:
        """
        Callback function when node monitoring did not see any status change

        Args:
            node_monitor (NodeMonitor): Instance of NodeMonitor class
        """
        # To avoid double counting of metrics
        with self._lock:
            # Updating s3 and cloudwatch metrics only first time a dead node is detected
            if not self._is_dead_node_detected:
                self._upload_s3_job_status(self._job_status, self._job_status_msg)
                self._write_cw_metrics(self._job_status)

    @property
    def monitor_nodes(self) -> List[str]:
        """
        Returns the list of nodes to be monitored

        Returns:
            List[str]: list of nodes to be monitored
        """
        return self._monitor_nodes.copy()
