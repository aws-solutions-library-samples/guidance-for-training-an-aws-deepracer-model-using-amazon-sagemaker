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


class JobStatus(object):
    """
    JobStatus type
    """
    INITIALIZING = "INITIALIZING"
    RUNNING = "RUNNING"
    FAILED = "FAILED"


class JobStatusMsg(object):
    """
    JobStatus Message
    """
    INITIALIZING = "ROS nodes are initializing"
    RUNNING = "All ROS nodes started running"
    FAILED = "ROS node has failed"


class JobStatusMetricDimension(object):
    """
    JobStatus metric dimensions
    """
    JOB_STATUS = "JobStatus"
    OWNER_ID = "OwnerId"
    SIM_APP_ID = "SimAppId"
