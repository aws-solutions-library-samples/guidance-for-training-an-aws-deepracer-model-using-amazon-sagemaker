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
""" Utils for the S3 functions"""
import os
import json
import time

from typing import Tuple


class S3Utils(object):
    """
    Static class for s3 utils
    """
    @staticmethod
    def get_s3_heartbeat_location_path() -> str:
        """
        Returns  s3 job location path set as environment variable when simulation is started

        Returns
            str: Environment variable set for HEARTBEAT_S3_LOCATION
        """
        return os.environ.get('HEARTBEAT_S3_LOCATION', '')

    @staticmethod
    def get_heartbeat_s3_info() -> Tuple[str, str]:
        """
        Split the S3 url for job status location having bucket and key

        Returns
            Tuple[str, str]: s3 bucket and s3 key as tuple
        """
        s3_job_location_path = S3Utils.get_s3_heartbeat_location_path()
        path_parts = s3_job_location_path.replace("s3://", "").split("/")
        bucket = path_parts.pop(0)
        key = "/".join(path_parts)
        return bucket, key

    @staticmethod
    def get_s3_heartbeat_file_content(job_status: str, message: str) -> str:
        """
        Returns s3 file content for job status file

        Returns
            str: Containing job status and message
        """
        return json.dumps({
            "jobStatus": job_status,
            "message": message,
            "epoch": time.time()
        })
