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
""" Utils for the RoboMaker functions"""
import os


class RoboMakerUtils(object):
    """
    Static class for robomaker utils
    """
    @staticmethod
    def get_robomaker_simulation_arn() -> str:
        """
        Returns the RoboMaker simulation arn set as environment variable when simulation is started

        Returns
            str: Environment variable set for AWS_ROBOMAKER_SIMULATION_JOB_ARN
        """
        return os.environ.get('AWS_ROBOMAKER_SIMULATION_JOB_ARN', '')

    @staticmethod
    def get_robomaker_simapp_id() -> str:
        """
        Parse the simulation id from the RoboMaker ARN

        Returns
            str: Simulation ID for the RoboMaker job
        """
        robomaker_simulation_arn = RoboMakerUtils.get_robomaker_simulation_arn()
        return robomaker_simulation_arn.split("/")[-1]

    @staticmethod
    def get_deepracer_owner_id() -> str:
        """
        Returns the owner id set as environment variable when simulation is started

        Returns
            str: Environment variable set for OWNER_ID
        """
        return os.environ.get('OWNER_ID', '')

    @staticmethod
    def get_aws_region() -> str:
        """
        Returns aws region from ACM_REGION env

        Returns
            str: returns aws_region from ACM_REGION env
        """
        return os.environ.get('ACM_REGION', 'us-east-1')
