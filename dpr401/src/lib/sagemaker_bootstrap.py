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

from __future__ import absolute_import

import json
import logging
import os
import subprocess

LOG = logging.getLogger(__name__)


def get_params():
    """
    Get SM_TRAINING_ENV parameter values set by sagemaker instance.

    Returns:
        dict: SM_TRAINING_ENV parameter values as a dictionary.
    """
    params_blob = os.environ.get('SM_TRAINING_ENV', '')
    params = json.loads(params_blob)
    return params


def get_user_args_dict():
    """
    Get sagemaker user arguments.

    Returns:
        dict: a dictionary of user arguments passed into sagemake instance.
    """
    params = get_params()
    user_args = os.environ.get('SM_USER_ARGS', '')
    LOG.info("SM_USER_ARGS=%s", user_args)
    LOG.info("All eniron vars=%s", os.environ)
    # add job_name for logging
    user_args_dict = convert_to_dict(json.loads(user_args))
    LOG.info("user_args_dict=%s", user_args_dict)
    user_args_dict["--job_name"] = params['job_name']
    return user_args_dict


def convert_to_dict(input_list):
    """
    Convert list to dictionary.

    Args:
        input_list (list): input list to convert to dictionary.
    Returns:
        dict: the list of arguments converted to dictionary.
    """
    ret_dict = {input_list[i]: input_list[i + 1] for i in range(0, len(input_list), 2)}
    return ret_dict


def concat_dict_to_string(input_dict):
    """
    Join the arguments in input dictionary as string so that we can pass them
    as arguments to shell scripts and python scripts.

    Args:
        input_dict (dict): a dictionary arguments to concatenate as string

    Returns:
        ret_str: a string formed by joining dictionary by empty spaces
    """
    ret_str = ' '.join(key + ' ' + str(val) for key, val in input_dict.items())
    return ret_str


def get_env_variables(user_args_dict, is_sageonly=False):
    """
    Get environment variables for passing into subprocess
    based on user argument dictionary

    Args:
        user_args_dict (dict): a dictionary of user arguments passed into sagemake instance.

    Raises:
        ValueError: if the passed launch file is not supported by the image.

    Returns:
        dict: the dictionary of environment variables to pass into subprocess shell.
    """
    env = os.environ.copy()
    env["SAGEMAKER_SHARED_S3_BUCKET"] = user_args_dict["--s3_bucket"]
    env["SAGEMAKER_SHARED_S3_PREFIX"] = user_args_dict["--s3_prefix"]
    env["APP_REGION"] = user_args_dict["--aws_region"]
    env["MODEL_METADATA_FILE_S3_KEY"] = user_args_dict["--model_metadata_s3_key"]
    if is_sageonly:
        env["S3_YAML_NAME"] = user_args_dict["--s3_yaml_name"]
        env["KINESIS_VIDEO_STREAM_NAME"] = user_args_dict["--kinesis_stream_name"]
        env["WORLD_NAME"] = user_args_dict["--world_name"]
        env["S3_ROS_LOG_BUCKET"] = user_args_dict["--s3_ros_log_bucket"]
        env["JOB_NAME"] = user_args_dict["--job_name"]
        env["SIMULATION_LAUNCH_FILE"] = user_args_dict["--simulation_launch_file"]
        if env["SIMULATION_LAUNCH_FILE"] == "distributed_training.launch":
            env["JOB_TYPE"] = "training"
        elif env["SIMULATION_LAUNCH_FILE"] == "evaluation.launch":
            env["JOB_TYPE"] = "evaluation"
            env["MODEL_S3_BUCKET"] = user_args_dict["--s3_bucket"]
            env["MODEL_S3_PREFIX"] = user_args_dict["--s3_prefix"]
        else:
            raise ValueError("Launch file {} not supported for SageOnly Job".format(env["SIMULATION_LAUNCH_FILE"]))
    return env


def run_command(cmd, env_args):
    """ Calling the bash script with appropriate arguments.
    Args:
        cmd (str): script along with argument
        env_args (dict): a dictionary of environment variables to set in the shell that runs the command.
    Raises:
        RuntimeError: Failed job exception is bubbled to sagemaker training.
    """
    LOG.info("Launching training command: %s", cmd)
    process = subprocess.Popen([cmd], env=env_args, shell=True)
    process.communicate()  # wait until the subprocess finishes
    retval = process.returncode
    if retval != 0:
        msg = "Train command returned exit code %s" % retval
        LOG.error(msg)
        raise RuntimeError(msg)


def train():
    """
    Runs the configured sage-train command
    with all the hyperparameters for RoboMaker + SageMaker job
    TODO: remove this when we fully deprecate the RoboMaker + SageMaker jobs.
    """
    os.chdir("/opt/ml/code")

    user_args_dict = get_user_args_dict()
    hyperparams = concat_dict_to_string(user_args_dict)
    env_args = get_env_variables(user_args_dict)

    base_cmd = "/opt/ml/code/sage-train.sh"
    cmd = "%s %s" % (base_cmd, hyperparams)
    run_command(cmd, env_args)


def sageonly():
    """
    Runs the configured SAGEMAKER_TRAINING_COMMAND with all
    the hyperparameters for sagemaker only job.
    """
    os.chdir("/opt/ml/code")

    user_args_dict = get_user_args_dict()
    hyperparams = concat_dict_to_string(user_args_dict)
    env_args = get_env_variables(user_args_dict, is_sageonly=True)

    LOG.info("Arguments to sageonly: %s", hyperparams)

    if env_args["SIMULATION_LAUNCH_FILE"] == "distributed_training.launch":
        base_cmd = "/opt/ml/code/sageonly.sh"
    elif env_args["SIMULATION_LAUNCH_FILE"] == "evaluation.launch":
        base_cmd = "/opt/ml/code/sageonly_evals.sh"
    cmd = "%s %s" % (base_cmd, hyperparams)
    run_command(cmd, env_args)
