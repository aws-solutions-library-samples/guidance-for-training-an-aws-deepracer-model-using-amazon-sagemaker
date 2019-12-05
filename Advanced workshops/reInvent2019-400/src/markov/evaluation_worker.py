import logging
import argparse
import os
import sys
import json
from collections import Counter

import markov.environments
from markov.s3_client import SageS3Client
from markov.s3_boto_data_store import S3BotoDataStore, S3BotoDataStoreParameters
from markov.utils import load_model_metadata
from markov.utils import Logger
from rl_coach.base_parameters import TaskParameters
from rl_coach.core_types import EnvironmentSteps, EnvironmentEpisodes
from rl_coach.utils import short_dynamic_import
from rl_coach.data_stores.data_store import DataStoreParameters, SyncFiles

from google.protobuf import text_format
from tensorflow.python.training.checkpoint_state_pb2 import CheckpointState

from gym.envs.registration import register
import markov.defaults as defaults

CUSTOM_FILES_PATH = "./custom_files"
if not os.path.exists(CUSTOM_FILES_PATH):
    os.makedirs(CUSTOM_FILES_PATH)

logger = Logger(__name__, logging.INFO).get_logger()


def download_custom_files_if_present(s3_client, s3_prefix):
    environment_file_s3_key = os.path.normpath(s3_prefix + "/environments/deepracer_racetrack_env.py")
    environment_local_path = os.path.join(CUSTOM_FILES_PATH, "deepracer_racetrack_env.py")
    success_environment_download = s3_client.download_file(s3_key=environment_file_s3_key,
                                                           local_path=environment_local_path)

    preset_file_s3_key = os.path.normpath(s3_prefix + "/presets/preset.py")
    preset_local_path = os.path.join(CUSTOM_FILES_PATH, "preset.py")
    success_preset_download = s3_client.download_file(s3_key=preset_file_s3_key,
                                                      local_path=preset_local_path)
    return success_preset_download, success_environment_download


def get_latest_checkpoint(checkpoint_dir):
    if os.path.exists(os.path.join(checkpoint_dir, 'checkpoint')):
        ckpt = CheckpointState()
        contents = open(os.path.join(checkpoint_dir, 'checkpoint'), 'r').read()
        text_format.Merge(contents, ckpt)
        # rel_path = os.path.relpath(ckpt.model_checkpoint_path, checkpoint_dir)
        rel_path = ckpt.model_checkpoint_path
        return int(rel_path.split('_Step')[0])


def should_stop(checkpoint_dir):
    if os.path.exists(os.path.join(checkpoint_dir, SyncFiles.FINISHED.value)):
        logger.info("Received termination signal from trainer. Goodbye.")
        return True
    return False


def evaluation_worker(graph_manager, number_of_trials, local_model_directory):
    # initialize graph
    task_parameters = TaskParameters(evaluate_only=True)
    task_parameters.__dict__['checkpoint_restore_dir'] = local_model_directory
    graph_manager.create_graph(task_parameters)
    graph_manager.reset_internal_state()

    data_store = graph_manager.data_store

    episodes_counter = Counter()

    try:
        # This will only work for DeepRacerRacetrackEnv enviroments
        graph_manager.top_level_manager.environment.env.env.set_allow_servo_step_signals(True)
    except Exception as ex:
        print("[ERROR] Method not defined in enviroment class: {}".format(ex))


    while True:
        # Get current checkpoint number
        current_checkpoint = get_latest_checkpoint(local_model_directory)

        # Register the checkpoint with the environment for logging
        graph_manager.top_level_manager.environment.env.env.set_checkpoint_num(current_checkpoint)

        while episodes_counter[current_checkpoint] < 15:
            graph_manager.evaluate(EnvironmentEpisodes(1))
            episodes_counter[current_checkpoint] += 1

        latest_checkpoint = data_store.get_latest_checkpoint()
        if latest_checkpoint:
            if latest_checkpoint > current_checkpoint:
                data_store.get_a_particular_model(checkpoint_number=current_checkpoint+1)
                graph_manager.restore_checkpoint()
    
        if should_stop(local_model_directory):
            break

    # Close the down the job
    graph_manager.top_level_manager.environment.env.env.cancel_simulation_job()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--preset',
                        help="(string) Name of a preset to run (class name from the 'presets' directory.)",
                        type=str,
                        required=False)
    parser.add_argument('--s3_bucket',
                        help='(string) S3 bucket',
                        type=str,
                        default=os.environ.get("MODEL_S3_BUCKET", "gsaur-test"))
    parser.add_argument('--s3_prefix',
                        help='(string) S3 prefix',
                        type=str,
                        default=os.environ.get("MODEL_S3_PREFIX", "sagemaker"))
    parser.add_argument('--aws_region',
                        help='(string) AWS region',
                        type=str,
                        default=os.environ.get("APP_REGION", "us-east-1"))
    parser.add_argument('--number_of_trials',
                        help='(integer) Number of trials',
                        type=int,
                        default=os.environ.get("NUMBER_OF_TRIALS", 15))
    parser.add_argument('-c', '--local_model_directory',
                        help='(string) Path to a folder containing a checkpoint to restore the model from.',
                        type=str,
                        default='./checkpoint')
    parser.add_argument('--model_metadata_s3_key',
                        help='(string) Model Metadata File S3 Key',
                        type=str,
                        default=os.environ.get("MODEL_METADATA_FILE_S3_KEY", None))

    args = parser.parse_args()

    s3_client = SageS3Client(bucket=args.s3_bucket, s3_prefix=args.s3_prefix, aws_region=args.aws_region)

    register(id=defaults.ENV_ID, entry_point=defaults.ENTRY_POINT,
             max_episode_steps=defaults.MAX_STEPS, reward_threshold=defaults.THRESHOLD)

    # Load the model metadata
    model_metadata_local_path = os.path.join(CUSTOM_FILES_PATH, 'model_metadata.json')
    load_model_metadata(s3_client, args.model_metadata_s3_key, model_metadata_local_path)

    # Download the model
    # s3_client.download_model(args.local_model_directory)

    preset_file_success, _ = download_custom_files_if_present(s3_client, args.s3_prefix)

    if preset_file_success:
        preset_location = os.path.join(CUSTOM_FILES_PATH, "preset.py")
        preset_location += ":graph_manager"
        graph_manager = short_dynamic_import(preset_location, ignore_module_case=True)
        logger.info("Using custom preset file!")
    else:
        logger.info("Preset file could not be downloaded. Exiting!")
        sys.exit(1)

    ds_params_instance = S3BotoDataStoreParameters(bucket_name=args.s3_bucket,
                                                   checkpoint_dir=args.local_model_directory,
                                                   aws_region=args.aws_region,
                                                   s3_folder=args.s3_prefix)

    data_store = S3BotoDataStore(ds_params_instance)

    data_store.get_a_particular_model(checkpoint_number=1)

    graph_manager.data_store = data_store

    graph_manager.env_params.seed = 0

    evaluation_worker(
        graph_manager=graph_manager,
        number_of_trials=args.number_of_trials,
        local_model_directory=args.local_model_directory
    )


if __name__ == '__main__':
    main()
