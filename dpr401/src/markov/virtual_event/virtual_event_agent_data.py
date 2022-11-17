import os
import io
import json
import logging
import rospy
import botocore
import tensorflow as tf

from markov import utils
from markov.auth.refreshed_session import refreshed_session
from markov.log_handler.logger import Logger
from markov.boto.sqs.sqs_client import SQSClient
from markov.boto.s3.s3_client import S3Client
from markov.boto.s3.utils import get_s3_key
from markov.boto.s3.constants import (S3_RACE_STATUS_FILE_NAME,
                                      MODEL_METADATA_LOCAL_PATH_FORMAT,
                                      MODEL_METADATA_S3_POSTFIX,
                                      ModelMetadataKeys,
                                      RaceStatusKeys)
from markov.boto.s3.files.model_metadata import ModelMetadata
from markov.boto.s3.files.checkpoint import Checkpoint
from markov.constants import SIMAPP_VERSION_2
from markov.log_handler.deepracer_exceptions import GenericNonFatalException
from markov.log_handler.constants import (SIMAPP_EVENT_SYSTEM_ERROR,
                                          SIMAPP_EVENT_USER_ERROR,
                                          SIMAPP_EVENT_ERROR_CODE_400,
                                          SIMAPP_EVENT_ERROR_CODE_500)
from markov.virtual_event.constants import (MAX_NUM_OF_SQS_MESSAGE,
                                            SQS_WAIT_TIME_SEC,
                                            LOCAL_MODEL_DIR)
from markov.virtual_event.utils import (validate_json_input,
                                        Struct,
                                        is_str_in_list_format)
from markov.virtual_event.virtual_event_json_schema import (
    SINGLE_RACER_INFO_JSON_SCHEMA,
    LIST_OF_RACERS_INFO_JSON_SCHEMA)

LOG = Logger(__name__, logging.INFO).get_logger()


class VirtualEventAgentData():
    """
    VirtualEventAgentData class
    """
    def __init__(self):
        """
        VirtualEventAgentData constructor
        """
        self._kvs_webrtc_names = rospy.get_param("KINESIS_WEBRTC_SIGNALING_CHANNEL_NAME")
        self._queue_url = str(rospy.get_param("SQS_QUEUE_URL", "sqs_queue_url"))
        self._region = rospy.get_param("AWS_REGION", "us-east-1")
        self._s3_client = S3Client(region_name=self._region)
        self._num_of_agents = 1
        if isinstance(self._kvs_webrtc_names, list):
            self._num_of_agents = len(self._kvs_webrtc_names)
        self._agent_names = ["agent"] if self._num_of_agents == 1 \
            else ["agent_{}".format(str(idx)) for idx in range(self._num_of_agents)]
        self._racecar_names = \
            [agent_name.replace("agent", "racecar") for agent_name in self._agent_names]
        self._sqs_client = SQSClient(queue_url=self._queue_url,
                                     region_name=self._region,
                                     max_num_of_msg=MAX_NUM_OF_SQS_MESSAGE,
                                     wait_time_sec=SQS_WAIT_TIME_SEC,
                                     session=refreshed_session(self._region))
        self._racer_profiles = list()

    @property
    def num_of_agents(self):
        """
        Returns:
            int: number of agents
        """
        return self._num_of_agents

    @property
    def racecar_names(self):
        """
        Returns:
            list: list of racer car names
        """
        return self._racecar_names

    @property
    def agent_names(self):
        """
        Returns:
            list: list of agent names
        """
        return self._agent_names

    @property
    def racer_profiles(self):
        """
        Returns:
            list: list of racer profiles object instances
        """
        return self._racer_profiles

    def poll(self):
        """
        Poll from sqs for next racers
        """
        while True:
            messages = self._sqs_client.get_messages()
            if messages:
                message = messages[0].strip()
                try:
                    # TODO: this is temp solution only. After cloud service backend completely migrate to
                    # LIST_OF_RACERS_INFO_JSON_SCHEMA contract, we can remove SINGLE_RACER_INFO_JSON_SCHEMA
                    # logic and if else will not be needed anymore. Therefore, is_str_in_list_format is a
                    # temp solution
                    if is_str_in_list_format(message):
                        validate_json_input(message, LIST_OF_RACERS_INFO_JSON_SCHEMA)
                        self._racer_profiles = [Struct(m) for m in json.loads(message)]
                    else:
                        validate_json_input(message, SINGLE_RACER_INFO_JSON_SCHEMA)
                        self._racer_profiles = [Struct(json.loads(message))]
                    idx = 0
                    for agent_name, racecar_name, profile \
                            in zip(self._agent_names, self._racecar_names, self._racer_profiles):
                        setattr(profile, "agent_name", agent_name)
                        setattr(profile, "racecar_name", racecar_name)
                        setattr(profile, "index", idx)
                        idx += 1
                    break
                except GenericNonFatalException as ex:
                    ex.log_except_and_continue()

    def download(self):
        """
        Download from s3 bucket
        """
        self._download_model_metadata()
        self._download_checkpoint()

    def persist(self,
                status_code,
                error_name=None,
                error_details=None):
        """
        Upload race status into s3

        Args:
            status_code (str): Status code for race.
            error_name (Optional[str]): The name of the error if is 4xx or 5xx.
                                        Defaults to "".
            error_details (Optional[str]): The detail message of the error
                                           if is 4xx or 5xx.
                                           Defaults to "".
        """
        self._persist_race_status(status_code, error_name, error_details)

    def clear(self):
        """
        Clear virtual event data
        """
        self._racer_profiles = list()
        for root, _, files in os.walk(LOCAL_MODEL_DIR):
            for f in files:
                os.remove(os.path.join(root, f))
        tf.reset_default_graph()

    def _download_model_metadata(self):
        """
        Download model metadata and set attribute for racer profiles class instances

        Raises:
            GenericNonFatalException: An non fatal exception which we will
                                      catch and proceed with work loop.
        """
        for profile in self._racer_profiles:
            model_metadata_s3_key = get_s3_key(profile.inputModel.s3KeyPrefix,
                                               MODEL_METADATA_S3_POSTFIX)
            try:
                model_metadata = ModelMetadata(bucket=profile.inputModel.s3BucketName,
                                               s3_key=model_metadata_s3_key,
                                               region_name=self._region,
                                               local_path=MODEL_METADATA_LOCAL_PATH_FORMAT.format(profile.agent_name))
                setattr(profile, "model_metadata", model_metadata)
                model_metadata_info = model_metadata.get_model_metadata_info()
                setattr(profile, "sensors", model_metadata_info[ModelMetadataKeys.SENSOR.value])
                setattr(profile, "simapp_version", model_metadata_info[ModelMetadataKeys.VERSION.value])
            except botocore.exceptions.ClientError as err:
                error_msg = "[s3] Client Error: Failed to download model_metadata file: \
                            s3_bucket: {}, s3_key: {}, {}.".format(profile.inputModel.s3BucketName,
                                                                   model_metadata_s3_key,
                                                                   err)
                raise GenericNonFatalException(error_msg=error_msg,
                                               error_code=SIMAPP_EVENT_ERROR_CODE_400,
                                               error_name=SIMAPP_EVENT_USER_ERROR)
            except Exception as err:
                error_msg = "[s3] System Error: Failed to download model_metadata file: \
                            s3_bucket: {}, s3_key: {}, {}.".format(profile.inputModel.s3BucketName,
                                                                   model_metadata_s3_key,
                                                                   err)
                raise GenericNonFatalException(error_msg=error_msg,
                                               error_code=SIMAPP_EVENT_ERROR_CODE_500,
                                               error_name=SIMAPP_EVENT_SYSTEM_ERROR)

    def _download_checkpoint(self):
        """
        Download Checkpoint object and set attribute for racer profiles class instance
        """
        for profile in self._racer_profiles:
            # download checkpoint from s3
            checkpoint = Checkpoint(bucket=profile.inputModel.s3BucketName,
                                    s3_prefix=profile.inputModel.s3KeyPrefix,
                                    region_name=self._region,
                                    agent_name=profile.agent_name,
                                    checkpoint_dir=LOCAL_MODEL_DIR)
            # make coach checkpoint compatible
            if profile.simapp_version < SIMAPP_VERSION_2 and not checkpoint.rl_coach_checkpoint.is_compatible():
                checkpoint.rl_coach_checkpoint.make_compatible(checkpoint.syncfile_ready)
            # get best model checkpoint string
            model_checkpoint_name = checkpoint.deepracer_checkpoint_json.get_deepracer_best_checkpoint()
            # Select the best checkpoint model by uploading rl coach .coach_checkpoint file
            model_kms = profile.inputModel.s3KmsKeyArn if hasattr(profile.inputModel, 's3KmsKeyArn') else None
            checkpoint.rl_coach_checkpoint.update(
                model_checkpoint_name=model_checkpoint_name,
                s3_kms_extra_args=utils.get_s3_extra_args(model_kms))
            setattr(profile, "checkpoint", checkpoint)

    def _persist_race_status(self, status_code, error_name=None, error_details=None):
        """
        Upload race status into s3

        Args:
            status_code (str): Status code for race.
            error_name (Optional[str]): The name of the error if is 4xx or 5xx.
                                        Defaults to "".
            error_details (Optional[str]): The detail message of the error
                                           if is 4xx or 5xx.
                                           Defaults to "".
        """
        for profile in self._racer_profiles:
            if error_name is not None and error_details is not None:
                status = {RaceStatusKeys.STATUS_CODE.value: status_code,
                          RaceStatusKeys.ERROR_NAME.value: error_name,
                          RaceStatusKeys.ERROR_DETAILS.value: error_details}
            else:
                status = {RaceStatusKeys.STATUS_CODE.value: status_code}
            status_json = json.dumps(status)
            s3_key = os.path.normpath(os.path.join(profile.outputStatus.s3KeyPrefix,
                                                   S3_RACE_STATUS_FILE_NAME))
            race_status_kms = profile.outputStatus.s3KmsKeyArn if \
                hasattr(profile.outputStatus, 's3KmsKeyArn') else None
            self._s3_client.upload_fileobj(bucket=profile.outputStatus.s3BucketName,
                                           s3_key=s3_key,
                                           fileobj=io.BytesIO(status_json.encode()),
                                           s3_kms_extra_args=utils.get_s3_extra_args(race_status_kms))
            LOG.info("[VirtualEventAgentData] Successfully uploaded race status file to \
                     s3 bucket {} with s3 key {}.".format(profile.outputStatus.s3BucketName,
                                                          s3_key))
