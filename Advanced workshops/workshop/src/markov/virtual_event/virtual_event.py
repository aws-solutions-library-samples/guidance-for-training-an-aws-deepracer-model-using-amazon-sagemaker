import json
import logging
import os
import time
import rospy

from markov import utils
from markov.agents.utils import RunPhaseSubject
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.deepracer_exceptions import GenericNonFatalException
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_VIRTUAL_EVENT_RACE_EXCEPTION)
from markov.rollout_utils import (PhaseObserver,
                                  configure_environment_randomizer,
                                  signal_robomaker_markov_package_ready)
from markov.boto.s3.constants import (SECTOR_TIME_LOCAL_PATH,
                                      SECTOR_TIME_S3_POSTFIX,
                                      SECTOR_X_FORMAT)
from markov.boto.s3.files.virtual_event_best_sector_time import VirtualEventBestSectorTime
from markov.boto.s3.utils import get_s3_key
from markov.track_geom.track_data import TrackData
from markov.track_geom.utils import (get_hide_positions,
                                     get_start_positions)
from markov.track_geom.constants import START_POS_OFFSET, MIN_START_POS_OFFSET, MAX_START_POS_OFFSET
from markov.gazebo_utils.model_updater import ModelUpdater
from markov.virtual_event.virtual_event_agent_data import VirtualEventAgentData
from markov.virtual_event.virtual_event_agent_model import VirtualEventAgentModel
from markov.virtual_event.cameras.virtual_event_agent_camera_model import VirtualEventAgentCameraModel
from markov.virtual_event.cameras.virtual_event_top_camera_model import VirtualEventTopCameraModel
from markov.virtual_event.virtual_event_simtrace_video import VirtualEventSimtraceVideo
from markov.virtual_event.virtual_event_race_data import VirtualEventRaceData
from markov.virtual_event.virtual_event_graph_manager import VirtualEventGraphManager
from markov.virtual_event.virtual_event_eval_metric import VirtualEventEvalMetric
from markov.gazebo_tracker.trackers.get_model_state_tracker import GetModelStateTracker

LOG = Logger(__name__, logging.INFO).get_logger()


class VirtualEvent():
    """
    VirtualEvent class
    """
    def __init__(self):
        """
        VirtualEvent constructor
        """
        self._track_data = TrackData.get_instance()
        self._is_event_end = False
        self._run_phase_subject = RunPhaseSubject()
        self._model_updater = ModelUpdater.get_instance()

        self._virtual_event_race_data = VirtualEventRaceData()

        self._virtual_event_agent_data = VirtualEventAgentData()

        self._virtual_event_graph_manager = None

        self._virtual_event_agent_models = list()

        self._virtual_event_simtrace_videos = list()

        self._virtual_event_eval_metrics = \
            [VirtualEventEvalMetric(
                agent_name=agent_name,
                race_data=self._virtual_event_race_data)
             for agent_name in self._virtual_event_agent_data.agent_names]

        start_pos_offset = max(min(float(rospy.get_param("START_POS_OFFSET", START_POS_OFFSET)), MAX_START_POS_OFFSET),
                               MIN_START_POS_OFFSET)

        self._virtual_event_agent_camera_models = \
            [VirtualEventAgentCameraModel(
                camera_namespace=racecar_name,
                start_pose=self._track_data.get_racecar_start_pose(
                    racecar_idx=camera_idx,
                    racer_num=self._virtual_event_agent_data.num_of_agents,
                    start_position=get_start_positions(self._virtual_event_agent_data.num_of_agents,
                                                       start_pos_offset)[camera_idx]))
                for camera_idx, racecar_name in enumerate(self._virtual_event_agent_data.racecar_names)]

        [camera_model.spawn() for camera_model in self._virtual_event_agent_camera_models]

        self._virtual_event_top_camera_model = VirtualEventTopCameraModel()
        self._virtual_event_top_camera_model.spawn()

        self._persist_initial_sector_time()

        # ROS service to indicate all the RoboMaker Markov packages are ready for consumption
        signal_robomaker_markov_package_ready()

        PhaseObserver('/agent/training_phase', self._run_phase_subject)

    @property
    def is_event_end(self):
        """
        Returns:
            bool: True if event end, False otherwise
        """
        return self._is_event_end

    def poll(self):
        """
        Poll from sqs the next racer information.
        """
        LOG.info("[VirtualEventAgentData] polling from sqs...")
        self._virtual_event_agent_data.poll()
        LOG.info("[VirtualEventAgentData] polling completed")

    def setup(self):
        """
        Setup up current race

        Returns:
            bool: True if setup race is successful.
                  False is a non fatal exception occurred.
        """

        try:
            LOG.info("[VirtualEvent] setup starting...")
            self._model_updater.unpause_physics()
            LOG.info("[VirtualEvent] unpause physics")

            if self._virtual_event_agent_models:
                [GetModelStateTracker.get_instance().remove(racecar_name)
                    for racecar_name in self._virtual_event_agent_data.racecar_names]
                [agent_model.delete() for agent_model in self._virtual_event_agent_models]

            self._virtual_event_agent_data.download()

            self._virtual_event_agent_models = \
                [VirtualEventAgentModel(
                    profile=profile,
                    hide_position=get_hide_positions(
                        self._virtual_event_agent_data.num_of_agents)[profile.index])
                 for profile in self._virtual_event_agent_data.racer_profiles]

            [camera_model.reset_pose() for camera_model in self._virtual_event_agent_camera_models]

            [agent_model.spawn() for agent_model in self._virtual_event_agent_models]

            self._virtual_event_simtrace_videos = \
                [VirtualEventSimtraceVideo(
                    profile,
                    self._virtual_event_race_data.region)
                 for profile in self._virtual_event_agent_data.racer_profiles]

            [eval_metric.reset(
                profile=profile,
                is_saving_simtrace=simtrace_video.is_saving_simtrace)
             for eval_metric, profile, simtrace_video in zip(
                self._virtual_event_eval_metrics,
                self._virtual_event_agent_data.racer_profiles,
                self._virtual_event_simtrace_videos)]

            self._virtual_event_graph_manager = VirtualEventGraphManager(
                racer_profiles=self._virtual_event_agent_data.racer_profiles,
                race_data=self._virtual_event_race_data,
                eval_metrics=[eval_metric.eval_metric
                              for eval_metric in self._virtual_event_eval_metrics],
                run_phase_subject=self._run_phase_subject,
                virtual_event_agent_camera_models=self._virtual_event_agent_camera_models)

            LOG.info("[VirtualEvent] setup completed.")
            return True
        except GenericNonFatalException as ex:
            ex.log_except_and_continue()
            self._virtual_event_agent_data.persist(
                status_code=ex.error_code,
                error_name=ex.error_name,
                error_details=ex.error_msg)
            self._virtual_event_agent_data.clear()
            return False
        except Exception as ex:
            log_and_exit("[VirtualEvent] fatal error during setup: {}".format(ex),
                         SIMAPP_VIRTUAL_EVENT_RACE_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)

    def start(self):
        """
        Start current race
        """
        LOG.info("[VirtualEvent] race starting...")
        [simtrace_video.setup() for simtrace_video in self._virtual_event_simtrace_videos]
        configure_environment_randomizer()
        self._model_updater.unpause_physics()
        LOG.info("[VirtualEvent] unpause physics")
        self._virtual_event_graph_manager.evaluate()

    def finish(self):
        """
        Finish current race
        """
        LOG.info("[VirtualEvent] race finishing...")
        time.sleep(1)
        self._model_updater.pause_physics()
        LOG.info("[VirtualEvent] pause physics")
        [agent_camera_model.detach() for agent_camera_model in self._virtual_event_agent_camera_models]
        [simtrace_video.persist() for simtrace_video in self._virtual_event_simtrace_videos]
        self._virtual_event_agent_data.persist(status_code=200)
        self._virtual_event_agent_data.clear()
        LOG.info("[VirtualEvent] race finished.")

    def _persist_initial_sector_time(self):
        """
        Persist initial sector time into s3 bucket for entire race

        upload a default best sector time for all sectors with time inf for each sector
        if there is not best sector time existed in s3

        use the s3 bucket and prefix for yaml file stored as environment variable because
        here is SimApp use only. For virtual event there is no s3 bucket and prefix past
        through yaml file. All are past through sqs. For simplicity, reuse the yaml s3 bucket
        and prefix environment variable.
        """
        virtual_event_best_sector_time = VirtualEventBestSectorTime(
            bucket=os.environ.get("YAML_S3_BUCKET", ''),
            s3_key=get_s3_key(os.environ.get("YAML_S3_PREFIX", ''), SECTOR_TIME_S3_POSTFIX),
            region_name=os.environ.get("APP_REGION", "us-east-1"),
            local_path=SECTOR_TIME_LOCAL_PATH)
        response = virtual_event_best_sector_time.list()
        # this is used to handle situation such as robomaker job crash, so the next robomaker job
        # can catch the best sector time left over from crashed job
        if "Contents" not in response:
            virtual_event_best_sector_time.persist(body=json.dumps(
                {SECTOR_X_FORMAT.format(idx + 1): float("inf")
                 for idx in range(int(rospy.get_param("NUM_SECTORS", "3")))}),
                s3_kms_extra_args=utils.get_s3_kms_extra_args())
