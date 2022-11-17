import rospy

from rl_coach.base_parameters import TaskParameters
from rl_coach.core_types import EnvironmentSteps
from markov import utils
from markov.agent_ctrl.constants import ConfigParams
from markov.agents.rollout_agent_factory import (create_rollout_agent,
                                                 create_obstacles_agent,
                                                 create_bot_cars_agent)
from markov.defaults import reward_function
from markov.environments.constants import (LINK_NAMES, STEERING_TOPICS,
                                           VELOCITY_TOPICS)
from markov.gazebo_utils.model_updater import ModelUpdater
from markov.s3_boto_data_store import S3BotoDataStore, S3BotoDataStoreParameters
from markov.sagemaker_graph_manager import get_graph_manager
from markov.track_geom.utils import get_start_positions
from markov.track_geom.constants import (START_POS_OFFSET,
                                         MIN_START_POS_OFFSET,
                                         MAX_START_POS_OFFSET)
from markov.virtual_event.constants import (PAUSE_TIME_BEFORE_START,
                                            LOCAL_MODEL_DIR)
from std_srvs.srv import EmptyRequest


class VirtualEventGraphManager():
    """
    VirtualEventGraphManager class
    """
    def __init__(self,
                 racer_profiles,
                 race_data,
                 eval_metrics,
                 run_phase_subject,
                 virtual_event_agent_camera_models):
        """
        VirtualEventGraphManager constructor

        Args:
            racer_profiles (list): list of racer profile object
            race_data (VirtualEventRaceData): VirtualEventRaceData class instance
            eval_metrics (list): list of EvalMetrics class instance
            run_phase_subject (RunPhaseSubject): RunPhaseSubject class instance
            virtual_event_agent_camera_models (VirtualEventAgentCameraModel):
                VirtualEventAgentCameraModel class instance
        """
        self._model_updater = ModelUpdater.get_instance()
        self._racer_profiles = racer_profiles
        self._num_of_agents = len(self._racer_profiles)
        self._race_data = race_data
        self._eval_metrics = eval_metrics
        self._run_phase_subject = run_phase_subject
        self._virtual_event_agent_camera_models = virtual_event_agent_camera_models
        self._start_pos_offset = max(min(float(rospy.get_param("START_POS_OFFSET", START_POS_OFFSET)), MAX_START_POS_OFFSET),
                                     MIN_START_POS_OFFSET)
        self._setup()

    def evaluate(self):
        """
        Evaluate graph manager
        """
        if self._race_data.is_continuous:
            self._evaluate()
        else:
            self._evaluate()
            for _ in range(self._race_data.number_of_trials - 1):
                self._current_graph_manager.evaluate(EnvironmentSteps(1))
        # physics is paused at the end of evaluate, so unpause here
        # to make sure virtual event video editor can get the latest
        # metrics. Physics will be paused again at the finsih method of
        # VirtualEvent class.
        self._model_updater.unpause_physics()

    def _evaluate(self):
        """
        Evaluate: step order is important and do not change

        First, reset internal state such as reset metrics, wait for
        sensor to be ready and reset the agent to the starting position.
        Second, it adds the virtual event camera to camera manager to follow specific agent.
        Third, call evaluate without reset again.

        This order is critical important and should not be changed for below reason.

        In the first step, a manual reset is done. In this manual reset step. It will
            1. reset agent metrics such as add pause second to total race second
            and clean up previous left over parameters.
            2. wait for sensor topics to become ready. Especially, when use LIDAR,
            LIDAR topic can take longer to come up.
            3. reset agent to the start line
        After the first step, camera is then attached to racecar through camera manager add.
        We should not add camera into camera manager before graph manager manual reset internal
        state. The reason is because racecar is initially spawn at a hide location. If we add
        camera before reset_internal_state, camera will record video at hide position.

        Last, we will start evaluate without reset again because we have already manually reset.
        It is critical here to NOT reset again. If we reset for a second time, we will double count
        the prepare time before the race
        """
        self._current_graph_manager.reset_internal_state(force_environment_reset=True)
        # Update CameraManager by adding cameras into the current namespace. By doing so
        # a single follow car camera will follow the current active racecar.
        [camera_model.attach() for camera_model in self._virtual_event_agent_camera_models]
        self._current_graph_manager.evaluate(EnvironmentSteps(1), reset_before_eval=False)

    def _setup(self):
        """
        Setup graph manager
        """
        sm_hyperparams_dict = {}
        self._current_graph_manager, _ = get_graph_manager(
            hp_dict=sm_hyperparams_dict,
            agent_list=self._get_agent_list(),
            run_phase_subject=self._run_phase_subject,
            enable_domain_randomization=self._race_data.enable_domain_randomization,
            done_condition=self._race_data.done_condition,
            pause_physics=self._model_updater.pause_physics_service,
            unpause_physics=self._model_updater.unpause_physics_service)

        ds_params_instance = S3BotoDataStoreParameters(
            checkpoint_dict={profile.agent_name: profile.checkpoint for profile in self._racer_profiles})

        self._current_graph_manager.data_store = S3BotoDataStore(params=ds_params_instance,
                                                                 graph_manager=self._current_graph_manager,
                                                                 ignore_lock=True,
                                                                 log_and_cont=True)
        self._current_graph_manager.env_params.seed = 0

        self._current_graph_manager.data_store.wait_for_checkpoints()
        self._current_graph_manager.data_store.modify_checkpoint_variables()

        task_parameters = TaskParameters()
        task_parameters.checkpoint_restore_path = LOCAL_MODEL_DIR

        self._current_graph_manager.create_graph(task_parameters=task_parameters,
                                                 stop_physics=self._model_updater.pause_physics_service,
                                                 start_physics=self._model_updater.unpause_physics_service,
                                                 empty_service_call=EmptyRequest)

    def _get_agent_list(self):
        """
        Get agent list

        Returns:
            list: list of agent config
        """
        agent_list = list()
        for idx, profile in enumerate(self._racer_profiles):
            agent_config = {
                'model_metadata': profile.model_metadata,
                ConfigParams.CAR_CTRL_CONFIG.value: {
                    ConfigParams.LINK_NAME_LIST.value: [
                        link_name.replace('racecar', profile.racecar_name) for link_name in LINK_NAMES],
                    ConfigParams.VELOCITY_LIST.value: [
                        velocity_topic.replace('racecar', profile.racecar_name) for velocity_topic in VELOCITY_TOPICS],
                    ConfigParams.STEERING_LIST.value: [
                        steering_topic.replace('racecar', profile.racecar_name) for steering_topic in STEERING_TOPICS],
                    ConfigParams.CHANGE_START.value: utils.str2bool(rospy.get_param('CHANGE_START_POSITION', False)),
                    ConfigParams.ALT_DIR.value: utils.str2bool(rospy.get_param('ALTERNATE_DRIVING_DIRECTION', False)),
                    ConfigParams.MODEL_METADATA.value: profile.model_metadata,
                    ConfigParams.REWARD.value: reward_function,
                    ConfigParams.AGENT_NAME.value: profile.racecar_name,
                    ConfigParams.VERSION.value: profile.simapp_version,
                    ConfigParams.NUMBER_OF_RESETS.value: self._race_data.number_of_resets,
                    ConfigParams.PENALTY_SECONDS.value: self._race_data.penalty_seconds,
                    ConfigParams.NUMBER_OF_TRIALS.value: self._race_data.number_of_trials,
                    ConfigParams.IS_CONTINUOUS.value: self._race_data.is_continuous,
                    ConfigParams.RACE_TYPE.value: self._race_data.race_type,
                    ConfigParams.COLLISION_PENALTY.value: self._race_data.collision_penalty,
                    ConfigParams.OFF_TRACK_PENALTY.value: self._race_data.off_track_penalty,
                    ConfigParams.START_POSITION.value: get_start_positions(self._num_of_agents,
                                                                           self._start_pos_offset)[idx],
                    ConfigParams.DONE_CONDITION.value: self._race_data.done_condition,
                    ConfigParams.IS_VIRTUAL_EVENT.value: True,
                    ConfigParams.RACE_DURATION.value: self._race_data.race_duration}}

            agent_list.append(create_rollout_agent(agent_config,
                                                   self._eval_metrics[idx],
                                                   self._run_phase_subject))
            agent_list.append(create_obstacles_agent())
            agent_list.append(create_bot_cars_agent(pause_time_before_start=PAUSE_TIME_BEFORE_START))
        return agent_list
