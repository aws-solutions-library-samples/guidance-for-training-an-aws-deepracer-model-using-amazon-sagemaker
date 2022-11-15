import os
import rospkg
import time
import markov.rollout_constants as const

from geometry_msgs.msg import Pose
from markov.architecture.constants import Input
from markov.gazebo_utils.model_updater import ModelUpdater
from markov.constants import DEFAULT_COLOR
from markov.spawn.models.agent_model import AgentModel
from markov.spawn.constants import DeepRacerPackages
from markov.track_geom.track_data import TrackData
from markov.sensors.constants import (
    LIDAR_360_DEGREE_SAMPLE,
    LIDAR_360_DEGREE_HORIZONTAL_RESOLUTION,
    LIDAR_360_DEGREE_MIN_ANGLE,
    LIDAR_360_DEGREE_MAX_ANGLE,
    LIDAR_360_DEGREE_MIN_RANGE,
    LIDAR_360_DEGREE_MAX_RANGE,
    LIDAR_360_DEGREE_RANGE_RESOLUTION,
    LIDAR_360_DEGREE_NOISE_MEAN,
    LIDAR_360_DEGREE_NOISE_STDDEV
)


class VirtualEventAgentModel():
    """
    virtualEventModleUpdater class
    """
    def __init__(self, profile, hide_position):
        """
        VirtualEventAgentModel constructor

        Args:
            profile:(object) race profile class instance
            hide_position (tuple): (x, y) of hide position
        """
        self._profile = profile
        self._hide_position = hide_position
        self._model_updater = ModelUpdater.get_instance()
        self._agent_model = AgentModel()
        self._deepracer_path = rospkg.RosPack().get_path(DeepRacerPackages.DEEPRACER_SIMULATION_ENVIRONMENT)
        body_shell_path = os.path.join(self._deepracer_path, "meshes", "f1")
        self._valid_body_shells = \
            set(".".join(f.split(".")[:-1]) for f in os.listdir(body_shell_path) if os.path.isfile(
                os.path.join(body_shell_path, f)))
        self._valid_body_shells.add(const.BodyShellType.DEFAULT.value)
        self._valid_car_colors = set(e.value for e in const.CarColorType if "f1" not in e.value)
        self._track_data = TrackData.get_instance()

    def spawn(self):
        """
        Spawn agent in gazebo
        """
        hide_pose = Pose()
        hide_pose.position.x = self._hide_position[0]
        hide_pose.position.y = self._hide_position[1]
        # spawn agent
        self._agent_model.spawn(name=self._profile.racecar_name,
                                pose=hide_pose,
                                racecar_bitmask=str(2**int(self._profile.index)),
                                include_second_camera="true" if Input.STEREO.value in self._profile.sensors else "false",
                                include_lidar_sensor=str(any(["lidar" in sensor.lower() for sensor in self._profile.sensors])).lower(),
                                body_shell_type=self._get_body_shell(),
                                lidar_360_degree_sample=str(LIDAR_360_DEGREE_SAMPLE),
                                lidar_360_degree_horizontal_resolution=str(LIDAR_360_DEGREE_HORIZONTAL_RESOLUTION),
                                lidar_360_degree_min_angle=str(LIDAR_360_DEGREE_MIN_ANGLE),
                                lidar_360_degree_max_angle=str(LIDAR_360_DEGREE_MAX_ANGLE),
                                lidar_360_degree_min_range=str(LIDAR_360_DEGREE_MIN_RANGE),
                                lidar_360_degree_max_range=str(LIDAR_360_DEGREE_MAX_RANGE),
                                lidar_360_degree_range_resolution=str(LIDAR_360_DEGREE_RANGE_RESOLUTION),
                                lidar_360_degree_noise_mean=str(LIDAR_360_DEGREE_NOISE_MEAN),
                                lidar_360_degree_noise_stddev=str(LIDAR_360_DEGREE_NOISE_STDDEV))
        # even though in spawn method, we wait_for_spawn by checking model state through get_model_state. However,
        # it looks like even model state is ready, immediate update gazebo visual can cause transient visual update failure
        # even with retry logic in update visual. Therefore, sleep for couple seconds here.
        time.sleep(3)
        self._update_visual()

    def delete(self):
        """
        Delete agent in gazebo
        """
        self._agent_model.delete()
        self._track_data.remove_object(name=self._profile.racecar_name)

    def _update_visual(self):
        """
        Update agent visual in gazebo
        """
        # update agent  shell and color
        visuals = self._model_updater.get_model_visuals(self._profile.racecar_name)
        body_shell_type = self._get_body_shell()
        if const.F1 not in body_shell_type:
            if hasattr(self._profile, "carConfig") and \
                    hasattr(self._profile.carConfig, "carColor"):
                car_color = self._profile.carConfig.carColor if self._profile.carConfig.carColor in self._valid_car_colors \
                    else DEFAULT_COLOR
            else:
                car_color = DEFAULT_COLOR
            self._model_updater.update_color(visuals, car_color)

    def _get_body_shell(self):
        """
        Get agent body shell type

        Returns:
            str: agent body shell type
        """
        body_shell_type = const.BodyShellType.DEFAULT.value
        if hasattr(self._profile, "carConfig") and hasattr(self._profile.carConfig, "bodyShellType"):
            body_shell_type = self._profile.carConfig.bodyShellType \
                if self._profile.carConfig.bodyShellType in self._valid_body_shells \
                else const.BodyShellType.DEFAULT.value
        return body_shell_type
