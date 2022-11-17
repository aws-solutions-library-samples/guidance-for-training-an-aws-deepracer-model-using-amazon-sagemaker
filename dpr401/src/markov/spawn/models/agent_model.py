"""this module handle all agent model spawn and delete"""

import os
import random
import rospkg
import rospy
import rosnode
import yaml
import time

from markov.spawn.constants import (DeepRacerPackages)
from markov.spawn.gazebo_model import GazeboModel
from markov.spawn.gazebo_xml_loader import GazeboXmlLoader
from markov.spawn.models.abs_model import AbsModel
from geometry_msgs.msg import Pose
from subprocess import Popen
from typing import Optional
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                          SIMAPP_EVENT_ERROR_CODE_500)


class AgentModel(AbsModel):
    """agent model class to handle gazebo spawn and delete
    """
    def __init__(self, max_retry_attempts: int = 10, backoff_time_sec: float = 1.0):
        """Constructor

        Args:
            max_retry_attempts (int): max retry attempts for waiting spawn/delete to complete
            backoff_time_sec (float): backoff time in seconds for spawn/delete to complete
        """
        super().__init__(max_retry_attempts=max_retry_attempts,
                         backoff_time_sec=backoff_time_sec)
        self._agent_file_path = os.path.join(
            self._rospack.get_path(DeepRacerPackages.DEEPRACER_SIMULATION_ENVIRONMENT),
            "urdf",
            "deepracer_kinematics",
            "racecar.xacro")
        self._control_nodes = ["/{}/controller_manager", "/{}/robot_state_publisher"]

    def _spawn(self, pose: Optional[Pose] = None, **kwargs) -> None:
        """spawn agent model in gazebo simulator

        Args:
            pose (Optional[Pose]): model pose
            **kwargs: Arbitrary keyword arguments

        """
        model_urdf = GazeboXmlLoader.parse(file_path=self._agent_file_path,
                                           **kwargs,
                                           racecar_name=self._model_name)

        # load robot_description into ros parameter server
        rospy.set_param("/{}/robot_description".format(self._model_name), model_urdf)

        # roslaunch controller_manager and robot_state_publisher
        Popen("roslaunch deepracer_simulation_environment racecar_control_kinematics.launch \
            racecar_name:={} make_required:={} __ns:={}".format(self._model_name,
                                                                "false",
                                                                self._model_name),
              shell=True,
              executable="/bin/bash")
        self._wait_for_rosnode(alive_nodes=[node.format(self._model_name) for node in self._control_nodes])

        # spawn agent urdf model
        GazeboModel.get_instance().spawn_urdf(model_name=self._model_name,
                                              model_xml=model_urdf,
                                              robot_namespace="/{}".format(self._model_name),
                                              initial_pose=pose if pose else Pose(),
                                              reference_frame='')

    def _delete(self) -> None:
        """delete the agent model

        we have noticed such error message that after the agent1 is deleted
        its camera service (/agent1/camera/zed/set_parameters) is still left over.
        When spawn the next agent with the same namespace, we will have error message below.
        However, even though it is polluting the logs, we have not seen any issue yet.

        [ERROR] [1616111166.451933744, 49.818000000]:
        Tried to advertise a service that is already advertised in this node [/agent1/camera/zed/set_parameters]
        """
        # kill agent controller manager and robot state publisher node
        Popen("rosnode kill /{}/controller_manager".format(self._model_name),
              shell=True,
              executable="/bin/bash")
        Popen("rosnode kill /{}/robot_state_publisher".format(self._model_name),
              shell=True,
              executable="/bin/bash")
        self._wait_for_rosnode(dead_nodes=[node.format(self._model_name) for node in self._control_nodes])

        # delete agent model from gazebo
        GazeboModel.get_instance().delete(model_name=self._model_name)

    def _is_ros_node_alive(self, node_name: str) -> bool:
        """Return whether ros node is alive or not

        Args:
            node_name (str): ros node name

        Returns:
            bool: True is ros node is alive, False otherwise.
        """
        if node_name in rosnode.get_node_names():
            return True
        return False

    def _wait_for_rosnode(self, alive_nodes: Optional[list] = None, dead_nodes: Optional[list] = None) -> None:
        """Wait for starting/killing ros node to complete

        Args:
            alive_nodes(Optional[list]): list of alive nodes which should be started
            dead_nodes(Optional[list]): list of dead nodes which should be killed

        """
        try_count = 0
        alive_nodes = alive_nodes or list()
        dead_nodes = dead_nodes or list()
        while True:
            if all([self._is_ros_node_alive(node) for node in alive_nodes]) and \
                    all([not self._is_ros_node_alive(node) for node in dead_nodes]):
                break
            try_count += 1
            if try_count > self._max_retry_attempts:
                # Only reset model name to None is wait for roslaunch start failure.
                # for rosnode kill failure, we should keep model name as it is before
                if alive_nodes and not dead_nodes:
                    self._model_name = None
                log_and_exit("[AgentModel]: _wait_for_rosnode starting ros node {} "
                             "or killing ros node {} failed".format(alive_nodes, dead_nodes),
                             SIMAPP_SIMULATION_WORKER_EXCEPTION,
                             SIMAPP_EVENT_ERROR_CODE_500)
            rospy.loginfo("[AgentModel]: _wait_for_rosnode starting ros node {} "
                          "or killing ros node {}".format(alive_nodes, dead_nodes))
            time.sleep(self._backoff_time_sec)
