"""This Singleton class handles gazebo model spawn and delete service"""

import rospy

from markov.track_geom.constants import (SPAWN_URDF_MODEL,
                                         SPAWN_SDF_MODEL,
                                         DELETE_MODEL)
from geometry_msgs.msg import Pose
from markov.rospy_wrappers import ServiceProxyWrapper
from gazebo_msgs.srv import (SpawnModel, DeleteModel,
                             SpawnModelResponse, DeleteModelResponse)
from threading import RLock


class GazeboModel(object):
    """GazeboSpanwer Singleton class"""
    _instance = None
    _instance_lock = RLock()

    @staticmethod
    def get_instance() -> 'GazeboModel':
        """
        Method for getting a reference to the GazeboModel object

        Returns:
            GazeboModel: GazeboModel instance
        """
        with GazeboModel._instance_lock:
            if GazeboModel._instance is None:
                GazeboModel()
            return GazeboModel._instance

    def __init__(self, is_singleton: bool = True):
        """Constructor for GazeboModel

        Args:
            is_singleton (bool): flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if GazeboModel._instance is not None:
                raise RuntimeError("Attempting to construct multiple GazeboModel")
            GazeboModel._instance = self
        rospy.wait_for_service(SPAWN_URDF_MODEL)
        rospy.wait_for_service(SPAWN_SDF_MODEL)
        rospy.wait_for_service(DELETE_MODEL)
        self._spawn_urdf_model = ServiceProxyWrapper(SPAWN_URDF_MODEL,
                                                     SpawnModel)
        self._spawn_sdf_model = ServiceProxyWrapper(SPAWN_SDF_MODEL,
                                                    SpawnModel)
        self._delete_model = ServiceProxyWrapper(DELETE_MODEL,
                                                 DeleteModel)

    def spawn_sdf(self,
                  model_name: str,
                  model_xml: str,
                  robot_namespace: str,
                  initial_pose: Pose,
                  reference_frame: str) -> SpawnModelResponse:
        """spawn sdf model

        http://docs.ros.org/en/jade/api/gazebo_msgs/html/srv/SpawnModel.html

        Args:
            model_name (string): name of the model to be spawn
            model_xml (string): this should be an urdf or gazebo xml
            robot_namespace (string): spawn robot and all ROS interfaces under this namespace
            initial_pose (Pose): only applied to canonical body
            reference_frame (string): initial_pose is defined relative to the frame of this model/body
                                      if left empty or "world", then gazebo world frame is used
                                      if non-existent model/body is specified, an error is returned
                                      and the model is not spawned

        Returns:
            SpawnModelResponse: response msg
        """
        return self._spawn_sdf_model(model_name,
                                     model_xml,
                                     robot_namespace,
                                     initial_pose,
                                     reference_frame)

    def spawn_urdf(self,
                   model_name: str,
                   model_xml: str,
                   robot_namespace: str,
                   initial_pose: Pose,
                   reference_frame: str) -> SpawnModelResponse:
        """spawn urdf model

        http://docs.ros.org/en/jade/api/gazebo_msgs/html/srv/SpawnModel.html

        Args:
            model_name (string): name of the model to be spawn
            model_xml (string): this should be an urdf or gazebo xml
            robot_namespace (string): spawn robot and all ROS interfaces under this namespace
            initial_pose (Pose): only applied to canonical body
            reference_frame (string): initial_pose is defined relative to the frame of this model/body
                                      if left empty or "world", then gazebo world frame is used
                                      if non-existent model/body is specified, an error is returned
                                      and the model is not spawned

        Returns:
            SpawnModelResponse: response msg
        """
        return self._spawn_urdf_model(model_name,
                                      model_xml,
                                      robot_namespace,
                                      initial_pose,
                                      reference_frame)

    def delete(self, model_name: str) -> DeleteModelResponse:
        """delete model

        http://docs.ros.org/en/jade/api/gazebo_msgs/html/srv/DeleteModel.html

        Args:
            model_name (string): name of the Gazebo Model to be deleted

        Returns:
            DeleteModelResponse: response msg
        """
        return self._delete_model(model_name)
