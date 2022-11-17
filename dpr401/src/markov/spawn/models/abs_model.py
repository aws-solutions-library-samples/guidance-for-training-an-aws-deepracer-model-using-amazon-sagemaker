"""this module implement a abstract model"""

import abc
import random
import rospkg
import rospy
import time

from geometry_msgs.msg import Pose
from markov.gazebo_tracker.trackers.get_model_state_tracker import GetModelStateTracker
from threading import RLock
from typing import Optional
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                          SIMAPP_EVENT_ERROR_CODE_500)


class AbsModel(object, metaclass=abc.ABCMeta):
    """Abstract model class

    Attributes
        _model_names (set): set of model names
    """
    _model_names = set()

    def __init__(self, max_retry_attempts: int = 10, backoff_time_sec: float = 1.0):
        """Constructor

        Args:
            max_retry_attempts (int): max retry attempts for waiting spawn/delete to complete
            backoff_time_sec (float): backoff time in seconds for spawn/delete to complete
        """
        self._model_name = None
        self._rospack = rospkg.RosPack()
        self._lock = RLock()
        self._max_retry_attempts = max_retry_attempts
        self._backoff_time_sec = backoff_time_sec

    @property
    def model_name(self) -> str:
        """Return model name

        Returns:
            str: model name
        """
        return self._model_name

    @property
    def rospack(self) -> rospkg.RosPack():
        """Return ros package

        Returns:
            rospkg.RosPack(): ros package
        """
        return self._rospack

    def spawn(self, name: str, pose: Optional[Pose] = None, **kwargs) -> None:
        """Spawn model in gazebo simulator

        Args:
            name (str): model name
            pose (Optional[Pose]): model pose
            **kwargs: Arbitrary keyword arguments
        """
        with self._lock:
            if self._model_name is None and \
                    name not in AbsModel._model_names:
                self._model_name = name
                self._spawn(pose=pose, **kwargs)
                self._wait_for_spawn()
                AbsModel._model_names.add(name)
            else:
                rospy.loginfo("[AbsModel]: {} cannot be spawned "
                              "again without be deleted".format(name))

    def delete(self) -> None:
        """Delete model from gazebo simulator
        """
        with self._lock:
            if self._model_name is not None:
                self._delete()
                self._wait_for_delete()
                AbsModel._model_names.remove(self._model_name)
                self._model_name = None
            else:
                rospy.loginfo(
                    "[AbsModel]: model does not exist and "
                    "cannot be deleted")

    def _wait_for_spawn(self) -> None:
        """Wait for spawn to complete
        """
        try_count = 0
        while True:
            msg = GetModelStateTracker.get_instance().get_model_state(
                model_name=self._model_name,
                relative_entity_name="",
                blocking=True)
            if msg.success:
                rospy.loginfo("[AbsModel]: spawn {} completed".format(self._model_name))
                break
            try_count += 1
            if try_count > self._max_retry_attempts:
                model_name = self._model_name
                self._model_name = None
                log_and_exit("[AbsModel]: spawn {} failed".format(model_name),
                             SIMAPP_SIMULATION_WORKER_EXCEPTION,
                             SIMAPP_EVENT_ERROR_CODE_500)
            rospy.loginfo("[AbsModel]: model {} is in processing of "
                          "spawning".format(self._model_name))
            time.sleep(self._backoff_time_sec)

    def _wait_for_delete(self) -> None:
        """Wait for delete to complete
        """
        try_count = 0
        while True:
            msg = GetModelStateTracker.get_instance().get_model_state(
                model_name=self._model_name,
                relative_entity_name="",
                blocking=True)
            if msg.success:
                rospy.loginfo("[AbsModel]: model {} is in processing of "
                              "deleting".format(self._model_name))
                time.sleep(self._backoff_time_sec)
                try_count += 1
                if try_count > self._max_retry_attempts:
                    log_and_exit("[AbsModel]: delete {} failed".format(self._model_name),
                                 SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                 SIMAPP_EVENT_ERROR_CODE_500)
            else:
                rospy.loginfo("[AbsModel]: delete {} completed".format(self._model_name))
                break

    @abc.abstractmethod
    def _spawn(self, pose: Optional[Pose] = None, **kwargs) -> None:
        """Spawn model in gazebo simulator

        Args:
            pose (Optional[Pose]): model pose
            **kwargs: Arbitrary keyword arguments

        Raises:
            NotImplementedError
        """
        raise NotImplementedError('spawn is not implemented')

    @abc.abstractmethod
    def _delete(self) -> None:
        """Delete model from gazebo simulator

        Raises:
            NotImplementedError
        """
        raise NotImplementedError('delete is not implemented')
