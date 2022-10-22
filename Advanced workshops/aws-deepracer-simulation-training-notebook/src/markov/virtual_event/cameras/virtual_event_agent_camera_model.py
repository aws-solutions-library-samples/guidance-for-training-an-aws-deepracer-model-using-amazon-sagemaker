import os

from markov.defaults import DEFAULT_MAIN_CAMERA
from markov.virtual_event.cameras.virtual_event_abs_camera_model import VirtualEventAbsCameraModel


class VirtualEventAgentCameraModel(VirtualEventAbsCameraModel):
    """
    VirtualEventCameraManager class
    """
    def __init__(self, camera_namespace, start_pose):
        """
        VirtualEventAgentCameraModel constructor

        Args:
            camera_namespace (str): camera namespace
            start_pose (Pose): camera start pose
        """
        super().__init__(
            camera_type=DEFAULT_MAIN_CAMERA,
            model_name="/{}/{}".format(camera_namespace, "main_camera"),
            namespace=camera_namespace)
        self._start_pose = start_pose

    def spawn(self):
        """
        Spawn cameras
        """
        self._camera.spawn_model(self._start_pose,
                                 os.path.join(self._deepracer_path, "models",
                                              "camera", "model.sdf"))

    def reset_pose(self):
        """
        Reset camera pose
        """
        self._camera.reset_pose(
            car_pose=self._start_pose)
