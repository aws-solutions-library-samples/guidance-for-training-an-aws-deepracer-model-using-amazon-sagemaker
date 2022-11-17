import os

from markov.defaults import DEFAULT_SUB_CAMERA
from markov.virtual_event.cameras.virtual_event_abs_camera_model import VirtualEventAbsCameraModel


class VirtualEventTopCameraModel(VirtualEventAbsCameraModel):
    """
    VirtualEventTopCameraModel class
    """
    def __init__(self):
        """
        VirtualEventTopCameraModel constructor
        """
        super().__init__(
            camera_type=DEFAULT_SUB_CAMERA,
            model_name="/{}".format("sub_camera"),
            namespace=DEFAULT_SUB_CAMERA)

    def spawn(self):
        """
        Spawn cameras
        """
        self._camera.spawn_model(None, os.path.join(self._deepracer_path, "models",
                                                    "top_camera", "model.sdf"))
