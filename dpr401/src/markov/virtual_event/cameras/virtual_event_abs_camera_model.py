import abc
import rospkg

from markov.cameras.camera_factory import CameraFactory
from markov.cameras.camera_manager import CameraManager
from markov.spawn.constants import DeepRacerPackages

# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class VirtualEventAbsCameraModel(ABC):
    """
    VirtualEventAbsCameraModel class
    """
    def __init__(self, camera_type, model_name, namespace):
        """
        VirtualEventAbsCameraModel constructor

        Args:
            camera_type (str): camera type
            model_name (str): camera model name
            namespace (str): camera namespace
        """
        self._camera_manager = CameraManager.get_instance()
        self._namespace = namespace
        self._deepracer_path = rospkg.RosPack().get_path(
            DeepRacerPackages.DEEPRACER_SIMULATION_ENVIRONMENT)
        self._camera = CameraFactory.create_instance(
            camera_type=camera_type,
            model_name=model_name,
            namespace=namespace)
        self.detach()

    @abc.abstractmethod
    def spawn(self):
        """
        Spawn camera
        """
        raise NotImplementedError('VirtualEventAbsCameraModel must implement spawn')

    def detach(self):
        """
        stop following item specifying by namespace through popping camera name
        """
        self._camera_manager.pop(self._namespace)

    def attach(self):
        """
        start following item specifying by namespace through adding camera name
        """
        self._camera_manager.add(self._camera,
                                 self._namespace)
