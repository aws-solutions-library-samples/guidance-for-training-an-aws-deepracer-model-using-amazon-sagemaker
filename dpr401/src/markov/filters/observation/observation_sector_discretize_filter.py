from rl_coach.core_types import ObservationType
from rl_coach.filters.observation.observation_filter import ObservationFilter
from rl_coach.spaces import ObservationSpace
import numpy as np
from markov.log_handler.deepracer_exceptions import GenericRolloutException
from markov.sensors.constants import (
    LIDAR_360_DEGREE_MIN_RANGE,
    LIDAR_360_DEGREE_MAX_RANGE,
)


class ObservationSectorDiscretizeFilter(ObservationFilter):
    """
    Split the observation space into sectors and categorize the values to binary based on clipping distance
    """
    def __init__(self, num_sectors, num_values_per_sector, clipping_dist):
        """

        Args:
            num_sectors (int): number of sectors in lidar observation
            num_values_per_sector (int): number of discretized values in each sector
            clipping_dist (float): maximum distance to use for lidar raw data
        """
        super().__init__()
        num_sectors = max(num_sectors, 1)
        num_values_per_sector = max(num_values_per_sector, 1)
        clipping_dist = max(min(clipping_dist, LIDAR_360_DEGREE_MAX_RANGE), LIDAR_360_DEGREE_MIN_RANGE)
        self._num_sectors = num_sectors
        self._clipping_dist = clipping_dist
        self._discrete_range = self._clipping_dist / num_values_per_sector
        self._num_values_per_sector = num_values_per_sector

    def validate_input_observation_space(self, input_observation_space: ObservationSpace):
        if input_observation_space.shape[0] % self._num_sectors != 0:
            raise GenericRolloutException("Number of total lidar values is not divisible by number of values in each sector")

    def filter(self, observation: ObservationType, update_internal_state: bool = True) -> ObservationType:
        # Divide the raw lidar data into sectors and take the min from raw lidar data in each sector
        observation = np.min(observation.reshape(-1, int(observation.shape[0] / self._num_sectors)), axis=1)
        # Clip the observation data to clipping distance.
        np.minimum(observation, self._clipping_dist, out=observation)
        # Discretize the observation sector data
        observation = np.floor(observation / self._discrete_range).astype(np.int)
        # One hot encode the discretized observation sector data
        one_hot_encode = np.zeros((observation.size, self._num_values_per_sector + 1))
        one_hot_encode[np.arange(observation.size), observation] = 1
        # Get rid of last column as it means lidar didn't detect anything
        one_hot_encode = np.delete(one_hot_encode, -1, axis=1)
        result = one_hot_encode.reshape(-1).astype(float)
        return result

    def get_filtered_observation_space(self, input_observation_space: ObservationSpace) -> ObservationSpace:
        input_observation_space.shape[0] = self._num_sectors * self._num_values_per_sector
        input_observation_space.high = 1.0
        input_observation_space.low = 0.0
        return input_observation_space
