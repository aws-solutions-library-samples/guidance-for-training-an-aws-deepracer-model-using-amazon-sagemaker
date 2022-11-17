"""constants for spawn"""


class DeepRacerPackages():
    """DeepRacer ROS Packages
    """
    DEEPRACER_SIMULATION_ENVIRONMENT = \
        "deepracer_simulation_environment"


# LIDAR values: these values are defined in racecar.launch
# if you changed these value, please make sure you also update
# racecar.launch lidar input value
LIDAR_360_DEGREE_SAMPLE = "64"
LIDAR_360_DEGREE_HORIZONTAL_RESOLUTION = "1"
LIDAR_360_DEGREE_MIN_ANGLE = "-2.61799"
LIDAR_360_DEGREE_MAX_ANGLE = "2.61799"
LIDAR_360_DEGREE_MIN_RANGE = "0.15"
LIDAR_360_DEGREE_MAX_RANGE = "0.5"
LIDAR_360_DEGREE_RANGE_RESOLUTION = "0.01"
LIDAR_360_DEGREE_NOISE_MEAN = "0.0"
LIDAR_360_DEGREE_NOISE_STDDEV = "0.01"

# sleep seconds after delete/spawn gazebo service call
SLEEP_SECONDS_AFTER_GAZEBO_MODEL_SERVICE_CALL = 5
