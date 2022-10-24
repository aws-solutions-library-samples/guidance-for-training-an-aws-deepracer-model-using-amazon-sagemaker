import rospy

from markov import utils
from markov.virtual_event.constants import DEFAULT_RACE_DURATION


class VirtualEventRaceData():
    """
    VirtualEventRaceData class
    """
    def __init__(self):
        """
        VirtualEventRaceData constructor
        """
        self._region = rospy.get_param("AWS_REGION", "us-east-1")
        self._race_duration = int(rospy.get_param("RACE_DURATION", DEFAULT_RACE_DURATION))
        self._number_of_trials = int(rospy.get_param("NUMBER_OF_TRIALS", 3))
        self._number_of_resets = int(rospy.get_param("NUMBER_OF_RESETS", 0))
        self._penalty_seconds = float(rospy.get_param("PENALTY_SECONDS", 2.0))
        self._off_track_penalty = float(rospy.get_param("OFF_TRACK_PENALTY", 2.0))
        self._collision_penalty = float(rospy.get_param("COLLISION_PENALTY", 5.0))
        self._is_continuous = utils.str2bool(rospy.get_param("IS_CONTINUOUS", False))
        self._race_type = rospy.get_param("RACE_TYPE", "TIME_TRIAL")
        self._done_condition = any
        self._enable_domain_randomization = False

    @property
    def region(self):
        """
        Returns:
            str: aws region
        """
        return self._region

    @property
    def race_duration(self):
        """
        Returns:
            int: race duration
        """
        return self._race_duration

    @property
    def number_of_trials(self):
        """
        Returns:
            int: number of trials
        """
        return self._number_of_trials

    @property
    def number_of_resets(self):
        """
        Returns:
            int: number of resets
        """
        return self._number_of_resets

    @property
    def penalty_seconds(self):
        """
        Returns:
            float: penalty seconds
        """
        return self._penalty_seconds

    @property
    def off_track_penalty(self):
        """
        Returns:
            float: off track penalty
        """
        return self._off_track_penalty

    @property
    def collision_penalty(self):
        """
        Returns:
            float: collision penalty
        """
        return self._collision_penalty

    @property
    def is_continuous(self):
        """
        Returns:
            bool: True if is continuous, False otherwise
        """
        return self._is_continuous

    @property
    def race_type(self):
        """
        Returns:
            str: race type
        """
        return self._race_type

    @property
    def done_condition(self):
        """
        Returns:
            Callable[[list], bool]: race done condition
        """
        return self._done_condition

    @property
    def enable_domain_randomization(self):
        """
        Returns:
            bool: True if enable domain randomization, False otherwise
        """
        return self._enable_domain_randomization
