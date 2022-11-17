'''This module implements concrete reset rule for the immobilization'''

import markov.agent_ctrl.constants as const

from markov.reset.abstract_reset_rule import AbstractResetRule
from markov.reset.constants import AgentCtrlStatus, AgentPhase
from markov.metrics.constants import EpisodeStatus


class ImmobilizedResetRule(AbstractResetRule):
    name = EpisodeStatus.IMMOBILIZED.value

    def __init__(self):
        super(ImmobilizedResetRule, self).__init__(ImmobilizedResetRule.name)
        self._immobilize_count = 0
        self._last_x = 0.0
        self._last_y = 0.0

    def _update(self, agent_status):
        '''Update the immobilized reset rule done flag

        Args:
            agent_status (dict): agent status dictionary
        '''
        agent_phase = agent_status[AgentCtrlStatus.AGENT_PHASE.value]
        x = agent_status[AgentCtrlStatus.X.value]
        y = agent_status[AgentCtrlStatus.Y.value]
        dist = ((((x - self._last_x)**2) + ((y - self._last_y)**2))**0.5)
        # If car is moving at 0.05 m/s with a step rate of 1/15 per second,
        # dist should be 0.05/15 = 0.003 (m), by using a scale factor ~10,
        # we will use distance between each step less than 0.0003 (m) as
        # immobilized.
        # Also, when car is at 0 m/s, its distance between each step call
        # is at the level between 1e-8 (m) and 1e-6 (m). This can capture the true
        # immobilization if car is not moving.
        if agent_phase == AgentPhase.RUN.value and dist <= 0.0003:
            self._immobilize_count += 1
        else:
            self._immobilize_count = 0
        if self._immobilize_count >= const.NUM_STEPS_TO_CHECK_STUCK:
            self._immobilize_count = 0
            self._done = True
        self._last_x, self._last_y = x, y
