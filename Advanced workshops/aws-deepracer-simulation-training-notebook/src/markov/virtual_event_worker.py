import logging
import os
import rospy
from markov import utils
from markov.gazebo_utils.model_updater import ModelUpdater
from markov.virtual_event.virtual_event import VirtualEvent
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_WORKER_EXCEPTION)


LOG = Logger(__name__, logging.INFO).get_logger()


def main():
    """
    Main function for virutal event
    """
    ModelUpdater.get_instance().pause_physics()
    virtual_event = VirtualEvent()
    LOG.info("[VirtualEventWorker] virtual event start.")

    while True:
        if not virtual_event.is_event_end:
            virtual_event.poll()
        if virtual_event.is_event_end:
            break
        if virtual_event.setup():
            virtual_event.start()
            virtual_event.finish()

    LOG.info("[VirtualEventWorker] virtual event end.")
    utils.cancel_simulation_job()


if __name__ == '__main__':
    try:
        rospy.init_node('virtual_event_manager', anonymous=True)
        main()
    except Exception as ex:
        log_and_exit("Virtual event worker error: {}".format(ex),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)
