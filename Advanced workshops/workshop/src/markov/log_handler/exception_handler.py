# Handles logging and graceful exit
import json
import logging
import os
import io
import signal
import time
import datetime
import inspect
from collections import OrderedDict
import traceback
import re
from markov.log_handler.constants import (SIMAPP_ERROR_HANDLER_EXCEPTION, SIMAPP_EVENT_SYSTEM_ERROR,
                                          SIMAPP_EVENT_USER_ERROR, SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_EVENT_ERROR_CODE_400, SIMAPP_ERROR_EXIT, FAULT_MAP,
                                          UNCLASSIFIED_FAULT_CODE, EXCEPTION_HANDLER_SYNC_FILE,
                                          ERROR_HANDLER_EXCEPTION_FAULT_CODE,
                                          SAGEONLY_SIMAPP_JOB_PID_FILE_PATH, SAGEONLY_TRAINING_JOB_PID_FILE_PATH,
                                          SAGEONLY_PID_FILE_NOT_PRESENT_TIME_OUT,
                                          SAGEONLY_PID_FILE_NOT_PRESENT_SLEEP_TIME, CONDA_ENV_NAME,
                                          CLOUDWATCH_LOG_WORKER_SLEEP_TIME, CONDA_DEFAULT_ENV)
from markov.log_handler.logger import Logger
from markov.constants import DEEPRACER_JOB_TYPE_ENV, DeepRacerJobType

LOG = Logger(__name__, logging.INFO).get_logger()


def log_and_exit(msg, error_source, error_code):
    '''Helper method that logs an exception and exits the application.
    In case of multiple exceptions due to nodes failing, only the first exception will be logged
    using logic to check if the sync file ERROR.txt exists in the environment.

    Args:
        msg (String): The message to be logged
        error_source (String): The source of the error, training worker, rolloutworker, etc
        error_code (String): 4xx or 5xx error

    Returns:
        JSON string data format: Consists of the error log dumped (if sync file not present)
    '''
    try:
        s3_crash_status_file_name = os.environ.get("CRASH_STATUS_FILE_NAME", None)
        #TODO: Find an atomic way to check if file is present else create
        # If the sync file is already present, skip logging
        if os.path.isfile(EXCEPTION_HANDLER_SYNC_FILE):
            simapp_exit_gracefully()
        else:
            # Create a sync file ERROR.txt in the environment and log the exception
            with open(EXCEPTION_HANDLER_SYNC_FILE, 'w') as sync_file:
                dict_obj = OrderedDict()
                json_format_log = dict()
                fault_code = get_fault_code_for_error(msg)
                file_name = inspect.stack()[1][1].split("/")[-1]
                function_name = inspect.stack()[1][3]
                line_number = inspect.stack()[1][2]
                dict_obj['date'] = str(datetime.datetime.now())
                dict_obj['function'] = "{}::{}::{}".format(file_name, function_name, line_number)
                dict_obj['message'] = msg
                dict_obj["exceptionType"] = error_source
                if error_code == SIMAPP_EVENT_ERROR_CODE_400:
                    dict_obj["eventType"] = SIMAPP_EVENT_USER_ERROR
                else:
                    dict_obj["eventType"] = SIMAPP_EVENT_SYSTEM_ERROR
                dict_obj["errorCode"] = error_code
                #TODO: Include fault_code in the json schema to track faults - pending cloud team assistance
                #dict_obj["faultCode"] = fault_code
                json_format_log["simapp_exception"] = dict_obj
                json_log = json.dumps(json_format_log)
                LOG.error(json_log)
                sync_file.write(json_log)
                # Temporary fault code log
                LOG.error("ERROR: FAULT_CODE: {}".format(fault_code))
            simapp_exit_gracefully(json_log=json_log,
                                   s3_crash_status_file_name=s3_crash_status_file_name)

    except Exception as ex:
        msg = "Exception thrown in logger - log_and_exit: {}".format(ex)
        dict_obj = OrderedDict()
        json_format_log = dict()
        fault_code = ERROR_HANDLER_EXCEPTION_FAULT_CODE
        dict_obj['date'] = str(datetime.datetime.now())
        dict_obj['function'] = 'exception_handler.py::log_and_exit::66'
        dict_obj['message'] = msg
        dict_obj["exceptionType"] = SIMAPP_ERROR_HANDLER_EXCEPTION
        dict_obj["eventType"] = SIMAPP_EVENT_SYSTEM_ERROR
        dict_obj["errorCode"] = SIMAPP_EVENT_ERROR_CODE_500
        #TODO: Include fault_code in the json schema to track faults - pending cloud team assistance
        #dict_obj["faultCode"] = fault_code
        json_format_log["simapp_exception"] = dict_obj
        json_log = json.dumps(json_format_log)
        LOG.error(json_log)
        # Temporary fault code log
        LOG.error("ERROR: FAULT_CODE: {}".format(fault_code))
        simapp_exit_gracefully(json_log=json_log,
                               s3_crash_status_file_name=s3_crash_status_file_name)


def read_pids_from_file(file_path):
    """Read pids from file path

    Args:
        file_path (str): files contains pids to kill (each pid delimited by \n)

    Returns:
        list: list of pids to kill
    """
    # define an empty list
    pids = []
    try:
        # open file and read the content in a list
        with open(file_path, 'r') as fp:
            for line in fp:
                # remove linebreak which is the last character of the string
                pid = line[:-1]
                # add item to the list
                pids.append(int(pid))
        return pids
    except FileNotFoundError:
        LOG.info("DeepRacer read job pid file not found: %s", file_path)
    except Exception as ex:
        LOG.info("DeepRacer read job pid exception: %s", ex)


def kill_sagemaker_simapp_jobs_by_pid():
    """This method reads from the simulation and training pid files and kills the processes one by one.
    """
    total_wait_time = 0
    while not os.path.exists(SAGEONLY_SIMAPP_JOB_PID_FILE_PATH) or not os.path.exists(SAGEONLY_TRAINING_JOB_PID_FILE_PATH):
        if total_wait_time >= SAGEONLY_PID_FILE_NOT_PRESENT_TIME_OUT:
            LOG.info("simapp_exit_gracefully - Stopped waiting. SimApp Pid Exists=%s, Training Pid Exists=%s.",
                     os.path.exists(SAGEONLY_SIMAPP_JOB_PID_FILE_PATH),
                     os.path.exists(SAGEONLY_TRAINING_JOB_PID_FILE_PATH))
            return
        LOG.info("simapp_exit_gracefully - Waiting for simapp and training job to come up.")
        time.sleep(SAGEONLY_PID_FILE_NOT_PRESENT_SLEEP_TIME)
        total_wait_time += SAGEONLY_PID_FILE_NOT_PRESENT_SLEEP_TIME

    simapp_pids = read_pids_from_file(SAGEONLY_SIMAPP_JOB_PID_FILE_PATH)
    training_pids = read_pids_from_file(SAGEONLY_TRAINING_JOB_PID_FILE_PATH)
    LOG.info("simapp_exit_gracefully - SimApp pids=%s, Training pids=%s.", simapp_pids, training_pids)

    # The sleep time is for logs to get uploaded to cloudwatch
    time.sleep(CLOUDWATCH_LOG_WORKER_SLEEP_TIME)

    pids_to_kill = []
    if os.environ.get(CONDA_DEFAULT_ENV) == CONDA_ENV_NAME:
        pids_to_kill = simapp_pids
        pids_to_kill.extend(training_pids)
    else:
        pids_to_kill = training_pids
        pids_to_kill.extend(simapp_pids)

    LOG.info("simapp_exit_gracefully - Killing pids %s.", pids_to_kill)
    for pid in pids_to_kill:
        kill_by_pid(pid)


def kill_by_pid(pid):
    """Kill the process refered by pid by raising a SIGTERM.

    Args:
        pid (int): pid of the process to kill.
    """
    try:
        LOG.info("simapp_exit_gracefully - Killing pid %s.", pid)
        os.killpg(pid, signal.SIGTERM)
        LOG.info("simapp_exit_gracefully - Killed pid %s.", pid)
    except Exception as ex:
        LOG.error("simapp_exit_gracefully - Process %s already died =%s", pid, ex)


def simapp_exit_gracefully(simapp_exit=SIMAPP_ERROR_EXIT, json_log=None,
                           s3_crash_status_file_name=None):
    # simapp exception leading to exiting the system
    # - close the running processes
    # - upload simtrace data to S3
    LOG.info("simapp_exit_gracefully: simapp_exit-{}".format(simapp_exit))
    LOG.info("Terminating simapp simulation...")
    callstack_trace = ''.join(traceback.format_stack())
    LOG.info("simapp_exit_gracefully - callstack trace=Traceback (callstack)\n{}".format(callstack_trace))
    exception_trace = traceback.format_exc()
    LOG.info("simapp_exit_gracefully - exception trace={}".format(exception_trace))
    upload_to_s3(json_log=json_log,
                 s3_crash_status_file_name=s3_crash_status_file_name)
    if os.environ.get(DEEPRACER_JOB_TYPE_ENV) == DeepRacerJobType.SAGEONLY.value:
        LOG.info("simapp_exit_gracefully - Job type is SageOnly. Killing SimApp and Training jobs by PID")
        kill_sagemaker_simapp_jobs_by_pid()
    if simapp_exit == SIMAPP_ERROR_EXIT:
        LOG.info("Calling cancel_simulation_job because of failure.")
        # I know this dynamic import can be considered as bad code design
        # however, it's needed to playaround the circular import issue
        # without large scale code change in all places that import log_and_exit
        # TODO: refactor this when we migrate entirely to python 3
        from markov import utils
        utils.cancel_simulation_job()



# the global variable for upload_to_s3
is_upload_to_s3_called = False


def upload_to_s3(json_log, s3_crash_status_file_name):
    if s3_crash_status_file_name is None or json_log is None:
        LOG.info("simapp_exit_gracefully - skipping s3 upload.")
        return
    # this global variable is added to avoid us running into infinte loop
    # because s3 client could call log and exit as well.
    global is_upload_to_s3_called
    if not is_upload_to_s3_called:
        is_upload_to_s3_called = True
        try:
            # I know this dynamic import can be considered as bad code design
            # however, it's needed to playaround the circular import issue
            # without large scale code change in all places that import log_and_exit
            # TODO: refactor this when we migrate entirely to python 3
            from markov import utils
            from markov.boto.s3.s3_client import S3Client
            LOG.info("simapp_exit_gracefully - first time upload_to_s3 called.")
            s3_client = S3Client()
            s3_key = os.path.normpath(os.path.join(os.environ.get("YAML_S3_PREFIX", ''),
                                                   s3_crash_status_file_name))
            s3_client.upload_fileobj(bucket=os.environ.get("YAML_S3_BUCKET", ''),
                                     s3_key=s3_key,
                                     fileobj=io.BytesIO(json_log.encode()),
                                     s3_kms_extra_args=utils.get_s3_extra_args())
            LOG.info("simapp_exit_gracefully - Successfully uploaded simapp status to \
                      s3 bucket {} with s3 key {}.".format(os.environ.get("YAML_S3_BUCKET", ''),
                                                           s3_key))
        except Exception as ex:
            LOG.error("simapp_exit_gracefully - upload to s3 failed=%s", ex)


def get_fault_code_for_error(msg):
    '''Helper method that classifies an error message generated in log_and_exit 
    into individual error codes from the maintained FAULT_MAP. If an unseen error
    is seen, it is classified into fault code 0

    Args:
        msg (String): The message to be classified

    Returns:
        String: Consists of classified fault code
    '''
    for fault_code, err in FAULT_MAP.items():
        # Match errors stored in FAULT_MAP with the exception thrown
        classified = re.search(r"{}".format(err.lower()), msg.lower()) is not None
        if classified:
            return str(fault_code)
    return UNCLASSIFIED_FAULT_CODE
