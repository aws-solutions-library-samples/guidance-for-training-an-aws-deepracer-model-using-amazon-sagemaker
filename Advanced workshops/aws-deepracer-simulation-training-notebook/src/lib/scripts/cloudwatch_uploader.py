#!/usr/bin/env python
"""
Upload CloudWatch Logs to log streams.
"""
import argparse
import itertools
import json
import os
import time
import uuid
from datetime import datetime
from sys import getsizeof
from markov.boto.cloudwatch.cloudwatch_client import CloudWatchClient
from markov.log_handler.constants import (CLOUDWATCH_LOG_WORKER_SLEEP_TIME,
                                          NUM_CHARACTERS_IN_CW_LOG,
                                          MAX_CLOUDWATCH_PUT_LOGS_BATCH_SIZE_BYTES,
                                          MAX_CLOUDWATCH_LOG_EVENT_BATCH_LENGTH,
                                          LOG_FILE_MAX_LIMIT_BYTES)


class CloudWatchUploader(object):
    def __init__(self, cw_log_group_name, cw_log_stream_name, log_symlink_file_path):
        """Initialize a CloudWatch logs uploader

        Args:
            cw_log_group_name (str): cloudwatch group name
            cw_log_stream_name (str): cloudwatch stream name
            log_file_path (str): log file path to monitor
        """
        self._cw_client = CloudWatchClient()
        self._cw_log_group_name = cw_log_group_name
        self._cw_log_stream_name = cw_log_stream_name
        self._log_symlink_file_path = log_symlink_file_path
        self._log_file_path = self._get_new_log_file_path()

        self._cw_client.create_log_group(self._cw_log_group_name)
        self._cw_client.create_log_stream(self._cw_log_group_name, self._cw_log_stream_name)

        self._log_file_pointer = self._get_log_file_pointer()
        self._is_job_running = True
        self._sequence_token = None
        self._create_simlink()

    @property
    def log_file_path(self):
        """ Property to get the log file path

        Returns:
            (str) Path to the log file for cloudwatch logs
        """
        return self._log_file_path

    def _create_simlink(self):
        """ Atomic modification of symbolic link

        Training and simulation logs are piped to symbolic link text file.
        If this file grows larger, then it will be pointed to a new file and older logs
        are deleted.
        """
        # Since the script is called multiple times to upload different logs, there will be a race-condition on the
        # temporary file. Hence using a UUID to keep the file name unique when called from different process.
        tmp_symlink_path = os.path.join(os.path.dirname(self._log_symlink_file_path), str(uuid.uuid4()))
        os.symlink(self.log_file_path, tmp_symlink_path)
        os.replace(tmp_symlink_path, self._log_symlink_file_path)

    def _get_new_log_file_path(self):
        """ Path to a new log file with timestamp

        Training and simulation logs are piped to symbolic link text file.
        If file grows larger, then it will be pointed to a new file and older logs
        are deleted. This function will provide path to create a new log file.

        Returns:
            (str) New log file path
        """
        current_date_time_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return os.path.join(
            os.path.dirname(self._log_symlink_file_path),
            os.path.basename(self._log_symlink_file_path) + "_{}.txt".format(current_date_time_str))

    def _get_log_file_pointer(self):
        """Get the monitored log file's file pointer

        Returns:
            fp: file pointer for the log file
        """
        file_pointer = open(self.log_file_path, "a+")
        file_pointer.seek(0, 2)
        return file_pointer

    @classmethod
    def _get_cw_data_format(cls, msg):
        """read the log message and convert it into a log event dict.

        Cloudwatch logs allows 1,048,576 bytes of UTF-8. Each character would be from 1 to 4 bytes.
        So restricting the number of characters to 250000 characters, considering each
        character is 4 bytes. If this is more than 250000 characters, we can no way put them in a
        single batch to upload to cloudwatch. Only way is to break this message into multiple log_event.

        Args:
            msg (str): log message

        Returns:
            (list): List of dictionary of all the log events.
        """
        log_events = list()
        while msg:
            log_events.append({
                "timestamp": int(time.time() * 1000),
                "message": str(msg[:NUM_CHARACTERS_IN_CW_LOG])
            })
            msg = msg[NUM_CHARACTERS_IN_CW_LOG:]
        return log_events

    def _upload_logs_to_cloudwatch(self, batch):
        """Upload the batch of log events to cloudwatch.

        Args:
            batch (list): list of dictionaries of log events.
        """
        self._sequence_token = self._cw_client.put_log_events(
            self._cw_log_group_name, self._cw_log_stream_name, batch, self._sequence_token)

    def is_log_file_reached_max_limit(self):
        """ Check if the log file reached the max limit

        Returns:
            (bool): True if the file size has reached max limit else False
        """
        log_file_size = os.stat(self.log_file_path).st_size
        if log_file_size > LOG_FILE_MAX_LIMIT_BYTES:
            return True
        return False

    def _monitor_log_file(self):
        """Monitor the log file.

        When the below command gets executed, all the output goes to .log file
        roslaunch deepracer_simulation_environment $SIMULATION_LAUNCH_FILE > /opt/ml/simapp.log

        The way it works is, /opt/ml/simapp.log is no more an actual file, its a symlink to other log file. Lets say
        /opt/ml/simapp.log -> /opt/ml/simapp_20210721_1015_0000.log

        When this file exceeds the limit of 1GB, I do the following

         1. Create a new log file say /opt/ml/simapp_20210721_1030_0000.log
         2. prev_log_fp - Is a pointer that is still pointing to older log file
         3. Now change the symlink of /opt/ml/simapp.log to new log file /opt/ml/simapp_20210721_1030_0000.log.
            This way it starts writing the logs to a new file.
         4. lines = prev_log_fp.read() This will read everything from the prev_log_fp which is pointing to older log file.
         5. Delete the older log file /opt/ml/simapp_20210721_1015_0000.log
        This way we donot lose any data.

        Yields:
            str: lines in the log file.
        """
        # If the generator is called in the middle of the sentence completion, then the logs are split
        # between two lines. To avoid that having an incomplete_line variable which is logged in the next
        # uploading of cloudwatch log event.
        incomplete_line = ""
        try:
            while self._is_job_running:
                is_max_log_limit = self.is_log_file_reached_max_limit()

                if is_max_log_limit:
                    prev_log_file_path = self.log_file_path
                    log_pointer = self._log_file_pointer
                    self.log_file_path = self._get_new_log_file_path()
                    self._log_file_pointer = self._get_log_file_pointer()
                    self._create_simlink()
                    lines = log_pointer.read()
                    os.remove(prev_log_file_path)
                else:
                    lines = self._log_file_pointer.read()

                if not lines:
                    time.sleep(CLOUDWATCH_LOG_WORKER_SLEEP_TIME)
                    continue
                log_line_splits = lines.split('\n')
                # When the logs are split on new line and if the last value is empty then its a complete sentence.
                log_line_splits[0] = "{}{}".format(incomplete_line, log_line_splits[0])
                incomplete_line = log_line_splits.pop()
                yield log_line_splits
        except Exception as ex:
            print(ex)
        finally:
            self._log_file_pointer.close()
            self._is_job_running = False

    def run(self):
        """Run the cloudwatch uploader. It monitors the log file, uploads the content to cloudwatch.
        """
        for log_line_splits in self._monitor_log_file():
            try:
                # Although this is good for developers to look at logs at one place. I think for production we need
                # to comment this print statement. This is logging all the RoboMaker and SageMaker logs in the
                # default cloudwatch logs. This increases computation and cost
                # print(log_line_splits)  # for logging in main sageonly job
                cw_batch = [CloudWatchUploader._get_cw_data_format(line) for line in log_line_splits if line]
                # Flatten the nested lists of log messages
                cw_batch = list(itertools.chain(*cw_batch))
                # If cloudwatch batch is empty then skip logging.
                # Cloudwatch has a hard limit on number of bytes in the batch and the length on batch size.
                if cw_batch and getsizeof(json.dumps(cw_batch)) < MAX_CLOUDWATCH_PUT_LOGS_BATCH_SIZE_BYTES and \
                        len(cw_batch) < MAX_CLOUDWATCH_LOG_EVENT_BATCH_LENGTH:
                    self._upload_logs_to_cloudwatch(cw_batch)
                else:
                    start_index, total_bytes = 0, 0
                    for i, log in enumerate(cw_batch):
                        log_size = getsizeof(json.dumps(log))
                        if total_bytes + log_size > MAX_CLOUDWATCH_PUT_LOGS_BATCH_SIZE_BYTES or \
                                (i - start_index) >= MAX_CLOUDWATCH_LOG_EVENT_BATCH_LENGTH:
                            self._upload_logs_to_cloudwatch(cw_batch[start_index:i])
                            start_index = i
                            total_bytes = 0
                        total_bytes += log_size
                    self._upload_logs_to_cloudwatch(cw_batch[start_index:])

            except Exception as ex:
                # If there is any other exception adding to cloudwatch logs ignore that error and
                # continue monitoring the logs. Although the exception reason is logged in the cloudwatch_client.py,
                # logging it again incase exception is coming from one of the other functions.
                print("Failed to upload logs to cloudwatch: {}".format(ex))
                pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cw_log_group_name',
                        help='(string) Cloudwatch log_group_name',
                        type=str)
    parser.add_argument('--cw_log_stream_name',
                        help='(string) Cloudwatch log_stream_name',
                        type=str)
    parser.add_argument('--log_symlink_file_path',
                        help='(string) File path of the file that has to monitored and logged to cloudwatch',
                        type=str)

    args = parser.parse_args()
    log_group_name = args.cw_log_group_name
    log_stream_name = args.cw_log_stream_name
    log_symlink_file_path = args.log_symlink_file_path

    print("Started monitoring the logs for {}".format(log_symlink_file_path))
    cw_uploader = CloudWatchUploader(log_group_name, log_stream_name, log_symlink_file_path)
    cw_uploader.run()
    print("Finished monitoring the logs for {}".format(log_symlink_file_path))


if __name__ == '__main__':
    main()
