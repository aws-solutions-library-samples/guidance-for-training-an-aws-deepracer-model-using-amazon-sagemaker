"""This module implement CloudWatch client"""

import botocore
import time
import logging

from markov.log_handler.constants import (SIMAPP_EVENT_SYSTEM_ERROR,
                                          SIMAPP_EVENT_USER_ERROR,
                                          SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_EVENT_ERROR_CODE_400,
                                          SIMAPP_S3_DATA_STORE_EXCEPTION)
from markov.boto.constants import BotoClientNames
from markov.boto.deepracer_boto_client import DeepRacerBotoClient
from markov.log_handler.logger import Logger

LOG = Logger(__name__, logging.INFO).get_logger()


class CloudWatchClient(DeepRacerBotoClient):
    """CloudWatch Boto Client"""
    name = BotoClientNames.CLOUD_WATCH_LOGS.value

    def __init__(self, region_name="us-east-1", max_retry_attempts=5,
                 backoff_time_sec=1.0, session=None,
                 log_and_cont=False):
        """CloudWatch client

        Args:
            region_name (str): aws region name.
            max_retry_attempts (int): maximum number of retry.
            backoff_time_sec (float): backoff second between each retry.
            session (boto3.Session): An alternative session to use.
                                     Defaults to None.
            log_and_cont (bool, optional): Log the error and continue with the flow.
                                           Defaults to False.
        """
        super(CloudWatchClient, self).__init__(region_name=region_name,
                                               max_retry_attempts=max_retry_attempts,
                                               backoff_time_sec=backoff_time_sec,
                                               boto_client_name=self.name,
                                               session=session)
        self._log_and_cont = log_and_cont
        self._max_retry_attempts = max_retry_attempts

    def _get_cw_client(self):
        """ Get a cloudwatch boto client.

        Raises:
            Exception: the exception raised by creating boto client.

        Returns:
            client: a cloudwatch boto client.
        """
        try:
            return self.get_client()
        except Exception as ex:
            error_msg = "[CloudWatch] Exception: {}".format(ex)
            LOG.error(error_msg)
            raise Exception(error_msg)

    def create_log_group(self, log_group_name):
        """Create log group using the log_group_name.

        Args:
            log_group_name (str): Name for the log group.

        Raises:
            Exception: Excption other than ResourceAlreadyExistsException.
        """
        client = self._get_cw_client()
        try:
            client.create_log_group(logGroupName=log_group_name)
        except client.exceptions.ResourceAlreadyExistsException as ex:
            error_msg = "[CloudWatchClient] ResourceAlreadyExistsException: {}".format(ex)
            LOG.info(error_msg)
            pass
        except Exception as ex:
            error_msg = "[CloudWatchClient] Exception: Unable to create log group {}: {}".format(log_group_name, ex)
            LOG.error(error_msg)
            raise Exception(error_msg)

    def create_log_stream(self, log_group_name, log_stream_name):
        """Create log stream in the log group with log stream name.

        Args:
            log_group_name (str): the log group name to create log stream in.
            log_stream_name (str): the log stream name to create a log stream with.

        Raises:
            Exception: Excption other than ResourceAlreadyExistsException.
        """
        client = self._get_cw_client()
        try:
            client.create_log_stream(logGroupName=log_group_name,
                                     logStreamName=log_stream_name)
        except client.exceptions.ResourceAlreadyExistsException as ex:
            error_msg = "[CloudWatchClient] ResourceAlreadyExistsException: {}".format(ex)
            LOG.info(error_msg)
            pass
        except Exception as ex:
            error_msg = "[CloudWatchClient] Exception: Unable to execute create_log_stream in CloudWatch: {}".format(ex)
            LOG.error(error_msg)
            raise Exception(error_msg)

    def put_log_events(self, log_group_name, log_stream_name, batch, sequence_token):
        """Put log events into the log stream in the log group.

        Args:
            log_group_name (str): The log group to put the log batch in.
            log_stream_name (str): The log stream within the log group to put the log batch in.
            batch (list): list of dictionaries of log events.
            sequence_token (str): the sequence_token to put the log in next.

        Raises:
            Exception: Exception other then DataAlreadyAcceptedException and InvalidSequenceTokenException

        Returns:
            str: the next sequence token to use.
        """
        retry_num = 0
        while retry_num < self._max_retry_attempts:
            retry_num += 1
            client = self._get_cw_client()
            try:
                if sequence_token:
                    response = client.put_log_events(logGroupName=log_group_name, logStreamName=log_stream_name,
                                                     logEvents=batch, sequenceToken=sequence_token)
                else:
                    response = client.put_log_events(logGroupName=log_group_name, logStreamName=log_stream_name,
                                                     logEvents=batch)
                return response["nextSequenceToken"]
            except client.exceptions.DataAlreadyAcceptedException as ex:
                error = "[CloudWatchClient] DataAlreadyAcceptedException put_log_events :{}".format(ex)
                LOG.info(error)
                sequence_token = self._parse_next_sequence_token_from_exception(ex)
                return sequence_token
            except client.exceptions.InvalidSequenceTokenException as ex:
                error = "[CloudWatchClient] InvalidSequenceTokenException put_log_events :{}".format(ex)
                LOG.info(error)
                sequence_token = self._parse_next_sequence_token_from_exception(ex)
            except Exception as ex:
                error = "[CloudWatchClient] Exception put_log_events :{}".format(ex)
                LOG.info(error)
                # Sleep before the retry to avoid throttling
                time.sleep(0.200)

        error_msg = "[CloudWatch] Exception: put_log_events after {} retries; {};{};{}".format(
            self._max_retry_attempts, log_group_name, log_stream_name, batch)
        raise Exception(error_msg)

    def _parse_next_sequence_token_from_exception(self, ex):
        """Parse the next sequence token from exception message.

        Args:
            ex (Exception): the exception being thrown.

        Returns:
            str: the parsed next sequence token.
        """
        parsed_token = None
        if "sequenceToken: " in str(ex):
            # Format for DataAlreadyAcceptedExceptions
            parsed_token = str(ex).split("sequenceToken: ")[1]
        elif "sequenceToken is: " in str(ex):
            # Format for InvalidSequenceTokenException
            parsed_token = str(ex).split("sequenceToken is: ")[1]
        else:
            parsed_token = None

        if parsed_token == "null":
            # This happens when sending log events with a token to
            # a stream that doesn't expect a token.
            parsed_token = None
        return parsed_token
