#!/usr/bin/env python

'''
Get a particular log given stream ID

#https://boto3.amazonaws.com/v1/documentation/api/latest/reference/services/logs.html#CloudWatchLogs.Client.filter_log_events
'''
import boto3
import sys

def get_log_events(log_group, stream_name=None, stream_prefix=None, start_time=None, end_time=None):
    client = boto3.client('logs')
    if stream_name is None and stream_prefix is None:
        print ("both stream name and prefix can't be None")
        return

    kwargs = {
        'logGroupName': log_group,
        'logStreamNames': [stream_name],
        'limit': 10000,
    }

    if stream_prefix:
        kwargs = {
            'logGroupName': log_group,
            'logStreamNamePrefix': stream_prefix,
            'limit': 10000,
        }

    kwargs['startTime'] = start_time
    kwargs['endTime'] = end_time

    while True:
        resp = client.filter_log_events(**kwargs)
        yield from resp['events']
        try:
            kwargs['nextToken'] = resp['nextToken']
        except KeyError:
            break

def download_log(fname, stream_name=None, stream_prefix=None,
            log_group=None, start_time=None, end_time=None):
    if start_time is None:
        start_time = 1451490400000 #2018
    if end_time is None:
        end_time =   2000000000000 #2033 #arbitrary future date
    if log_group is None:
        log_group = "/aws/robomaker/SimulationJobs" 

    with open(fname, 'w') as f:
        logs = get_log_events(
            log_group=log_group,
            stream_name=stream_name,
            stream_prefix=stream_prefix,
            start_time=start_time,
            end_time=end_time
        )
        for event in logs:
            f.write(event['message'].rstrip())
            f.write("\n")

