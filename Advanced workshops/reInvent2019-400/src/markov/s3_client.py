import boto3
import botocore
import os
import io
import json
import time
import sys

from google.protobuf import text_format
from tensorflow.python.training.checkpoint_state_pb2 import CheckpointState

import logging
logging.basicConfig(level=logging.INFO)

logger = logging.getLogger("SageS3Client")


class SageS3Client():
    def __init__(self, bucket=None, s3_prefix=None, aws_region=None):
        self.aws_region = aws_region
        self.bucket = bucket
        self.s3_prefix = s3_prefix
        self.config_key = os.path.normpath(s3_prefix + "/ip/ip.json")
        self.markov_prefix = os.path.normpath(s3_prefix + "/markov")
        self.hyperparameters_key = os.path.normpath(s3_prefix + "/ip/hyperparameters.json")
        self.done_file_key = os.path.normpath(s3_prefix + "/ip/done")
        self.model_checkpoints_prefix = os.path.normpath(s3_prefix + "/model/") + "/"
        self.lock_file = ".lock"
        logger.info("Initializing SageS3Client...")

    def get_client(self):
        session = boto3.session.Session()
        return session.client('s3', region_name=self.aws_region)

    def _get_s3_key(self, key):
        return os.path.normpath(self.model_checkpoints_prefix + "/" + key)

    def download_markov(self):
        s3_client = self.get_client()
        response = s3_client.list_objects_v2(Bucket=self.bucket,
                                             Prefix=self.markov_prefix)
        if "Contents" in response:
            for i in response["Contents"]:
                if ".ipynb_checkpoints" in i["Key"]:
                    continue
                s3_client.download_file(Bucket=self.bucket,
                                        Key=i["Key"],
                                        Filename=i["Key"].replace(self.markov_prefix,"./custom_files/markov"))
                logger.info("Downloaded %s" % i["Key"])

    def write_ip_config(self, ip):
        s3_client = self.get_client()
        data = {"IP": ip}
        json_blob = json.dumps(data)
        file_handle = io.BytesIO(json_blob.encode())
        file_handle_done = io.BytesIO(b'done')
        s3_client.upload_fileobj(file_handle, self.bucket, self.config_key)
        s3_client.upload_fileobj(file_handle_done, self.bucket, self.done_file_key)

    def upload_hyperparameters(self, hyperparams_json):
        s3_client = self.get_client()
        file_handle = io.BytesIO(hyperparams_json.encode())
        s3_client.upload_fileobj(file_handle, self.bucket, self.hyperparameters_key)

    def upload_model(self, checkpoint_dir):
        s3_client = self.get_client()
        num_files = 0
        for root, dirs, files in os.walk("./" + checkpoint_dir):
            for filename in files:
                abs_name = os.path.abspath(os.path.join(root, filename))
                s3_client.upload_file(abs_name,
                                      self.bucket,
                                      "%s/%s/%s" % (self.s3_prefix, checkpoint_dir, filename))
                num_files += 1

    def download_model(self, checkpoint_dir):
        s3_client = self.get_client()
        filename = "None"
        try:
            filename = os.path.abspath(os.path.join(checkpoint_dir, "checkpoint"))
            if not os.path.exists(checkpoint_dir):
                os.makedirs(checkpoint_dir)

            while True:
                response = s3_client.list_objects_v2(Bucket=self.bucket,
                                                     Prefix=self._get_s3_key(self.lock_file))

                if "Contents" not in response:
                    # If no lock is found, try getting the checkpoint
                    try:
                        s3_client.download_file(Bucket=self.bucket,
                                                Key=self._get_s3_key("checkpoint"),
                                                Filename=filename)
                    except Exception as e:
                        time.sleep(2)
                        continue
                else:
                    time.sleep(2)
                    continue

                ckpt = CheckpointState()
                if os.path.exists(filename):
                    contents = open(filename, 'r').read()
                    text_format.Merge(contents, ckpt)
                    rel_path = ckpt.model_checkpoint_path
                    checkpoint = int(rel_path.split('_Step')[0])

                    response = s3_client.list_objects_v2(Bucket=self.bucket,
                                                         Prefix=self._get_s3_key(rel_path))
                    if "Contents" in response:
                        num_files = 0
                        for obj in response["Contents"]:
                            filename = os.path.abspath(os.path.join(checkpoint_dir,
                                                                    obj["Key"].replace(self.model_checkpoints_prefix,
                                                                                       "")))
                            s3_client.download_file(Bucket=self.bucket,
                                                    Key=obj["Key"],
                                                    Filename=filename)
                            num_files += 1
                        return True

        except Exception as e:
            logger.error("{} while downloading the model {} from S3".format(e, filename))
            return False

    def get_ip(self):
        s3_client = self.get_client()
        self._wait_for_ip_upload()
        try:
            s3_client.download_file(self.bucket, self.config_key, 'ip.json')
            with open("ip.json") as f:
                ip = json.load(f)["IP"]
            return ip
        except Exception as e:
            logger.error("Exception [{}] occured, Cannot fetch IP of redis server running in SageMaker. Job failed!".format(e))
            sys.exit(1)

    def _wait_for_ip_upload(self, timeout=600):
        s3_client = self.get_client()
        time_elapsed = 0
        while True:
            response = s3_client.list_objects(Bucket=self.bucket, Prefix=self.done_file_key)
            if "Contents" not in response:
                time.sleep(1)
                time_elapsed += 1
                if time_elapsed % 5 == 0:
                    logger.info ("Waiting for SageMaker Redis server IP... Time elapsed: %s seconds" % time_elapsed)
                if time_elapsed >= timeout:
                    logger.error("Cannot retrieve IP of redis server running in SageMaker. Job failed!")
                    sys.exit(1)
            else:
                return

    def download_file(self, s3_key, local_path):
        s3_client = self.get_client()
        try:
            s3_client.download_file(self.bucket, s3_key, local_path)
            return True
        except botocore.exceptions.ClientError as e:
            if e.response['Error']['Code'] == "404":
                logger.info("Exception [{}] occured on download file-{} from s3 bucket-{} key-{}".format(e.response['Error'], local_path, self.bucket, s3_key))
                return False
            else:
                logger.error("boto client exception error [{}] occured on download file-{} from s3 bucket-{} key-{}"
                            .format(e.response['Error'], local_path, self.bucket, s3_key))
                return False
        except Exception as e:
            logger.error("Exception [{}] occcured on download file-{} from s3 bucket-{} key-{}".format(e, local_path, self.bucket, s3_key))
            return False

    def upload_file(self, s3_key, local_path):
        s3_client = self.get_client()
        try:
            s3_client.upload_file(Filename=local_path,
                                  Bucket=self.bucket,
                                  Key=s3_key)
            return True
        except Exception as e:
            logger.error("{} on upload file-{} to s3 bucket-{} key-{}".format(e, local_path, self.bucket, s3_key))
            return False


if __name__ == '__main__':

    CUSTOM_FILES_PATH = "./custom_files"
    dirs_to_create = ["./custom_files",
                    "./custom_files/markov",
                    "./custom_files/markov/actions",
                    "./custom_files/markov/presets",
                    "./custom_files/markov/environments",
                    "./custom_files/markov/rewards"
                    ]

    for path in dirs_to_create:
        if not os.path.exists(path):
            os.makedirs(path)
    s3_bucket = os.environ.get("SAGEMAKER_SHARED_S3_BUCKET", "gsaur-test")
    s3_prefix = os.environ.get("SAGEMAKER_SHARED_S3_PREFIX", "sagemaker")
    aws_region = os.environ.get("APP_REGION", "us-east-1")
    s3_client = SageS3Client(bucket=s3_bucket, s3_prefix=s3_prefix, aws_region=aws_region)
    s3_client.download_markov()
    