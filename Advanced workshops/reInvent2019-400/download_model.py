import argparse
from datetime import date
import os
from pathlib import Path
from multiprocessing import Pool, TimeoutError


def get_date_prefix():
    today = date.today()
    return today.strftime("%b_%d").lower()


def download_and_tar(args):
    s3_bucket, job, checkpoint, model_folder_path = args
    os.system(f"aws s3 cp s3://{s3_bucket}/{job}/model/model_{checkpoint}.pb {model_folder_path}/model_{checkpoint}.pb")
    os.system(f"aws s3 cp s3://{s3_bucket}/{job}/model/model_metadata.json {model_folder_path}/model_metadata.json")
    os.system(f"tar -cvzf {model_folder_path}.tar.gz {model_folder_path}")


def main():
    parser = argparse.ArgumentParser(description='Download DeepRacer models and upload them to the car')
    parser.add_argument('--s3-bucket', type=str, help='S3 bucket', default="sagemaker-us-west-2-609956480270")
    parser.add_argument('--jobs', type=str, help='Job IDs. Separate with commas if models need to be downloaded from multiple jobs.')
    parser.add_argument('--checkpoints', type=str, help='Checkpoints to be downloaded. Separate with commas if multiple checkpoints need to be downloaded for each of the jobs.')
    parser.add_argument('--car-ip', type=str, help='IP address of the car. Make sure to setup passwordless SSH first')
    
    args = parser.parse_args()
    date_prefix = get_date_prefix()
    s3_bucket = args.s3_bucket
    jobs = args.jobs.split(',')
    checkpoints = args.checkpoints.split(',')
    car_ip = args.car_ip

    tar_gz_files = []
    work = []

    pool = Pool(processes=4)

    for checkpoint in checkpoints:
        for job in jobs:
            model_folder = Path(".") / Path("%s_%s_%s" % (date_prefix, job, str(checkpoint)))
            if not model_folder.is_dir():
                model_folder.mkdir()
            model_folder_path = model_folder.as_posix()
            work.append((s3_bucket, job, checkpoint, model_folder_path))
            tar_gz_files.append(f"{model_folder_path}.tar.gz")

    pool.map(download_and_tar, work)
    print("Created the following tar gz files.", tar_gz_files)
    print("Trying to upload them now.")

    for model in tar_gz_files:
        os.system(f"scp {model} deepracer@{car_ip}:/opt/aws/deepracer/artifacts/")
        os.system(f"ssh deepracer@{car_ip} cd /opt/aws/deepracer/artifacts/ && tar -xvzf {model}")
        os.system(f"ssh deepracer@{car_ip} cd /opt/aws/deepracer/artifacts/ && rm {model}")

        
if __name__ == '__main__':
    main()