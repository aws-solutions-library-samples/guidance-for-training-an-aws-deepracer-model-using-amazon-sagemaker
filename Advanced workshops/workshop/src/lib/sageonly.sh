#!/bin/bash

echo "Starting sageonly.sh"

set -e

COACH_EXP_NAME=sagemaker_rl

# ros melodic
export ROS_DISTRO=melodic
export PYTHONUNBUFFERED=1
export XAUTHORITY=/root/.Xauthority

IP_ADDRESSES=$(hostname -I)
echo "HOSTNAME -I ${IP_ADDRESSES}"

# Set space as the delimiter
IFS=' '
# Read the split words into an array based on space delimiter
read -a IPS_ADDRESS_LIST <<< "$IP_ADDRESSES"
unset IFS
export ROS_IP=${IPS_ADDRESS_LIST[0]}

export DEEPRACER_JOB_TYPE_ENV="SAGEONLY"

export PYTHONPATH=/opt/amazon/install/sagemaker_rl_agent/lib/python3.6/site-packages/:$PYTHONPATH

echo "Logging sagemaker and robomaker in seperate cloudwatch stream"
export SIMULATION_LOG_GROUP=/aws/deepracer/${JOB_TYPE}/SimulationJobs
export TRAINING_LOG_GROUP=/aws/deepracer/${JOB_TYPE}/TrainingJobs
python3.6 /opt/ml/code/scripts/cloudwatch_uploader.py --cw_log_group_name ${SIMULATION_LOG_GROUP} --cw_log_stream_name ${JOB_NAME} --log_symlink_file_path /opt/ml/simapp.log &
python3.6 /opt/ml/code/scripts/cloudwatch_uploader.py --cw_log_group_name ${TRAINING_LOG_GROUP} --cw_log_stream_name ${JOB_NAME} --log_symlink_file_path /opt/ml/training.log &

# Starting sagemaker instance inside conda environment
export PATH="/root/anaconda/bin:/root/anaconda/condabin:/root/anaconda/bin:/opt/ml:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
conda init bash
source /root/anaconda/etc/profile.d/conda.sh
source /root/.bashrc
conda activate sagemaker_env
# Start the redis server and Coach training worker
redis-server /etc/redis/redis.conf &
sleep 5 
echo "Running training job inside conda on single sagemaker instance..."
echo "Input argument to training worker $@"
# redirect stderr to stdout and have error messages sent to the same file as standard output
python3.6 /opt/amazon/install/sagemaker_rl_agent/lib/python3.6/site-packages/markov/training_worker.py $@ > /opt/ml/training.log 2>&1 &
conda deactivate

export PATH="/opt/ml/:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
if which x11vnc &>/dev/null; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    source /opt/amazon/install/setup.bash
    export DISPLAY=:0 # Select screen 0 by default.
    export GAZEBO_MODEL_PATH=./deepracer_simulation_environment/share/deepracer_simulation_environment
    xvfb-run -f $XAUTHORITY -l -n 0 -s ":0 -screen 0 1400x900x24" jwm &
    echo "Running simulation job on single sagemaker instance..."
    echo "Check ${SIMULATION_LOG_GROUP} and ${TRAINING_LOG_GROUP} for training and simulation logs."
    # redirect stderr to stdout and have error messages sent to the same file as standard output
    roslaunch deepracer_simulation_environment $SIMULATION_LAUNCH_FILE > /opt/ml/simapp.log 2>&1
fi

echo "Uploading ROS Gazebo Logs"
# || true allows the upload to continue even if some of source directory doesn't exist or upload failed
aws s3 cp /opt/ml/simapp.log s3://${S3_ROS_LOG_BUCKET}/${JOB_NAME}/ || true
aws s3 cp /opt/ml/training.log s3://${S3_ROS_LOG_BUCKET}/${JOB_NAME}/ || true
aws s3 sync /root/.ros/log/ s3://${S3_ROS_LOG_BUCKET}/${JOB_NAME}/ros || true
aws s3 sync /root/.gazebo/ s3://${S3_ROS_LOG_BUCKET}/${JOB_NAME}/gazebo || true

echo "Terminating with error"
exit 1
