#!/bin/bash
set -e
set -x

echo "Cleaning up local generated files"

cd /opt/ml/code
rm -rf EXCEPTION_HANDLER_SYNC_FILE
rm -rf dump.rdb
rm -rf checkpoint_sagemaker/ custom_files/ frozen_models/ renamed_checkpoint/

rm -rf /opt/ml/model/*
