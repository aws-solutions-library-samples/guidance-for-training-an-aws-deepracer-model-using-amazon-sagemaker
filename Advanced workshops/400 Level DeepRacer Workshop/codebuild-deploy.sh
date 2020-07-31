#!/bin/bash

set -vex
[ -n "${PRIMARY_BUCKET}" ] || exit 1
[ -n "${TEMPLATE_BUCKET}" ] || exit 1
[ -n "${CODEBUILD_GIT_CLEAN_BRANCH}" ] || exit 1
[ -n "${CLOUDFRONT}" ] || exit 1

set +vx

PRIMARY_BUCKET=$(echo $PRIMARY_BUCKET | cut -f1,2 -d.)

if [ "$CODEBUILD_GIT_CLEAN_BRANCH" = "master" ] ; then
  aws s3 sync public/ s3://${PRIMARY_BUCKET}/ --cache-control 'max-age=3600, public' --exclude '*' --include '*.html' --include '*.xml';
  aws s3 sync public/ s3://${PRIMARY_BUCKET}/ --cache-control 'max-age=86400, public' --exclude '*.html' --exclude '*.xml';
  aws cloudfront create-invalidation --distribution-id ${CLOUDFRONT} --paths /\*
else
  export DEPLOY_DIR=${CODEBUILD_GIT_CLEAN_BRANCH};
  aws s3 sync public/ s3://${PRIMARY_BUCKET}/${DEPLOY_DIR}/ --delete --cache-control 'max-age=3600, public' --exclude '*' --include '*.html' --include '*.xml';
  aws s3 sync public/ s3://${PRIMARY_BUCKET}/${DEPLOY_DIR}/ --delete --cache-control 'max-age=86400, public' --exclude '*.html' --exclude '*.xml';
  aws cloudfront create-invalidation --distribution-id ${CLOUDFRONT} --paths /${DEPLOY_DIR}\*
fi
