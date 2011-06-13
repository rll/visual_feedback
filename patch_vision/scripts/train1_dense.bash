#!/bin/bash
FEAT=$1
PATCH_SIZE=$2
STEP_SIZE=$3
TOP_DIR=/home/stephen/feature_test
IMAGE_DIR=$TOP_DIR/test_images
FEAT_DIR=$TOP_DIR/features/$FEAT/p${PATCH_SIZE}s${STEP_SIZE}
for img in $IMAGE_DIR/*;
do
  rosrun patch_vision make_featuremap.py -i $img -f $FEAT -p $PATCH_SIZE -s $STEP_SIZE -d $FEAT_DIR -v
done
