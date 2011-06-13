#!/bin/bash
FEAT=$1
PATCH_SIZE=$2
STEP_SIZE=$3
TOP_DIR=/home/stephen/feature_test
IMAGE_DIR=$TOP_DIR/images
FEAT_DIR=$TOP_DIR/$FEAT/p${PATCH_SIZE}s${STEP_SIZE}
for img in $IMAGE_DIR/*.JPG;
do
  rosrun patch_vision make_featuremap.py -i $img -f $FEAT -p $PATCH_SIZE -s $STEP_SIZE -d $FEAT_DIR
done
