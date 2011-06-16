#!/bin/bash
FEAT=$1
PATCH_SIZE=$2
STEP_SIZE=$3
TOP_DIR=/home/stephen/feature_test
IMAGE_DIR=$TOP_DIR/images
FEAT_DIR=$TOP_DIR/features/$FEAT/dCIRCLEp${PATCH_SIZE}s${STEP_SIZE}
for img in $IMAGE_DIR/*;
do
  rosrun patch_vision make_featuremap.py -i $img -f $FEAT -D DENSE_CIRCLE -p $PATCH_SIZE -s $STEP_SIZE -d $FEAT_DIR
done
