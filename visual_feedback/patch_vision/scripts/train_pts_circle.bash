#!/bin/bash
FEAT=$1
PATCH_SIZE=$2
TOP_DIR=/home/stephen/feature_test
IMAGE_DIR=$TOP_DIR/images
FEAT_DIR=$TOP_DIR/features/$FEAT/dPOINTS_CIRCLEp${PATCH_SIZE}
POINTS_DIR=$TOP_DIR/points
for img in $IMAGE_DIR/*.JPG;
do
  prefix=`basename $img .JPG`
  PT_FILE=${POINTS_DIR}/${prefix}.pts
  rosrun patch_vision make_featuremap.py -i $img -f $FEAT -D POINTS_CIRCLE -p $PATCH_SIZE -d $FEAT_DIR -P $PT_FILE
done
