#!/bin/bash
FEAT=$1
DETECTOR=$2
TOP_DIR=/home/stephen/feature_test
IMAGE_DIR=$TOP_DIR/images
FEAT_DIR=$TOP_DIR/features/$FEAT/d${DETECTOR}
for img in $IMAGE_DIR/*.JPG;
do
  rosrun patch_vision make_featuremap.py -i $img -f $FEAT -d $FEAT_DIR -D $DETECTOR -p 100
done
