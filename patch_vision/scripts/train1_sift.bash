#!/bin/bash
FEAT=$1
TOP_DIR=/home/stephen/feature_test
IMAGE_DIR=$TOP_DIR/test_images
FEAT_DIR=$TOP_DIR/features/$FEAT/dSIFT
for img in $IMAGE_DIR/*;
do
  rosrun patch_vision make_featuremap.py -i $img -f $FEAT -d $FEAT_DIR -D SIFT -v
done
