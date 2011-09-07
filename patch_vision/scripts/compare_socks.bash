#!/bin/bash
COMP=$1
REF=$2
FEAT=$3
DETAILS=$4
TOP_DIR=/home/stephen/feature_test
IMAGE_DIR=$TOP_DIR/images
FEAT_DIR=$TOP_DIR/features/$FEAT/$DETAILS
rosrun patch_vision feature_comparer.py -r $IMAGE_DIR/Image${REF}.jpg -rf $FEAT_DIR/Image${REF}.fm -c $IMAGE_DIR/Image${COMP}.jpg -cf $FEAT_DIR/Image${COMP}.fm -rz 4 -cz 4
