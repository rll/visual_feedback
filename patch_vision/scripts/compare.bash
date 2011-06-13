#!/bin/bash
COMP=$1
REF=$2
FEAT=$3
DETAILS=$4
TOP_DIR=/home/stephen/feature_test
IMAGE_DIR=$TOP_DIR/images
FEAT_DIR=$TOP_DIR/$FEAT/$DETAILS
rosrun patch_vision feature_comparer.py -r $IMAGE_DIR/DSC_00${REF}.JPG -rf $FEAT_DIR/DSC_00${REF}.fm -c $IMAGE_DIR/DSC_00${COMP}.JPG -cf $FEAT_DIR/DSC_00${COMP}.fm -rz 9 -cz 9
