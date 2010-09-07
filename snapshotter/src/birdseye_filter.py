#!/usr/bin/env python
import roslib
import sys
roslib.load_manifest("snapshotter")
import rospy
import math
import tf
from tf.msg import tfMessage
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from snapshotter.srv import *
from snapshotter.msg import *
from snapshot_filter import SnapshotFilter

class BirdseyeFilter(SnapshotFilter):
    
    def get_extended_params(self):
        self.cols = rospy.get_param("~cols",5)
        self.rows = rospy.get_param("~rows",4)
        self.height = rospy.get_param("~init_height",-70)
        #Mode can be "default" "save_h" and "load_h"
        self.mode = rospy.get_param("~mode","default")
        self.matrix_location = rospy.get_param("~matrix_location","H.yaml")
    
    def image_filter(self,cv_image,info,copy=None):
        image = cv_image


        #Only works on a grayscale image
        gray_image = cv.CreateImage(cv.GetSize(image),8,1)
        cv.CvtColor(image, gray_image, cv.CV_BGR2GRAY)
        print "Called with mode: %s"%self.mode
        if self.mode == "default" or self.mode=="save_h":
            print "Computing homography matrix from checkerboard"
            #Get the width and height of the board
            board_w = self.cols
            board_h = self.rows
            #Area of the board = "board_n"
            board_n  = board_w * board_h
            board_sz = (board_w,board_h)
            #This needs to be fed with a "height", so it knows how high up the perspective transform should be. 
            #I've found for the wide_stereo cameras, a value of -15 works well. For the prosilica, -40. Don't ask me why
            init_height = self.height                
            #Uses openCV to find the checkerboard
            (found,corners) = cv.FindChessboardCorners(image,board_sz, (cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_FILTER_QUADS))
            if(not found):
                print "Couldn't aquire checkerboard, only found 0 of %d corners\n"%board_n
                gr = CloneImage(image)
                cv.CvtColor(gray_image,gr,cv.CV_GRAY2BGR)
                return gr
            #We need subpixel accuracy, so we tell it where the corners are and it magically does the rest. I forget what (11,11) and (-1,-1) mean.
            cv.FindCornerSubPix(gray_image, corners,    
                  (11,11),(-1,-1),   
                  ( cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER, 30, 0.1 ))
            #Pull out the Image Points (3d location of the checkerboard corners, in the camera frame)
            #and the Object Points (2d location of the corners on the checkerboard object) 
            objPts = point_array(4)
            imgPts = point_array(4)
            objPts[0] = (0,0)
            objPts[1] = (board_w-1,0)
            objPts[2] = (0,board_h-1)
            objPts[3] = (board_w-1,board_h-1)
            imgPts[0] = corners[0]
            imgPts[1] = corners[board_w-1]
            imgPts[2] = corners[(board_h-1)*board_w]
            imgPts[3] = corners[(board_h-1)*board_w + board_w - 1]
            
            #Use GetPerspectiveTransform to populate our Homography matrix
            H = cv.CreateMat( 3, 3, cv.CV_32FC1)
            cv.GetPerspectiveTransform(objPts,imgPts,H)
            #Since we don't get any z information from this, we populate H[2,2] with our hard-coded height
            H[2,2] = init_height
            if self.mode == "save_h":
                print "Saving Homography matrix to %s"%self.matrix_location
                cv.Save(self.matrix_location,H)
        else:
            print "Loading Homography matrix from %s"%self.matrix_location
            H = cv.Load(self.matrix_location)
        birds_image = CloneImage(image)
        #birds_image = cv.CreateImage((image.width*3,image.height*3),8,3)
        #Uses the homography matrix to warp the perspective.
        cv.WarpPerspective(image,birds_image,H,  
            cv.CV_INTER_LINEAR+cv.CV_WARP_INVERSE_MAP+cv.CV_WARP_FILL_OUTLIERS )
        #Note: If you need to undo the transformation, you can simply invert H and call cv.WarpPerspective again.
        return birds_image
        


def point_array(length):
    lst = []
    for i in range(length):
        lst.append((0,0))
    return lst
    
def CloneImage(image):
    new_image = cv.CreateImage(cv.GetSize(image),8,3)
    cv.Copy(image,new_image)
    return new_image
    
def GetSize(image):
    return (image.width,image.height)

## Instantiate a new snapshotter node
def main(args):
    rospy.init_node("birdseye_filter")
    filt = BirdseyeFilter()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
