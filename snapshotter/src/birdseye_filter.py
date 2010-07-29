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
        self.width = rospy.get_param("~width",0.024)
        self.height = rospy.get_param("~height",0.024)
    
    def image_filter(self,cv_image,info,copy=None):
        return copy
        (corners,corners_cv,model) = self.detect(cv_image,self.cols,self.rows,self.width,self.height)
        #cv.DrawChessboardCorners(cv_image,(self.cols,self.rows),corners,1)
        intrinsics = self.intrinsic_matrix_from_info(info)
        dist_coeff = self.dist_coeff_from_info(info)
        rot = cv.CreateMat(3, 1, cv.CV_32FC1)
        trans = cv.CreateMat(3, 1, cv.CV_32FC1)
        cv.FindExtrinsicCameraParams2(model,corners_cv,intrinsics,dist_coeff,rot, trans)
        print rot
        print trans
        H = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.GetPerspectiveTransform(model,corners_cv,H)
        
        #cv.WarpPerspective(cv_image,copy,H)
        return copy
        
    def intrinsic_matrix_from_info(self, cam_info):
       intrinsic_matrix = cv.CreateMat(3, 3, cv.CV_32FC1)

       #Because we only want the upper 3x3 (normal) portion of the rectified intrinsic matrix
       for i in range(0, 3):
         for j in range(0, 3):
           intrinsic_matrix[i, j] = cam_info.P[4*i+j]
       return intrinsic_matrix
       
    def dist_coeff_from_info(self,cam_info):
        dist_coeff = cv.CreateMat(1, 4, cv.CV_32FC1)
        cv.Set(dist_coeff,0.0)
        return dist_coeff
        
    def get_board_corners(self, corners, corners_x, corners_y):
        return (corners[0], corners[corners_x    - 1], 
                corners[(corners_y - 1) * corners_x], corners[len(corners) - 1])

    def detect(self, image, corners_x, corners_y, spacing_x, spacing_y, width_scaling=1.0, height_scaling=1.0):
        #resize the image base on the scaling parameters we've been configured with
        scaled_width = int(.5 + image.width * width_scaling)
        scaled_height = int(.5 + image.height * height_scaling)
        
        #in cvMat its row, col so height comes before width
        image_scaled = cv.CreateMat(scaled_height, scaled_width, cv.GetElemType(image))
        cv.Resize(image, image_scaled, cv.CV_INTER_LINEAR)

        #Here, we'll actually call the openCV detector        
        found, corners = cv.FindChessboardCorners(image_scaled, (corners_x, corners_y), cv.CV_CALIB_CB_ADAPTIVE_THRESH)

        if found:
            board_corners = self.get_board_corners(corners, corners_x, corners_y)
            
            #find the perimeter of the checkerboard
            perimeter = 0.0
            for i in range(len(board_corners)):
                next = (i + 1) % 4
                xdiff = board_corners[i][0] - board_corners[next][0]
                ydiff = board_corners[i][1] - board_corners[next][1]
                perimeter += math.sqrt(xdiff * xdiff + ydiff * ydiff)

            #estimate the square size in pixels
            square_size = perimeter / ((corners_x - 1 + corners_y - 1) * 2)
            radius = int(square_size * 0.5 + 0.5)

            corners = cv.FindCornerSubPix(image_scaled, corners, (radius, radius), (-1, -1), (cv.CV_TERMCRIT_EPS + cv.CV_TERMCRIT_ITER, 30, 0.1))

            #uncomment to debug chessboard detection
            print 'Chessboard found'
            #cv.DrawChessboardCorners(image_scaled, (corners_x, corners_y), corners, 1)
            #cv.NamedWindow("image_scaled")
            #cv.ShowImage("image_scaled", image_scaled)
            #cv.WaitKey(600)

            object_points = None

            #we'll also generate the object points if the user has specified spacing
            if spacing_x != None and spacing_y != None:
                object_points = cv.CreateMat(3, corners_x * corners_y, cv.CV_32FC1)

                for y in range(corners_y):
                    for x in range(corners_x):
                        cv.SetReal2D(object_points, 0, y*corners_x + x, x * spacing_x)
                        cv.SetReal2D(object_points, 1, y*corners_x + x, y * spacing_y)
                        cv.SetReal2D(object_points, 2, y*corners_x + x, 0.0)

            #not sure why opencv functions return non opencv compatible datatypes... but they do so we'll convert
            corners_cv = cv.CreateMat(2, corners_x * corners_y, cv.CV_32FC1)
            for i in range(corners_x * corners_y):
                cv.SetReal2D(corners_cv, 0, i, corners[i][0])
                cv.SetReal2D(corners_cv, 1, i, corners[i][1])

            return (corners, corners_cv,object_points)

        else:
            #cv.NamedWindow("image_scaled")
            #cv.ShowImage("image_scaled", image_scaled)
            #cv.WaitKey(600)
            rospy.logwarn("Didn't find checkerboard")
            return (None, None)
    
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
