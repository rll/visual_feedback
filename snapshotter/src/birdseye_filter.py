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
    
    def image_filter(self,cv_image,info,copy=None):
        board_w = self.cols
        board_h = self.rows
        board_n  = board_w * board_h
        board_sz = (board_w,board_h)
        intrinsic = self.intrinsic_matrix_from_info(info)
        distortion = self.dist_coeff_from_info(info)
        image = cv_image
        init_height = self.height
        gray_image = cv.CreateImage(cv.GetSize(image),8,1)
        cv.CvtColor(image, gray_image, cv.CV_BGR2GRAY)
        
        #Undistort Image
        mapx = cv.CreateImage( cv.GetSize(image), cv.IPL_DEPTH_32F, 1 )
        mapy = cv.CreateImage( cv.GetSize(image), cv.IPL_DEPTH_32F, 1 )
        cv.InitUndistortMap(  
            intrinsic,  
            distortion,  
            mapx,  
            mapy  
            )
        t = CloneImage(image)
        cv.Remap( t, image, mapx, mapy )
        corner_count = 0
        (found,corners) = cv.FindChessboardCorners(image,board_sz, (cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_FILTER_QUADS))
        if(not found):
            print "Couldn't aquire checkerboard, only found %d of %d corners\n"%(corner_count,board_n)
            return cv_image
        cv.FindCornerSubPix(gray_image, corners,    
              (11,11),(-1,-1),   
              ( cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER, 30, 0.1 ))
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
        H = cv.CreateMat( 3, 3, cv.CV_32FC1)
        print H
        cv.GetPerspectiveTransform(objPts,imgPts,H)
        birds_image = CloneImage(image)
        H[2,2] = init_height
        cv.WarpPerspective(image,birds_image,H,  
            cv.CV_INTER_LINEAR+cv.CV_WARP_INVERSE_MAP+cv.CV_WARP_FILL_OUTLIERS )
        print birds_image
        return birds_image
        
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

            #corners = cv.FindCornerSubPix(image_scaled, corners, (radius, radius), (-1, -1), (cv.CV_TERMCRIT_EPS + cv.CV_TERMCRIT_ITER, 30, 0.1))

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
