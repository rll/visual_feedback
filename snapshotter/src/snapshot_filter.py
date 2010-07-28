#!/usr/bin/env python

##    @package snapshotter

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

class SnapshotFilter:

    ##   The constructor
    #    @param self The object pointer
    def __init__(self):
        input_topic = rospy.get_param("~input","defaultFilterInput")
        output_topic = rospy.get_param("~output","defaultFilterOutput")
        self.get_extended_params()
        self.name = rospy.get_name()
        self.bridge = CvBridge()
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.output_pub = rospy.Publisher(self.output_topic,Snapshot)
        self.input_sub = rospy.Subscriber(self.input_topic,Snapshot,self.handle_input)
    
    def get_extended_params(self):
        #Do nothing
        return
    
    #Takes a snapshot as input, returns a filtered snapshot as output
    def handle_input(self,snapshot):
        try:
            input_cv_image = self.bridge.imgmsg_to_cv(snapshot.image, "bgr8")
        except CvBridgeError, e:
            print e
        output_cv_image = self.image_filter(cv_image=input_cv_image,info=snapshot.info)
        output_info = snapshot.info
        try:
            output_image = self.bridge.cv_to_imgmsg(output_cv_image, "bgr8")
        except CvBridgeError, e:
            print e
        filters = snapshot.filters
        filters.append(self.name.replace('/',''))
        output_snapshot = Snapshot(image=output_image,info=output_info,filters=filters)
        self.output_pub.publish(output_snapshot)
        print "%s output"%self.name
        
        
    #Abstract method, returns the altered image and info
    def image_filter(self,cv_image,info):
        abstract
    
## Instantiate a new snapshotter node
def main(args):
    rospy.init_node("shapshot_filter")
    filt = SnapshotFilter()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
