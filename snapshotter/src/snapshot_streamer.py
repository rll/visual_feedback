#!/usr/bin/env python

##    @package snapshotter

import roslib
import sys
roslib.load_manifest("snapshotter")
import rospy
import math
from sensor_msgs.msg import Image
from snapshotter.msg import *


class SnapshotStreamer:

    ##   The constructor
    #    @param self The object pointer
    def __init__(self):
        self.name = rospy.get_name()
        self.input_topic = rospy.get_param("~input","%s/input"%self.name)
        self.output_topic = rospy.get_param("~output","%s/output"%self.name)
        self.input_sub = rospy.Subscriber(self.input_topic,Snapshot,self.handle_input)
        self.output_pub = rospy.Publisher(self.output_topic,Image)
        
    #Takes a snapshot as input, outputs its image
    def handle_input(self,snapshot):
        rospy.loginfo("RECEIVED INPUT!")
        self.output_pub.publish(snapshot.image)
        
        
## Instantiate a new streamer node
def main(args):
    rospy.init_node("shapshot_saver")
    saver = SnapshotStreamer()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
