#!/usr/bin/env python
import roslib
roslib.load_manifest("visual_feedback_utils")
import pickle
from sensor_msgs.msg import Image,CameraInfo
import rospy
from dynamic_reconfigure.client import Client as ReconfigureClient

def configure_prosilica():
    print "Configuring prosilica"
    client = ReconfigureClient("prosilica_driver")
    client.update_configuration ({'auto_exposure':False,'exposure':0.4,'auto_gain':False,'gain':0.0,
        'auto_whitebalance':False,'whitebalance_red':114,'whitebalance_blue':300})
        
def main(args):
    rospy.init_node("prosilica_configurer")
    configure_prosilica()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
    
        

