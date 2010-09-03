#!/usr/bin/env python
import roslib
roslib.load_manifest("visual_feedback_utils")
import rospy
import time

def get_next_message(topic,message_type,timeout=None):
    responses = []
    sub = rospy.Subscriber(topic,message_type,lambda msg: responses.append(msg))
    start = rospy.Time.now()
    if timeout:
        finish = rospy.Time.now() + rospy.Duration(timeout)
    while len(responses) == 0:
        rospy.sleep(0.1)
        if timeout:
            if rospy.Time.now() >= finish:
                responses.append(None)     
    sub.unregister()
    output = responses[0]
    for val in list(responses):
        responses.remove(val)
    return output
    
    
def call_and_response(output_topic,output_type,output_message,input_topic,input_type,timeout=None):
    pub = rospy.Publisher(output_topic,output_type)
    pub.publish(output_message)
    return get_next_message(input_topic,input_type,timeout)
