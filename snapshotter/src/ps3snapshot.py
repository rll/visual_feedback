#!/usr/bin/env python
import roslib
roslib.load_manifest('snapshotter')
import rospy
import joy
from joy.msg import Joy
from snapshotter.srv import TakeSnapshot

lasttime = rospy.Time(0)
def joy_callback(playstation):
    global lasttime, srv
    if rospy.Time.now() - lasttime < rospy.Duration(1): return
    for x in playstation.buttons:
        if x == 1:
            print "A Button!"
            lasttime = rospy.Time.now()
            srv()
            break


def main():
    global srv
    rospy.loginfo("Waiting for service: %s"%"/snapshotter/take_snapshot")
    rospy.wait_for_service("/snapshotter/take_snapshot")
    srv = rospy.ServiceProxy("/snapshotter/take_snapshot", TakeSnapshot)
    rospy.init_node('ps3snapshotter', anonymous=True)
    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
