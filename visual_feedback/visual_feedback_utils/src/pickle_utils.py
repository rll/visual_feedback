#!/usr/bin/env python
import roslib
roslib.load_manifest("visual_feedback_utils")
import pickle
from sensor_msgs.msg import Image,CameraInfo
import rospy

class InfoPickle:
    def __init__(self,camera_info):
        self.header = HeaderPickle(camera_info.header)
        self.roi = RoiPickle(camera_info.roi)
        self.height = camera_info.height
        self.width = camera_info.width
        self.D = camera_info.D
        self.K = camera_info.K
        self.R = camera_info.R
        self.P = camera_info.P
        
class RoiPickle:
    def __init__(self,roi):
        self.x_offset = roi.x_offset
        self.y_offset = roi.y_offset
        self.height = roi.height
        self.width = roi.width
        
class HeaderPickle:
    def __init__(self,header):
        self.stamp = None
        self.seq = header.seq
        self.frame_id = header.frame_id
        
def info_to_pickle(info):
    return InfoPickle(camera_info)
    
def pickle_to_info(info_pickle):
    info = CameraInfo()
    info.header.stamp = rospy.Time()
    info.header.seq = info_pickle.header.seq
    info.header.frame_id = info_pickle.header.frame_id
    info.roi.x_offset = info_pickle.roi.x_offset
    info.roi.y_offset = info_pickle.roi.y_offset
    info.roi.height = info_pickle.roi.height
    info.roi.width = info_pickle.roi.width
    info.height = info_pickle.height
    info.width = info_pickle.width
    info.D = info_pickle.D
    info.K = info_pickle.K
    info.R = info_pickle.R
    info.P = info_pickle.P
    
    return info
    
def dump_info(info,filepath):
    info_pickle = info_to_pickle(info)
    pickle_file = open(filepath,'w')
    pickle.dump(info_pickle,pickle_file)

def load_info(filepath):
    pickle_file = open(filepath)
    info_pickle = pickle.load(pickle_file)
    info = pickle_to_info(info_pickle)
    return info
    
        

