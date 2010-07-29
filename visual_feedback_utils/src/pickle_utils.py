#!/usr/bin/env python

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
        

