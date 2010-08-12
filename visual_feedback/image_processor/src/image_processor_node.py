#!/usr/bin/env python
import roslib
import sys
roslib.load_manifest("image_processor")
import rospy
import cv
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from stereo_click.msg import ClickPoint
from stereo_click.srv import *
from geometry_msgs.msg import PointStamped
from image_processor.srv import *
import topic_utils


class ImageProcessor:
    def __init__(self):
        self.name = rospy.get_name()
        self.mono_service_name = "%s/process_mono"%self.name
        self.stereo_service_name = "%s/process_stereo"%self.name
        self.annotation_topic = "%s/annotated"%self.name
        mono_converter = rospy.get_param("~mono_converter","convert_mono_node") #Outbound
        stereo_converter = rospy.get_param("~stereo_converter","convert_stereo_node") #Outbound
        self.convert_mono_service = "%s/convert"%mono_converter
        self.convert_stereo_service = "%s/convert"%stereo_converter
        self.bridge = CvBridge()
        self.anno_pub = rospy.Publisher(self.annotation_topic,Image)
        self.left_anno_pub = rospy.Publisher("%s_left"%self.annotation_topic,Image)
        self.right_anno_pub = rospy.Publisher("%s_right"%self.annotation_topic,Image)
        self.mono_service = rospy.Service(self.mono_service_name,ProcessMono,self.process_mono)
        self.stereo_service = rospy.Service(self.stereo_service_name,ProcessStereo,self.process_stereo)
        self.init_extended()
        
    def init_extended(self):
        #Do nothing
        return
    
    def process_mono(self,req):
        image_topic = "/%s/image_rect_color"%req.camera
        info_topic = "/%s/camera_info"%req.camera
        image = topic_utils.get_next_message(image_topic,Image)
        info = topic_utils.get_next_message(info_topic,CameraInfo)
        (click_points,params,param_names,image_annotated) = self.unpack_and_process(image,info)
        #Publish annotated image stream
        self.anno_pub.publish(image_annotated)
        #Compute 3d points
        pts3d = []
        try:
            convert_to_3d = rospy.ServiceProxy(self.convert_mono_service,ConvertPoint)
            pts3d = [convert_to_3d(pt).pt3d for pt in click_points]
        except rospy.ServiceException,e:
            rospy.loginfo("Service Call Failed: %s"%e)
        return ProcessMonoResponse(pts3d=pts3d,params=params,param_names=param_names,image_annotated=image_annotated)
            
    def process_stereo(self,req):
        image_topic_left = "/%s/left/image_rect_color"%req.camera
        info_topic_left = "%s/left/camera_info"%req.camera
        image_topic_right = "/%s/right/image_rect_color"%req.camera
        info_topic_right = "%s/right/camera_info"%req.camera
        image_left = topic_utils.get_next_message(image_topic_left,Image)
        info_left = topic_utils.get_next_message(info_topic_left,CameraInfo)
        image_right = topic_utils.get_next_message(image_topic_right,Image)
        info_right = topic_utils.get_next_message(info_topic_right,CameraInfo)
        (click_points_left,params,param_names,image_annotated_left) = self.unpack_and_process(image_left,info_left)
        (click_points_right,params,param_names,image_annotated_right) = self.unpack_and_process(image_right,info_right)
        #Publish annotated image stream
        self.left_anno_pub.publish(image_annotated_left)
        self.right_anno_pub.publish(image_annotated_right)
        #Compute 3d points
        try:
            convert_to_3d = rospy.ServiceProxy(self.convert_stereo_service,ConvertPoints)
            pts3d = [convert_to_3d(click_points_left[i],click_points_right[i]).pt3d for i in range(len(click_points_left))]
        except rospy.ServiceException,e:
            rospy.loginfo("Service Call Failed: %s"%e)
        return ProcessStereoResponse(pts3d=pts3d,params=params,param_names=param_names,image_annotated_left=image_annotated_left,image_annotated_right=image_annotated_right)
    
    
    def unpack_and_process(self,image,info):
        try:
            cv_image_mat = self.bridge.imgmsg_to_cv(image, "bgr8")
        except CvBridgeError, e:
            print "CVERROR converting from ImageMessage to cv IplImage"
        cv_image = cv.CreateImage(cv.GetSize(cv_image_mat),8,3)
        cv.Copy(cv_image_mat,cv_image)
        (pts2d,params_dict,cv_image_annotated) = self.process(cv_image,info)
        #Convert annotation to Image.msg
        try:
            image_annotated = self.bridge.cv_to_imgmsg(cv_image_annotated, "bgr8")
        except CvBridgeError, e:
            print "CVERROR converting from cv IplImage to ImageMessage"
        #Convert points2d to clickpoints
        click_points = [ClickPoint(x=x,y=y,camera_info=info) for (x,y) in pts2d]
        params = params_dict.values()
        param_names = params_dict.keys()
        return (click_points,params,param_names,image_annotated)
        
    def process(self,cv_image,info):
        abstract
        return (pts2d,params_dict,cv_image_annotated)
        
    
def main(args):
    rospy.init_node("image_processor_node")
    ipn = ImageProcessor()
    rospy.spin()
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
