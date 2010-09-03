#!/usr/bin/env python
import roslib
import sys
roslib.load_manifest("image_processor")
import rospy
from numpy import *
import pyflann
import math
import cv
import os.path
import pickle
import Geometry2D
import Vector2D
import tf
from geometry_msgs.msg import PointStamped
from image_processor_node import ImageProcessor
from shape_fitting_utils import *
import image_geometry

SHOW_CONTOURS = False
SHOW_UNSCALED_MODEL = False
SHOW_SCALED_MODEL = False
SHOW_FINAL_MODEL = False
SHOW_POINTS = False
SHOW_ITER = True

INV_CONTOUR = True
CONTOURS_ONLY = False
NEAREST_EDGE = 3.0

class ShapeFitter(ImageProcessor):
    
    def init_extended(self):
        self.config_dir = "%s/config"%os.path.dirname(os.path.dirname( os.path.realpath( __file__ ) ) )
        modelname = rospy.get_param("~model","model")
        self.mode = rospy.get_param("~mode","default")
        corrected_modelpath = "%s/%s_model.pickle"%(self.config_dir,modelname)
        self.threshold = rospy.get_param("~threshold",95)
        self.load_model(corrected_modelpath)
        
    def process(self,cv_image,info,image2=None):
        image_raw = cv_image
        image_gray = cv.CreateImage(cv.GetSize(image_raw),8,1)        
        cv.CvtColor(image_raw,image_gray,cv.CV_RGB2GRAY)
        self.image_gray = image_gray
        self.image_raw = cv.CreateImage(cv.GetSize(image_raw),8,3)
        cv.Copy(image_raw,self.image_raw)
        image_hsv = cv.CreateImage(cv.GetSize(image_raw),8,3)
        cv.CvtColor(image_raw,image_hsv,cv.CV_RGB2HSV)
        self.dist_fxn = l2_norm
        hue = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        sat = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        val = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        trash = cv.CreateImage(cv.GetSize(image_hsv),8,1)
        cv.Split(image_hsv,hue,sat,val,None)
        r = cv.CreateImage(cv.GetSize(image_raw),8,1)
        g = cv.CreateImage(cv.GetSize(image_raw),8,1)
        b = cv.CreateImage(cv.GetSize(image_raw),8,1)
        cv.Split(image_raw,r,g,b,None)
        """
        cv.NamedWindow("Raw")
        cv.NamedWindow("Red")
        cv.NamedWindow("Green")
        cv.NamedWindow("Blue")
        cv.ShowImage("Raw",image_raw)
        cv.ShowImage("Red",r)
        cv.ShowImage("Green",g)
        cv.ShowImage("Blue",b)
        
        cv.WaitKey()
        
        rorb = cv.CreateImage(cv.GetSize(image_raw),8,1)
        """
        self.image = hue
        self.image_sat = sat
        self.image_val = val
        self.image_g = g
        #Do the actual computation
        storage = cv.CreateMemStorage(0)
        
        self.image1 = cv.CloneImage( self.image )
        self.image3 = cv.CloneImage( self.image )
        self.image4 = cv.CloneImage( self.image_gray)
        self.image2 = cv.CloneImage( self.image_raw )
        cv.Threshold( self.image, self.image1, 75, 255, cv.CV_THRESH_BINARY )
        cv.Threshold( self.image, self.image3, 140, 255, cv.CV_THRESH_BINARY_INV )  
        cv.Not(self.image3,self.image3)
        cv.Or(self.image1,self.image3,self.image3)
        
        #and_img = cv.CloneImage( self.image_gray)
        #nand_img = cv.CloneImage( self.image_gray)
        #cv.And(self.image3,self.image4,and_img)
        #cv.Not(and_img,nand_img)

         #Filter out grippers
        cam_frame = info.header.frame_id
        now = rospy.Time.now()
        for link in ("l_gripper_l_finger_tip_link","r_gripper_l_finger_tip_link"):
            self.listener.waitForTransform(cam_frame,link,now,rospy.Duration(10.0))
            l_grip_origin = PointStamped()
            l_grip_origin.header.frame_id = link
            l_grip_in_camera = self.listener.transformPoint(cam_frame,l_grip_origin)
            camera_model = image_geometry.PinholeCameraModel()
            camera_model.fromCameraInfo(info)
            (u,v) = camera_model.project3dToPixel((l_grip_in_camera.point.x,l_grip_in_camera.point.y,l_grip_in_camera.point.z))
            if link[0] == "l":
                x_range = range(0,u)
            else:
                x_range = range(u,self.image3.width)
            if 0 < u < self.image3.width and 0 < v < self.image3.height:
                for x in x_range:
                    for y in range(0,self.image3.height):
                        self.image3[y,x] = 0.0
        #Filter out edges
        
        for i in range(15):
            for j in range(self.image3.height):
                self.image3[j,i] = 0.0
                self.image3[j,self.image3.width-i-1] = 0.0
        contour_reg = cv.FindContours   ( self.image1, storage,
                                    cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        contour_inv = cv.FindContours   ( self.image3, storage,
                                    cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        contour_gray = cv.FindContours   ( self.image4, storage,
                                    cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        
        max_length = 0
        max_contour = None
        if INV_CONTOUR:
            contours = [contour_inv]
        else:
            contours = [contour_reg,contour_gray]
        for contour in contours:
            while contour != None:
                length = area(contour)   
                if length > max_length and not self.image_edge(contour):
                    max_length = length
                    max_contour = contour
                    print "Replaced with %f"%length
                contour = contour.h_next()
        if max_contour == None:
            print "Couldn't find any contours"
            return ([],{},raw_image)
        else:
            print area(max_contour)
        shape_contour = max_contour
        if SHOW_CONTOURS:
            cv.DrawContours(self.image2,shape_contour,cv.CV_RGB(255,0,0),cv.CV_RGB(255,0,0),0,1,8,(0,0))
        if CONTOURS_ONLY:
            cv.ShowImage("Result",self.image2)
            return

        (real_center,real_top,real_theta,real_scale) = self.get_principle_info(shape_contour)
        #self.model = translate_poly(rotate_poly(shape_contour,-0.2,real_center),(500,500)) ##FIXME
        if SHOW_UNSCALED_MODEL:
            self.model.draw_to_image(self.image2,cv.CV_RGB(0,0,255))
        (model_center,model_top,model_theta,model_scale) = self.get_principle_info(self.get_model_contour())
        displ = displacement(model_center,real_center)
        if SHOW_POINTS:
            self.highlight_pt(real_center,cv.CV_RGB(200,200,200))
            self.highlight_pt(real_top,cv.CV_RGB(200,200,200))
        if SHOW_POINTS:
            self.highlight_pt(model_center,cv.CV_RGB(0,0,0))
            self.highlight_pt(model_top,cv.CV_RGB(0,0,0))
        print model_theta
        print real_theta
        angle = model_theta - real_theta
        print angle
        scale = real_scale/float(model_scale)
        model_trans = translate_poly(self.get_model_contour(),displ)
        model_rot = rotate_poly(model_trans,-1*angle,real_center)
        #scale = 1 #FIXME
        model_scaled = scale_poly(model_rot,scale,real_center)
        
        (model_center,model_top,model_theta,model_scale) = self.get_principle_info(model_scaled)
        if SHOW_POINTS:
            self.highlight_pt(model_center,cv.CV_RGB(128,128,128))
            self.highlight_pt(model_top,cv.CV_RGB(128,128,128))
        
        #Do the same to the actual model
        self.model.translate(displ)
        self.model.rotate(-1*angle,real_center)
        self.model.scale(scale,real_center)
        
 
        if SHOW_SCALED_MODEL:
            self.model.draw_to_image(self.image2,cv.CV_RGB(0,0,255))
  
        #Energy calculation
        print "Energy is: %f"%self.energy_fxn(self.model,shape_contour)
        print "Shape contour has %d points"%(len(shape_contour))
        sparse_shape_contour = self.make_sparse(shape_contour,1000)
            
        #Optimize
        new_model_symm = black_box_opt(model=self.model,contour=shape_contour,energy_fxn=self.energy_fxn,num_iters = 1,delta=25.0)
        #new_model_symm.draw_to_image(img=self.image2,color=cv.CV_RGB(0,255,0))

        new_model_asymm = black_box_opt(model=new_model_symm.make_asymm(),contour=shape_contour,energy_fxn=self.energy_fxn,num_iters=10,delta=25.0,exploration_factor=1.9)  
        if(SHOW_FINAL_MODEL):
            new_model_asymm.draw_to_image(img=self.image2,color=cv.CV_RGB(255,0,255))
        pts = []
        for vert in new_model_asymm.vertices_full():
            nearest_pt = min(shape_contour,key=lambda pt: distance(pt,vert))
            pts.append(nearest_pt)
        
        params = {}
        if self.mode == "triangles":
            return_pts = [pts[1],pts[4],pts[2],pts[3]]
            self.highlight_pt(pts[1],cv.CV_RGB(255,0,0))
            font = cv.InitFont(cv.CV_FONT_HERSHEY_COMPLEX,1.0,1.0)
            cv.PutText(self.image2,"(l)eft",(pts[1][0]-20,pts[1][1]-15),font,cv.CV_RGB(255,0,0))
            self.highlight_pt(pts[4],cv.CV_RGB(0,0,255))
            cv.PutText(self.image2,"(r)ight",(pts[4][0]-20,pts[4][1]-15),font,cv.CV_RGB(0,0,255))
            params = {"tilt":0.0}
        elif self.mode == "towel":
            return_pts = pts
        else:
            return_pts = pts
            params = {}
        if self.mode != "triangles":
            for pt in return_pts:
                self.highlight_pt(pt,cv.CV_RGB(255,255,255))
        return (return_pts,params,self.image2)
        
    def energy_fxn(self,model,contour):
        model_dist_param = 0.5
        contour_dist_param = 0.5
        sparse_contour = self.make_sparse(contour,1000)
        extra_sparse_contour = self.make_sparse(contour,500)
        model_contour = model.vertices_dense(constant_length=False,density=30)
        
        nn_model = self.nearest_neighbors_fast(model_contour,sparse_contour)
        model_dist_energy = sum([self.dist_fxn(dist) for dist in nn_model]) / float(len(nn_model))
        #Normalize
        model_dist_energy /= float(self.dist_fxn(max(self.image2.width,self.image2.height)))

        nn_contour = self.nearest_neighbors_fast(extra_sparse_contour,model_contour)
        contour_dist_energy = sum([self.dist_fxn(dist) for dist in nn_contour]) / float(len(nn_contour))
        #Normalize
        contour_dist_energy /= float(self.dist_fxn(max(self.image2.width,self.image2.height)))
        
        
        energy = model_dist_param * model_dist_energy + contour_dist_param * contour_dist_energy
        
        #if model.illegal():
        #    energy = 1.0
            
        return energy
    
    def load_model(self,filepath):
        self.model = pickle.load(open(filepath))
        #self.model = modelClass.vertices_full()
        
    def get_model_contour(self):
        return self.model.vertices_full()
        
    def get_dense_model_contour(self):
        return self.model.vertices_dense(constant_length=False,density=20)
            
    def nearest_neighbors_fast(self,model_contour,sparse_contour):
        model_arr = array(model_contour)
        contour_arr = array(sparse_contour)
        result,dists = self.flann.nn(sparse_contour,model_contour, num_neighbors=1,algorithm="kmeans",branching=32, iterations=7, checks=16);
        return [sqrt(dist) for dist in dists]
        
    def image_edge(self,contour):
        width = self.image.width
        height = self.image.height
        for (x,y) in contour:
            if x < NEAREST_EDGE:
                return True
            if x > width - NEAREST_EDGE:
                return True
            if y < NEAREST_EDGE:
                return True
            if y > height - NEAREST_EDGE:
                return True
        return False
        
    def highlight_pt(self,pt,color=None):
        if color == None:
            color = cv.CV_RGB(128,128,128)
        cv.Circle(self.image2,pt,5,color,-1)
    
    
    def get_principle_info(self,shape):
        """
        storage2 = cv.CreateMemStorage(0)
        bounding = cv.MinAreaRect2(shape,storage2)
        #(x, y, width, height) = bounding
        #center_x = x + width/2
        #center_y = y + width/2
        #center_x = avg([x for (x,y) in shape_contour])
        #center_y = avg([y for (x,y) in shape_contour])
        center = (bounding[0])
        """
        moments = cv.Moments(shape,0)
        center = get_center(moments)
        self.moments = moments
        
        theta = get_angle(moments)
        (top_pt,scale) = self.get_top(shape,center,theta)
        #scale = distance(center,top_pt)
        print "Scale = %s"%scale
        return(center,top_pt,theta,scale)
        
    def get_top(self,shape,center,theta):
        pt = center
        EPSILON = 1.0
        angle = theta
        scale = 0
        print "ANGLE = %s"%angle
        while(cv.PointPolygonTest(shape,pt,0) > 0):
            (x,y) = pt
            new_x = x + EPSILON*sin(angle)
            new_y = y - EPSILON*cos(angle)
            pt = (new_x,new_y)
            scale += EPSILON
        return (pt,scale)
        
    def make_sparse(self,contour,num_pts = 1000):
        sparsity = int(math.ceil(len(contour) / float(num_pts)))
        sparse_contour = []
        for i,pt in enumerate(contour):
            if i%sparsity == 0:
                sparse_contour.append(pt)
        return sparse_contour

def main(args):
    rospy.init_node("shape_fitter_node")
    sfn = ShapeFitter()
    rospy.spin()
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
