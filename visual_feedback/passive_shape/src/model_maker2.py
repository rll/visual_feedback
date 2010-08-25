#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import roslib
import sys
roslib.load_manifest("passive_shape")
import rospy
from numpy import *
import math
import cv
import os.path
from ShapeWindow import *
import ShapeWindowUtils
import Geometry2D
import pickle
import Vector2D


ASYMM = 0
SYMM = 1
SKEL = 2

TYPE = SKEL

class ModelMaker(ShapeWindow):
    
    def __init__(self,filepath,modelpath):
        bgd = cv.LoadImage(filepath,cv.CV_LOAD_IMAGE_GRAYSCALE)
        
        self.modelpath = modelpath
        size = (bgd.width,bgd.height)
        ShapeWindow.__init__(self,name="Model Maker",size=size)
        self.shapesLock.acquire()
        self.img = cv.CloneImage(bgd)
        self.background = cv.CloneImage(bgd)
        self.shapesLock.release()
    
    def initExtended(self):
        self.lineColor = Colors.BLACK
        if TYPE == SYMM:
            self.has_symmline = False
            self.symmline = None
            self.symmpts = []
        if TYPE == SKEL:
            self.mode = 0
            self.spine_bottom = None
            self.spine_top = None
            self.collar = None
            self.spine_arm_junction = None
            self.shoulder_joint = None
            self.shoulder_top = None
            self.sleeve_node = None
            self.sleeve_top = None
        clearShapesButton = CVButton(text="CLEAR",bottomLeft=Geometry2D.Point(50,100), onClick=self.clearAll)
        self.addOverlay(clearShapesButton)
        saveModelButton = CVButton(text="SAVE MODEL",bottomLeft=Geometry2D.Point(150,100), onClick = self.saveModel)
        self.addOverlay(saveModelButton)
        closeButton = CVButton(text="CLOSE",bottomLeft=Geometry2D.Point(350,100), onClick = self.close)
        self.addOverlay(closeButton)
        
    def clearAll(self):
        self.clearShapes()
        if TYPE==SYMM:
            self.mode = 0
    
    def onMouse(self,event,x,y,flags,param):
        if TYPE==SYMM:
            if not self.has_symmline:
                return self.symmLineDrawer(event,x,y,flags,param)
            else:
                return self.symmPolyDrawer(event,x,y,flags,param)
        elif TYPE==ASYMM:
            return self.polyDrawer(event,x,y,flags,param)
        else:
            return self.skelDrawer(event,x,y,flags,param)
            
    def skelDrawer(self,event,x,y,flags,param):
        if self.mode==0:
            #Draw spine_bottom
            if event==cv.CV_EVENT_LBUTTONUP:
                self.spine_bottom = Geometry2D.Point(x,y)
                self.permanentHighlightPt(self.spine_bottom)
                self.mode += 1
                    
        elif self.mode==1:
                if event==cv.CV_EVENT_LBUTTONUP:
                    self.spine_top = Geometry2D.Point(x,y)
                    self.permanentHighlightPt(self.spine_top)
                    self.permanentHighlightSegment(Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                    self.mode += 1
                else:
                    self.highlightSegment(Geometry2D.LineSegment(self.spine_bottom,Geometry2D.Point(x,y)))
            
        elif self.mode==2:
            #Draw collar
            if event==cv.CV_EVENT_LBUTTONUP:
                self.collar = Geometry2D.Point(x,y)
                self.virtual_collar = Geometry2D.mirrorPt(self.collar,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                self.permanentHighlightPt(self.collar)
                self.permanentHighlightSegment(Geometry2D.LineSegment(self.spine_top,self.collar))
                self.permanentHighlightPt(self.virtual_collar)
                self.permanentHighlightSegment(Geometry2D.LineSegment(self.spine_top,self.virtual_collar))
                self.mode += 1
            
            else:
                temp_collar = Geometry2D.Point(x,y)
                virtual_collar = Geometry2D.mirrorPt(temp_collar,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                self.highlightPt(temp_collar)
                self.highlightSegment(Geometry2D.LineSegment(self.spine_top,temp_collar))
                self.highlightPt(virtual_collar)
                self.highlightSegment(Geometry2D.LineSegment(self.spine_top,virtual_collar))
                

        elif self.mode==3:
            #Draw shoulder
            new_pt = Geometry2D.Point(x,y)
            if event==cv.CV_EVENT_LBUTTONUP:
                self.shoulder_joint = new_pt
                self.virtual_shoulder_joint = Geometry2D.mirrorPt(self.shoulder_joint,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                self.permanentHighlightPt(self.shoulder_joint)
                self.permanentHighlightPt(self.virtual_shoulder_joint)
                seg = Geometry2D.LineSegment(self.shoulder_joint,self.virtual_shoulder_joint)
                self.permanentHighlightSegment(seg)
                self.bottom_left = Geometry2D.Point(self.spine_bottom.x()-0.5*seg.dx(),self.spine_bottom.y()-0.5*seg.dy())
                self.bottom_right = Geometry2D.Point(self.spine_bottom.x()+0.5*seg.dx(),self.spine_bottom.y()+0.5*seg.dy())
                bottom_seg = Geometry2D.LineSegment(self.bottom_left,self.bottom_right)
                self.permanentHighlightPt(self.bottom_left)
                self.permanentHighlightPt(self.bottom_right)
                self.permanentHighlightSegment(bottom_seg)
                self.mode += 1
            else:
                temp_shoulder_joint = new_pt
                virtual_shoulder_joint = Geometry2D.mirrorPt(temp_shoulder_joint,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                self.highlightPt(temp_shoulder_joint)
                self.highlightPt(virtual_shoulder_joint)
                seg = Geometry2D.LineSegment(temp_shoulder_joint,virtual_shoulder_joint)
                self.highlightSegment(seg)
                bottom_left = Geometry2D.Point(self.spine_bottom.x()-0.5*seg.dx(),self.spine_bottom.y()-0.5*seg.dy())
                bottom_right = Geometry2D.Point(self.spine_bottom.x()+0.5*seg.dx(),self.spine_bottom.y()+0.5*seg.dy())
                bottom_seg = Geometry2D.LineSegment(bottom_left,bottom_right)
                self.highlightPt(bottom_left)
                self.highlightPt(bottom_right)
                self.highlightSegment(bottom_seg)

        elif self.mode==4:
            new_pt = Geometry2D.Point(x,y)
            if event==cv.CV_EVENT_LBUTTONUP:
                self.shoulder_top = new_pt
                seg = Geometry2D.LineSegment(self.shoulder_top,self.shoulder_joint)
                self.armpit = seg.extrapolate(2.0)
                self.virtual_shoulder_top = Geometry2D.mirrorPt(self.shoulder_top,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                self.virtual_armpit = Geometry2D.mirrorPt(self.armpit,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                self.permanentHighlightPt(self.shoulder_top)
                self.permanentHighlightPt(self.armpit)
                self.permanentHighlightPt(self.virtual_shoulder_top)
                self.permanentHighlightPt(self.virtual_armpit)
                self.permanentHighlightSegment(Geometry2D.LineSegment(self.armpit,self.shoulder_top))
                self.permanentHighlightSegment(Geometry2D.LineSegment(self.virtual_armpit,self.virtual_shoulder_top))
                self.mode+=1
            else:
                shoulder_top = new_pt
                seg = Geometry2D.LineSegment(shoulder_top,self.shoulder_joint)
                armpit = seg.extrapolate(2.0)
                virtual_shoulder_top = Geometry2D.mirrorPt(shoulder_top,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                virtual_armpit = Geometry2D.mirrorPt(armpit,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                self.highlightPt(shoulder_top)
                self.highlightPt(armpit)
                self.highlightPt(virtual_shoulder_top)
                self.highlightPt(virtual_armpit)
                self.highlightSegment(Geometry2D.LineSegment(armpit,shoulder_top))
                self.highlightSegment(Geometry2D.LineSegment(virtual_armpit,virtual_shoulder_top))
           
        elif self.mode==5:
            new_pt = Geometry2D.Point(x,y)
            if event==cv.CV_EVENT_LBUTTONUP:
                self.sleeve_node = new_pt
                self.virtual_sleeve_node = Geometry2D.mirrorPt(self.sleeve_node,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                self.permanentHighlightPt(self.sleeve_node)
                self.permanentHighlightPt(self.virtual_sleeve_node)
                self.permanentHighlightSegment(Geometry2D.LineSegment(self.shoulder_joint,self.sleeve_node))
                self.permanentHighlightSegment(Geometry2D.LineSegment(self.virtual_shoulder_joint,self.virtual_sleeve_node))
                self.mode += 1
            else:
                temp_sleeve_node = new_pt
                virtual_sleeve_node = Geometry2D.mirrorPt(temp_sleeve_node,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                self.highlightPt(temp_sleeve_node)
                self.highlightPt(virtual_sleeve_node)
                self.highlightSegment(Geometry2D.LineSegment(self.shoulder_joint,temp_sleeve_node))
                self.highlightSegment(Geometry2D.LineSegment(self.virtual_shoulder_joint,virtual_sleeve_node))
        elif self.mode==6:
            new_pt = Geometry2D.Point(x,y)
            if event==cv.CV_EVENT_LBUTTONUP:
                self.sleeve_top = new_pt
                seg = Geometry2D.LineSegment(self.sleeve_top,self.sleeve_node)
                self.sleeve_bottom = seg.extrapolate(2.0)
                self.virtual_sleeve_top = Geometry2D.mirrorPt(self.sleeve_top,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                self.virtual_sleeve_bottom = Geometry2D.mirrorPt(self.sleeve_bottom,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                self.permanentHighlightPt(self.sleeve_top)
                self.permanentHighlightPt(self.sleeve_bottom)
                self.permanentHighlightPt(self.virtual_sleeve_top)
                self.permanentHighlightPt(self.virtual_sleeve_bottom)
                self.permanentHighlightSegment(Geometry2D.LineSegment(self.sleeve_bottom,self.sleeve_top))
                self.permanentHighlightSegment(Geometry2D.LineSegment(self.virtual_sleeve_bottom,self.virtual_sleeve_top))

                self.addCVShape(CVPolygon(cv.CV_RGB(0,0,0),self.front(),
                                Geometry2D.Polygon( self.bottom_left,self.armpit,self.sleeve_bottom,self.sleeve_top,self.shoulder_top,self.collar,
                                                    self.virtual_collar,self.virtual_shoulder_top,self.virtual_sleeve_top,self.virtual_sleeve_bottom,self.virtual_armpit,self.bottom_right)))
                self.mode+=1
            else:
                sleeve_top = new_pt
                seg = Geometry2D.LineSegment(sleeve_top,self.sleeve_node)
                sleeve_bottom = seg.extrapolate(2.0)
                virtual_sleeve_top = Geometry2D.mirrorPt(sleeve_top,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                virtual_sleeve_bottom = Geometry2D.mirrorPt(sleeve_bottom,Geometry2D.LineSegment(self.spine_bottom,self.spine_top))
                self.highlightPt(sleeve_top)
                self.highlightPt(sleeve_bottom)
                self.highlightPt(virtual_sleeve_top)
                self.highlightPt(virtual_sleeve_bottom)
                self.highlightSegment(Geometry2D.LineSegment(sleeve_bottom,sleeve_top))
                self.highlightSegment(Geometry2D.LineSegment(virtual_sleeve_bottom,virtual_sleeve_top))    
            
    def permanentHighlightPt(self,pt):
        self.addCVShape(CVCircle(cv.CV_RGB(0,0,0),self.front(),Geometry2D.Circle(pt,3)))    
    
    def permanentHighlightSegment(self,seg):
        self.addCVShape(CVLineSegment(cv.CV_RGB(0,0,0),self.front(),seg))  
            
            
            
    def symmLineDrawer(self,event,x,y,flags,param):
        if event==cv.CV_EVENT_LBUTTONUP:
            self.symmpts.append((x,y))
            print "ADDED PT"
            cv.Circle(self.background,(x,y),3,cv.CV_RGB(0,0,0),-1)
            if len(self.symmpts) >= 2:
                self.symmline = Vector2D.make_ln_from_pts(self.symmpts[0],self.symmpts[1])
                ln_start = Vector2D.intercept(self.symmline,Vector2D.horiz_ln(y=0))
                ln_end = Vector2D.intercept(self.symmline,Vector2D.horiz_ln(y=self.background.height))
                cv.Line(self.background,ln_start,ln_end,cv.CV_RGB(0,0,0))
                self.has_symmline = True
                print self.symmline
        elif len(self.symmpts) > 0:
            self.addTempCVShape(CVLineSegment(cv.CV_RGB(255,255,255),2,Geometry2D.LineSegment(Geometry2D.Point(self.symmpts[0][0],self.symmpts[0][1]),Geometry2D.Point(x,y))))
                
    def symmPolyDrawer(self,event,x,y,flags,param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            self.newPoly.append(Geometry2D.Point(x,y))
            
        elif event == cv.CV_EVENT_RBUTTONDOWN:
            print "MADE IT"
            backwards = list(self.newPoly)
            backwards.reverse()
            backwards = [pt.toTuple() for pt in backwards]
            for pt in backwards:
                print "Looking at pt"
                newpt = Vector2D.mirror_pt(pt,self.symmline)
                print newpt
                self.newPoly.append(Geometry2D.Point(newpt[0],newpt[1]))
                print "Added pt"
                print len(self.newPoly)
            poly = Geometry2D.Polygon(*self.newPoly)
            cvPoly = CVPolygon(self.getDrawColor(),self.front(),poly)
            self.addCVShape(cvPoly)
            self.newPoly = []    
        
        elif len(self.newPoly) > 0:
            startPt = self.newPoly[-1]
            endPt = Geometry2D.Point(x,y)
            line = Geometry2D.LineSegment(startPt,endPt)
            cvLine = CVLineSegment(self.lineColor,self.tempFront(),line)
            self.addTempCVShape(cvLine)
        for i, pt in enumerate(self.newPoly):
            self.highlightPt(pt)
            if i > 0:
                startPt = self.newPoly[i-1]
                line = Geometry2D.LineSegment(startPt,pt)
                cvLine = CVLineSegment(self.lineColor,self.tempFront(),line)
                self.addTempCVShape(cvLine)
    
    def saveModel(self):
        file = open(self.modelpath,'w')
        if TYPE==ASYMM:
            model = self.getModelAsymm()
        if TYPE==SYMM:
            model = self.getModelSymm()
        elif TYPE==SKEL:
            model = self.getModelSkel()
        pickle.dump(model,file)
        
    def getModelAsymm(self):
        poly = self.getPolys()[0].getShape()
        #Due to symmetry, we only need half the points)
        vertices = poly.vertices()
        tuple_vertices = [v.toTuple() for v in vertices]
        model = Vector2D.Model_Asymm(tuple_vertices)
        return model
    
    def getModelSymm(self):
        poly = self.getPolys()[0].getShape()
        #Due to symmetry, we only need half the points)
        vertices = poly.vertices()[0:len(poly.vertices())/2]
        tuple_vertices = [v.toTuple() for v in vertices]
        model = Vector2D.Model_Symm(tuple_vertices,self.symmline)
        return model
        
    def getModelSkel(self):
        #Parameters: spine_bottom,spine_top,collar,shoulder_joint,shoulder_top,sleeve_center,sleeve_top
        return Vector2D.Model_Skel(
            spine_bottom=self.spine_bottom.toTuple(), spine_top=self.spine_top.toTuple(),
            collar=self.collar.toTuple(), shoulder_joint=self.shoulder_joint.toTuple(),
            shoulder_top=self.shoulder_top.toTuple(), sleeve_center=self.sleeve_node.toTuple(),sleeve_top=self.sleeve_top.toTuple()
            )
    

    
def main(args):
    corrected_filepath = args[0]
    corrected_modelpath = args[1]

    mm = ModelMaker(corrected_filepath,corrected_modelpath)
    while(not mm.isClosed()):
        rospy.sleep(0.05)
    return
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
