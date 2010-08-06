import math
import random
from numpy import *
import cv

def pt_x(pt):
    return pt[0]

def pt_y(pt):
    return pt[1]
    
def make_pt(x,y):
    return (float(x),float(y))

def pt_sum(pt1,pt2):
    return (pt1[0]+pt2[0],pt1[1]+pt2[1])
    
def pt_diff(pt1,pt2):
    return (pt1[0]-pt2[0],pt1[1]-pt2[1])
    
def pt_scale(pt,scale):
    return (pt[0]*scale,pt[1]*scale)
    
def dot_prod(pt1,pt2):
    return pt1[0]*pt2[0]+pt1[1]*pt2[1]
    
def vect_length(vect):
    return sqrt(dot_prod(vect,vect))
    
def make_ln_from_pts(pt1,pt2):
    vect = pt_diff(pt2,pt1)
    if pt_y(vect) < 0:
        vect = pt_scale(vect,-1)
    offset = pt1
    return make_ln(offset=offset,vect=vect)
    
def make_ln(offset,vect):
    norm_vect = pt_scale(vect, 1.0 / vect_length(vect))
    return (offset,norm_vect)
    
def horiz_ln(y):
    return make_ln(offset=(0,y),vect=(1,0))
    
def line_offset(ln):
    return ln[0]
    
def line_vector(ln):
    return ln[1]
    
def perpendicular(ln,pt):
    (dx,dy) = line_vector(ln)
    perp_vector = (-1*dy,dx)
    return make_ln(offset=pt,vect=perp_vector)

def projection(pt,ln):
    offset = line_offset(ln)
    pt_o = pt_diff(pt,offset)
    vect = line_vector(ln)
    new_pt = pt_scale(vect,dot_prod(pt_o,vect)/float(dot_prod(vect,vect)))
    return pt_sum(pt_o,offset)

    
def extrapolate(ln,amt):
    return pt_sum(line_offset(ln), pt_scale(line_vector(ln),amt))
    
def intercept(ln1,ln2):
    (m1,b1) = slope_intercept(ln1)
    
    (m2,b2) = slope_intercept(ln2)
    if m1 == m2:
        return None
    else:
        x = (b2 - b1) / (m1 - m2)
        y = m1*x + b1
        return make_pt(x,y)
    
def slope_intercept(ln):
    (xo,yo) = line_offset(ln)
    (xv,yv) = line_vector(ln)
    if xv==0:
        return (None,None) #Horizontal lines have no slope_intercept form
    m = float(yv)/xv
    b = yo - float(yv)/xv * xo
    return (m,b)
    
def mirror_pt(pt,ln):
    #First, convert from affine to linear
    offset = line_offset(ln)
    vect = line_vector(ln)
    pt_o = pt_diff(pt,offset)
    pt_r = pt_diff(pt_scale(vect,2 * dot_prod(pt_o,vect) / float(dot_prod(vect,vect))), pt_o)
    
    return pt_sum(pt_r,offset)
    
class Model:
    def __init__(self,vertices):
        self.vertices = list(vertices)
    
    def vertices_full(self):
        abstract
        
    def vertices_dense(self,density=10,length=10,constant_length=False):
        vertices = self.vertices_full()
        output = []
        for i,vert in enumerate(vertices):
            output.append(vert)
            next_vert = vertices[(i+1)%len(vertices)]
            displ = pt_diff(next_vert,vert)
            side_length = vect_length(displ)
            if not constant_length:
                dt = 1/float(density+1)
                num_pts = density
            else:
                num_pts = int(side_length / length) - 1
                dt = 1.0 / (num_pts+1)
            for j in range(num_pts):
                new_pt = pt_sum(vert,pt_scale(displ,dt*(j+1)))
                output.append(new_pt)
        return output
        
    def translate(self,trans):
        self.vertices = translate_pts(self.vertices,trans)
        
    def rotate(self,angle,origin=(0,0)):
        self.vertices = rotate_pts(self.vertices,angle,origin)
        
    def scale(self,amt,origin=(0,0)):
        self.vertices = scale_pts(self.vertices,amt,origin)
    
    def params(self):
        abstract
        
    def from_params(self,params):
        abstract
        
    def draw_to_image(self,img,color):
        cv.PolyLine(img,[self.vertices_full()],1,color,2)
        
    def draw_point(self,img,pt,color):
        cv.Circle(img,pt,5,color,-1)
        
    def draw_line(self,img,pt1,pt2,color):
        cv.Line(img,pt1,pt2,color)
        
        
class Model_Symm(Model):
    def __init__(self,vertices,symmline):
        Model.__init__(self,vertices)
        self.symmline = symmline
    
    def vertices_full(self):
        backwards = list(self.vertices)
        backwards.reverse()
        output = list(self.vertices)
        for pt in backwards:
            output.append(mirror_pt(pt,self.symmline))
        return output
    
    def translate(self,trans):
        Model.translate(self,trans)
        self.symmline = translate_ln(self.symmline,trans)
        
    def rotate(self,angle,origin=(0,0)):
        Model.rotate(self,angle,origin)
        self.symmline = rotate_ln(self.symmline,angle,origin)
        
    def scale(self,amt,origin=(0,0)):
        Model.scale(self,amt,origin)
        self.symmline = scale_ln(self.symmline,amt,origin)   
        
    def params(self):
        output = []
        for pt in self.vertices:
            output.append(pt_x(pt))
            output.append(pt_y(pt))
        return output
        
    def from_params(self,params):
        pts = []
        x = None
        for i,p in enumerate(params):
            if i%2 == 0:
                x = p
            else:
                y = p
                pts.append((x,y))
        return Model_Symm(pts,self.symmline)
        
    def make_asymm(self):
        return Model_Asymm(self.vertices_full())
        
class Model_Asymm(Model):

    def vertices_full(self):
        return list(self.vertices)
        
    def params(self):
        output = []
        for pt in self.vertices:
            output.append(pt_x(pt))
            output.append(pt_y(pt))
        return output
        
    def from_params(self,params):
        pts = []
        x = None
        for i,p in enumerate(params):
            if i%2 == 0:
                x = p
            else:
                y = p
                pts.append((x,y))
        return Model_Asymm(pts)
        
class Model_Skel(Model):
    def __init__(self,spine_bottom,spine_top,collar,shoulder_joint,shoulder_top,sleeve_center,sleeve_top):
        self.vertices = [spine_bottom,spine_top,collar,shoulder_joint,shoulder_top,sleeve_center,sleeve_top]
        
    def spine_bottom(self):
        return self.vertices[0]
        
    def spine_top(self):
        return self.vertices[1]
        
        
    def left_collar(self):
        return self.vertices[2]
        
    def right_collar(self):
        return self.left_to_right(self.left_collar())
        
    def left_shoulder_joint(self):
        return self.vertices[3]
        
    def right_shoulder_joint(self):
        return self.left_to_right(self.left_shoulder_joint())
        
    def left_shoulder_top(self):
        return self.vertices[4]
        #return (self.left_shoulder_joint()[0],self.vertices[4][1])
        
    def right_shoulder_top(self):
        return self.left_to_right(self.left_shoulder_top())
        
    def left_armpit(self):
        ln = perpendicular(make_ln_from_pts(self.left_shoulder_top(),self.left_shoulder_joint()),self.left_shoulder_joint())
        return mirror_pt(self.left_shoulder_top(),ln)
        #return mirror_pt(self.left_shoulder_top(),self.horiz_frame())
        
    def right_armpit(self):
        ln = perpendicular(make_ln_from_pts(self.right_shoulder_top(),self.right_shoulder_joint()),self.right_shoulder_joint())
        return mirror_pt(self.right_shoulder_top(),ln)
        #return mirror_pt(self.right_shoulder_top(),self.horiz_frame())
        
    def left_sleeve_center(self):
        return self.vertices[5]
        
    def right_sleeve_center(self):
        return self.left_to_right(self.left_sleeve_center())
        
    def left_sleeve_top(self):
        return self.vertices[6]
        
    def right_sleeve_top(self):
        return self.left_to_right(self.left_sleeve_top())
    
    def left_sleeve_bottom(self):
        ln = perpendicular(make_ln_from_pts(self.left_sleeve_top(),self.left_sleeve_center()),self.left_sleeve_center())
        return mirror_pt(self.left_sleeve_top(),ln)
        
    def right_sleeve_bottom(self):
        ln = perpendicular(make_ln_from_pts(self.right_sleeve_top(),self.right_sleeve_center()),self.right_sleeve_center())
        return mirror_pt(self.right_sleeve_top(),ln)
        
    def bottom_left(self):
        displ = pt_diff(self.right_shoulder_joint(),self.left_shoulder_joint())
        return pt_sum(self.spine_bottom(),pt_scale(displ,-0.5))
        
    def bottom_right(self):
        displ = pt_diff(self.right_shoulder_joint(),self.left_shoulder_joint())
        return pt_sum(self.spine_bottom(),pt_scale(displ,0.5))
        
    def left_to_right(self,pt):
        spine = make_ln_from_pts(self.spine_bottom(),self.spine_top())
        return mirror_pt(pt,spine)
        
    def horiz_frame(self):
        return make_ln_from_pts(self.left_shoulder_joint(),self.right_shoulder_joint())
        
    def bottom_edge(self):
        return make_ln(offset=self.spine_bottom(), vect= line_vector(self.horiz_frame()))
        
    def vertices_full(self):
        return [self.bottom_left(),self.left_armpit(),self.left_sleeve_bottom(),self.left_sleeve_top(),self.left_shoulder_top(),self.left_collar(),self.spine_top()
               ,self.right_collar(),self.right_shoulder_top(),self.right_sleeve_top(),self.right_sleeve_bottom(),self.right_armpit(),self.bottom_right()]
    
    def variable_pts(self):
        return  [self.spine_bottom(),self.spine_top(), self.left_collar(),self.left_shoulder_joint()
                ,self.left_shoulder_top(),self.left_sleeve_center(),self.left_sleeve_top()]
               
    def params(self):
        output = []
        for (x,y) in self.variable_pts():
            output.append(x)
            output.append(y)
        return output
        
    def from_params(self,params):
        pts = []
        x = None
        for i,v in enumerate(params):
            if i%2==0:
                x = v
            else:
                y = v
                pts.append((x,y))
        return Model_Skel(*pts)
        
    def draw_to_image(self,img,color):
        Model.draw_to_image(self,img,color)
        #Draw skeletal frame
        self.draw_point(img,self.spine_bottom(),color)
        self.draw_line(img,self.spine_bottom(),self.spine_top(),color)
        self.draw_point(img,self.left_shoulder_joint(),color)
        self.draw_point(img,self.right_shoulder_joint(),color)
        self.draw_line(img,self.left_shoulder_joint(),self.right_shoulder_joint(),color)
        self.draw_point(img,self.left_sleeve_center(),color)
        self.draw_line(img,self.left_shoulder_joint(),self.left_sleeve_center(),color)
        self.draw_point(img,self.right_sleeve_center(),color)
        self.draw_line(img,self.right_shoulder_joint(),self.right_sleeve_center(),color)
        self.draw_line(img,self.right_shoulder_top(),self.right_armpit(),color)
        self.draw_line(img,self.left_shoulder_top(),self.left_armpit(),color)

        
    def make_asymm(self):
        my_vertices = self.variable_pts()
        additional_vertices = [self.right_collar(),self.right_shoulder_joint(),self.right_shoulder_top(),self.right_sleeve_center(),self.right_sleeve_top()]
        full = my_vertices + additional_vertices
        return Model_Skel_Asymm(*full)
        
class Model_Skel_Asymm(Model_Skel):
    def __init__(self,spine_bottom,spine_top,left_collar,left_shoulder_joint,left_shoulder_top,left_sleeve_center,left_sleeve_top
                ,right_collar,right_shoulder_joint,right_shoulder_top,right_sleeve_center,right_sleeve_top):
        Model_Skel.__init__(self,spine_bottom,spine_top,left_collar,left_shoulder_joint,left_shoulder_top,left_sleeve_center,left_sleeve_top)
        self.vertices.append(right_collar)
        self.vertices.append(right_shoulder_joint)
        self.vertices.append(right_shoulder_top)
        self.vertices.append(right_sleeve_center)
        self.vertices.append(right_sleeve_top)
        
    #def right_collar(self):
    #    return self.child_vertices()[0]
        
    def right_shoulder_joint(self):
        return self.child_vertices()[1]
        
    def right_shoulder_top(self):
        return self.child_vertices()[2]
        #return (self.right_shoulder_joint()[0],self.child_vertices()[2][1])
        
    def right_sleeve_center(self):
        return self.child_vertices()[3]
        
    def right_sleeve_top(self):
        return self.child_vertices()[4]
        
    def child_vertices(self):
        start_i = len(Model_Skel.variable_pts(self))
        return self.vertices[start_i:]
        
    def variable_pts(self):
        pts = Model_Skel.variable_pts(self)
        pts.append(self.right_collar())
        pts.append(self.right_shoulder_joint())
        pts.append(self.right_shoulder_top())
        pts.append(self.right_sleeve_center())
        pts.append(self.right_sleeve_top())
        return pts
        
    def from_params(self,params):
        pts = []
        x = None
        for i,v in enumerate(params):
            if i%2==0:
                x = v
            else:
                y = v
                pts.append((x,y))
        return Model_Skel_Asymm(*pts)
            
            
def translate_pt(pt,trans):
    (x,y) = pt
    (x_displ,y_displ) = trans
    (x_t,y_t) = (x+x_displ,y+y_displ)
    return (x_t,y_t)

def translate_pts(pts,trans):
    return [translate_pt(pt,trans) for pt in pts]
    
def translate_ln(ln,trans):
    start = extrapolate(ln,-1)
    end = extrapolate(ln,1)
    (new_start,new_end) = translate_pts((start,end),trans)
    return make_ln_from_pts(new_start,new_end)

def rotate_pt(pt,angle,origin=(0,0)):
    (x,y) = pt
    (x_o,y_o) = origin
    (x_n,y_n) = (x-x_o,y-y_o)
    off_rot_x = x_n*cos(angle) - y_n*sin(angle)
    off_rot_y = y_n*cos(angle) + x_n*sin(angle)
    rot_x = off_rot_x + x_o
    rot_y = off_rot_y + y_o
    return (rot_x,rot_y)

def rotate_pts(pts,angle,origin=(0,0)):
    return [rotate_pt(pt,angle,origin) for pt in pts]
    
def rotate_ln(ln,angle,origin=(0.0)):
    start = extrapolate(ln,-1)
    end = extrapolate(ln,1)
    (new_start,new_end) = rotate_pts((start,end),angle,origin)
    return make_ln_from_pts(new_start,new_end)

def scale_pt(pt,amt,origin=(0,0)):
    (x,y) = pt
    (x_o,y_o) = origin
    (x_n,y_n) = (x-x_o,y-y_o)
    (x_ns,y_ns) = (amt*x_n,amt*y_n)
    (x_s,y_s) = (x_ns+x_o,y_ns+y_o)
    return (x_s,y_s)

def scale_pts(pts,amt,origin=(0,0)):
    return [scale_pt(pt,amt,origin) for pt in pts]
    
def scale_ln(ln,amt,origin=(0.0)):
    start = extrapolate(ln,-1)
    end = extrapolate(ln,1)
    (new_start,new_end) = scale_pts((start,end),amt,origin)
    return make_ln_from_pts(new_start,new_end)
        
