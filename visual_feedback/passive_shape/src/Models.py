import math
import random
from numpy import *
import cv
from Vector2D import *
import inspect


def make_sparse(contour,num_pts = 1000):
        sparsity = int(math.ceil(len(contour) / float(num_pts)))
        sparse_contour = []
        for i,pt in enumerate(contour):
            if i%sparsity == 0:
                sparse_contour.append(pt)
        return sparse_contour

#Represents a structural constraint

(LOWER,UPPER,ASYMPTOTE) = range(3)


#Abstract model class        
class Model:

    def preferred_delta(self):
        return 35.0

    def vertices_full(self):
        abstract
        
    def set_image(self,image):
        self.image = image
          
    def sides(self):
        verts =  self.vertices_full()
        segs = []
        for i,v in enumerate(verts):
            segs.append(make_seg(verts[i-1],v))
        return segs
        
    def vertices_dense(self,density=10,length=10,constant_length=False,contour_mode=False):
        vertices = self.vertices_full()
        
        if self.contour_mode():
            sil = self.get_silhouette(vertices,num_pts=density*len(self.sides()))
            return sil
        
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
                num_pts = max(int(side_length / length) - 1,0)
                dt = 1.0 / (num_pts+1)
            for j in range(num_pts):
                new_pt = pt_sum(vert,pt_scale(displ,dt*(j+1)))
                output.append(new_pt)
        
        return output
    
    def structural_penalty(self):
        return 0
        
    def contour_mode(self):
        return False
    
    #Simple hack to get the outer contour: draw it on a white background and find the largest contour            
    def get_silhouette(self,vertices,num_pts):
        storage = cv.CreateMemStorage(0)
        black_image = cv.CreateImage(cv.GetSize(self.image),8,1)
        cv.Set(black_image,cv.CV_RGB(0,0,0))
        cv.PolyLine(black_image,[vertices],4,cv.CV_RGB(255,255,255),0)
        #cv.NamedWindow("Black Image")
        #cv.ShowImage("Black Image",black_image)
        #cv.WaitKey(10.0)
        contour = cv.FindContours   ( black_image, storage,
                                    cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_NONE, (0,0))
        max_contour = None
        max_ar = 0.0
        while contour != None:
            ar = abs(cv.ContourArea(contour))   
            if ar > max_ar:
                max_ar = ar
                max_contour = contour
            contour = contour.h_next()
        
        return make_sparse(max_contour,num_pts)

    def center(self):
        xs = [x for (x,y) in self.vertices_full()]
        ys = [y for (x,y) in self.vertices_full()]
        min_x = min(xs)
        min_y = min(ys)
        max_x = max(xs)
        max_y = max(ys)
        return (0.5*(min_x + max_x),0.5*(min_y + max_y))
        
    def translate(self,trans):
        abstract
        
    def rotate(self,angle,origin=None):
        abstract
        
    def scale(self,amt,origin=None):
        abstract
    
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
        

    def make_asymm(self):
        return self
        
            
    def constrain(self,value,limit,limit_type,sigma):
        if sigma > 0:
            gaussian = exp( - (value - limit)**2 / (2*sigma**2) )
        # A sigma of 0 indicates a boolean constraint: as in the legality constraint
        else:
            gaussian = 0
        if limit_type == LOWER:
            if value <= limit:
                return 1
            else:
                return gaussian
        elif limit_type == UPPER:
            if value >= limit:
                return 1
            else:
                return gaussian
        else:
            return gaussian
    
    
    
    #Introduces a penalty given structural constraints
    def structural_penalty(self):
        abstract
        
    #
    def make_tunable(self):
        return self
            
    def final(self):
        return self   

    def illegal(self):
        return False
        
#Abstract class for a model which is fully defined by its points (i.e., has no other parameters like symmline)
class Point_Model(Model):
    def __init__(self,*vertices_and_params):
        
        try:
            assert len(vertices_and_params) == len(self.variable_pt_names()) + len(self.variable_param_names())
        except Exception,e:
            print "Only given %d params for %d variable_pts and %d variable_params"%(len(vertices_and_params),len(self.variable_pt_names()),len(self.variable_param_names()))
            assert False
        vertices = vertices_and_params[:len(self.variable_pt_names())]
        params = vertices_and_params[len(self.variable_pt_names()):]
        self.vertices = list(vertices)
        self.scalar_params = list(params)
        
    def variable_pt_names(self):
        return []
    
    #Dict of pts which should be defined relative to another pt. Key = pt, value = pt it's relative to
    def relative_pts(self):
        return {}
    
    def variable_param_names(self):
        return []
        
    
    
    def __getattr__(self,attr):
        pt_names = self.variable_pt_names()
        if attr in pt_names:
            index = pt_names.index(attr)
            return lambda: self.vertices[index]
        param_names = self.variable_param_names()
        if attr in param_names:
            index = param_names.index(attr)
            return lambda: self.scalar_params[index]
        #print "Couldn't find attr %s"%attr
        #return Model.__getattr__(self,attr)
        raise AttributeError, attr
        

                
    def translate(self,trans):
        self.vertices = translate_pts(self.vertices,trans)
        
    def rotate(self,angle,origin=None):
        if not origin:
            origin = self.center()
        self.vertices = rotate_pts(self.vertices,angle,origin)
        
    def scale(self,amt,origin=None):
        if not origin:
            origin = self.center()
        self.vertices = scale_pts(self.vertices,amt,origin)
    
    #Parameters are all of my vertices plus all of my other parameters
    def params(self):
        output = []
        for (name,pt) in [(self.variable_pt_names()[i],self.vertices[i]) for i in range(len(self.vertices))]:
            if name in self.relative_pts().keys():
                rel_pt_name = self.relative_pts()[name]
                rel_pt = self.__getattr__(rel_pt_name)()
                dx = pt_x(pt) - pt_x(rel_pt)
                dy = pt_y(pt) - pt_y(rel_pt)
                output.append(dx)
                output.append(dy)
            else:
                output.append(pt_x(pt))
                output.append(pt_y(pt))
        for param in self.scalar_params:
            output.append(param)
        return output
    
    #Reads in a list of x,y values, and creates a new instance of myself with those points    
    def from_params(self,params):
        init_args = []
        x = None
        point_params = params[:2*len(self.variable_pt_names())]
        scalar_params = params[2*len(self.variable_pt_names()):]
        for i,p in enumerate(point_params):
            if i%2 == 0:
                x = p
            else:
                y = p
                cur_pt_name = self.variable_pt_names()[i/2]
                if cur_pt_name in self.relative_pts().keys():
                    rel_pt_name = self.relative_pts()[cur_pt_name]
                    rel_pt_index = self.variable_pt_names().index(rel_pt_name)
                    rel_pt = init_args[rel_pt_index]
                    x += pt_x(rel_pt)
                    y += pt_y(rel_pt)
                init_args.append((x,y))
        for s in scalar_params:
            init_args.append(s)
        return self.clone(init_args)

    def clone(self,init_args):
        myclone = self.__class__(*init_args)
        myclone.set_image(self.image)
        return myclone
        
    def structural_penalty(self):
        return 1 if self.illegal() else 0
        
    def allow_intersections(self):
        return False
        
    def allow_flipping(self):
        return False
        
    def illegal(self):
            
        if not self.allow_intersections():
            sides = self.sides()
            for i in range(len(sides)):
                for j in range(i,len(sides)):
                    if i != j:
                        if seg_intercept(sides[i],sides[j]) != None:
                            print "Self intersection!"
                            return True
        if not self.allow_flipping():
            sides=  self.sides()
            for i in range(len(sides)):
                for j in range(i-2,i+1):
                    if seg_intercept(sides[i],sides[j]) != None:
                            print "Flipped"
                            return True
                
        return False
        
class Point_Model_Contour_Only_Asymm(Point_Model):
    def __init__(self,*vertices_and_params):
        vertices = list(vertices_and_params)
        self.num_variable_pts = len(vertices)
        Point_Model.__init__(self,*vertices_and_params)
        
    def variable_pt_names(self):
        #return ["pt_%d"%i for i in range(self.num_variable_pts)]
        return ["pt_%d"%i for i in range(13)]
        
    def vertices_full(self):
        return self.vertices
"""
# A model which is defined by fixed points and one foldline  
class Point_Model_Folded(Point_Model):
    #For now, we can easily assume all folds are left to right, and work a sign in later to fix it
    def __init__(self,initial_model,fold_angle,fold_displ):
        self.initial_model = initial_model
        self.vertices = []
        self.scalar_params = [fold_angle,fold_displ]
        print "Angle = %f, Displ = %f"%(fold_angle,fold_displ)
        
    def variable_param_names(self):
        return ["fold_angle","fold_displ"]
        
    def variable_pt_names(self):
        return []
        
    def vertices_full(self):
        print "CALCULATING VERTICES FULL"
        init_vertices_full = self.initial_model.vertices_full()
        foldline = self.foldline()
        perp = perpendicular(foldline,(self.fold_displ(),0.0))
        pts = []
        
        for pt in init_vertices_full:
            if dot_prod(pt,line_vector(perp)) < dot_prod(line_offset(perp),line_vector(perp)):
                pts.append(mirror_pt(pt,foldline))
                #pts.append(pt)
            else:
                pts.append(pt)
        last_inter = None
        offset = 0
        print len(pts)
        for i,seg in enumerate(self.initial_model.sides()):
            inter = seg_intercept(seg,foldline)
            if inter != None:
                pts.insert(i+offset,inter)
                print "INSERTING INTERCEPT"
                if last_inter != None:
                    pts.insert(i+1+offset,last_inter)
                    pts.insert(i+2+offset,inter)
                    offset += 2
                    last_iter = None
                else:
                    last_inter = inter
                offset += 1
        print len(pts)
        print "DONE CALCULATING VERTICES FULL"
        return list(pts)
    
    def contour_mode(self):
        return True
        
    def illegal(self):
        return False
      
    def foldline(self):
        return make_ln_from_pts((self.fold_displ(),0.0),(self.fold_displ()+100*cos(self.fold_angle()),100*sin(self.fold_angle())))
        
    def allow_intersections(self):
        return True
        
    def draw_to_image(self,img,color):
        self.draw_line(img,intercept(self.foldline(),horiz_ln(y=0.0)),intercept(self.foldline(),horiz_ln(y=img.height)),color)
        val = [self.draw_point(img,pt,color) for pt in self.vertices_full()]
        Point_Model.draw_to_image(self,img,color)
        
    def clone(self,*init_args):
        print "Init args are: %s"%init_args
        print len(init_args)
        print init_args[0][0]
        myclone = self.__class__(self.initial_model,*init_args[0])
        myclone.set_image(self.image)
        return myclone
"""

class Orient_Model(Point_Model):
    def __init__(self,initial_model,*params):
        self.initial_model = initial_model
        self.image = initial_model.image
        Point_Model.__init__(self,*params)
        
    def vertices_full(self):
        return self.transformed_model().vertices_full()
    
    def displacement(self):
        return (self.x_displacement(),self.y_displacement())
    
    def variable_param_names(self):
        return["angle"]
        #return ["x_displacement","y_displacement","angle","scale_amt"]
    
    def structural_penalty(self):
        if abs(self.angle()-pi/2) > pi/4:
            return 1
        else:
            return 0
    
    def x_displacement(self):
        return 0
    
    def y_displacement(self):
        return 0
    
    def scale_amt(self):
        return 1
        
    def preferred_delta(self):
        return 0.1
        
    def transformed_model(self):
        model_new = self.initial_model.from_params(self.initial_model.params())
        model_new.translate(self.displacement())
        model_new.rotate(self.angle()-pi/2,model_new.center())
        model_new.scale(self.scale_amt())
        return model_new
        
    def clone(self,init_args):
        myclone = self.__class__(self.initial_model,*init_args)
        myclone.set_image(self.image)
        return myclone
"""        
class Fine_Tune_Model(Point_Model):
    def __init__(self,initial_model,tunable_indices,*params):
        self.initial_model = initial_model
        self.image = initial_model.image
        self.initial_vertices_full = initial_model.vertices_full()
        self.tunable_indices = tunable_indices
        Point_Model.__init__(self,*params)
        
    def vertices_full(self):
        init_value = self.initial_vertices_full
        for i in self.tunable_indices:
            init_value[i] = self.__getattr__("pt_%d"%i)
        
    def variable_pt_names(self):
        return ["pt_%d"%i for i in self.tunable_indices]
     
    def structural_penalty(self):
        return self.in
"""
# A model which is defined by fixed points and one foldline  
class Point_Model_Folded(Point_Model):
    #For now, we can easily assume all folds are left to right, and work a sign in later to fix it
    def __init__(self,initial_model,*pts):
        self.initial_model = initial_model
        self.image = None
        Point_Model.__init__(self,*pts)
        
   # def variable_param_names(self):
   #     return ["fold_angle","fold_displ"]
        
    def variable_pt_names(self):
        return ["fold_bottom","fold_top"]
        
    def relative_pts(self):
        return {"fold_top":"fold_bottom"}
    
    def set_image(self,image):
        self.initial_model.set_image(image)
        Point_Model.set_image(self,image)
        
    def vertices_full(self):

        init_vertices_full = self.initial_model.vertices_full()
        foldline = self.foldline()
        perp = perpendicular(foldline,line_offset(foldline))
        pts = []
        
        for pt in init_vertices_full:
            if dot_prod(pt,line_vector(perp)) < dot_prod(line_offset(perp),line_vector(perp)):
                pts.append(mirror_pt(pt,foldline))
                #pts.append(pt)
            else:
                pts.append(pt)
        last_inter = None
        offset = 0
        for i,seg in enumerate(self.initial_model.sides()):
            inter = seg_intercept(seg,foldline)
            if inter != None:
                pts.insert(i+offset,inter)

                if last_inter != None:
                    pts.insert(i+1+offset,last_inter)
                    pts.insert(i+2+offset,inter)
                    offset += 2
                    last_iter = None
                else:
                    last_inter = inter
                offset += 1

        return list(pts)
    
    def contour_mode(self):
        return True
        
    def illegal(self):
        return False
      
    def foldline(self):
        return make_ln_from_pts(self.fold_bottom(),self.fold_top())
        
    def allow_intersections(self):
        return True
        
    def draw_to_image(self,img,color):
        self.draw_line(img,intercept(self.foldline(),horiz_ln(y=0.0)),intercept(self.foldline(),horiz_ln(y=img.height)),color)
        val = [self.draw_point(img,pt,color) for pt in self.vertices_full()]
        Point_Model.draw_to_image(self,img,color)
        
    def clone(self,init_args):
        myclone = self.__class__(self.initial_model,*init_args)
        myclone.set_image(self.image)
        return myclone
        
    def preferred_delta(self):
        return 5.0
        
class Point_Model_Folded_Robust(Point_Model_Folded):


    def params(self):
        params = Point_Model_Folded.params(self)
        params.extend(self.initial_model.params())
        return params
        
    def from_params(self,params):
        params = list(params)
        my_params = []
        for i in range(len(Point_Model_Folded.params(self))):
            my_params.append(params.pop(0))
        newmodel = Point_Model_Folded.from_params(self,my_params)
        new_initial_model = self.initial_model.from_params(params)
        newmodel.initial_model = new_initial_model
        newmodel.set_image(self.image)
        return newmodel
        
    def structural_penalty(self):
        return self.initial_model.structural_penalty()

class Point_Model_Variable_Symm(Point_Model):
    
    def __init__(self,symmetric,*vertices_and_params):
        self.symmetric = symmetric
        Point_Model.__init__(self,*vertices_and_params)
    
    
    def variable_pt_names(self):
        if self.symmetric:
            return self.symmetric_variable_pt_names()
        else:
            return self.symmetric_variable_pt_names() + sorted(self.mirrored_pts().values())
            
    def variable_param_names(self):
        if self.symmetric:
            return self.symmetric_variable_param_names()
        else:
            return self.symmetric_variable_param_names() + sorted(self.mirrored_params().values())
    
    #Modifies __getattr__ to lookup mirrored points and params
    def __getattr__(self,attr):
        if self.symmetric:
            for pt1_name,pt2_name in self.mirrored_pts().items():
                if pt2_name == attr:
                    pt1 = self.__getattr__(pt1_name)()
                    pt2 = mirror_pt(pt1,self.axis_of_symmetry())
                    return lambda : pt2
            for param1_name,param2_name in self.mirrored_params().items():
                if param2_name == attr:
                    param1 = self.__getattr__(param1_name)()
                    param2 = param1
                    return lambda : param2
        #Otherwise, proceed as usual
        return Point_Model.__getattr__(self,attr)
                
    
    # A list of all variable pts in the symmetric model        
    def symmetric_variable_pt_names(self):
        return []
        
    def symmetric_variable_param_names(self):
        return []
    
    # A dictionary of the name of the variable pt, and its mirrored equivalent
    def mirrored_pts(self):
        return {}
        
    #A mirrored parameter is identical to its counterpart
    def mirrored_params(self):
        return {}
    
    # The line of symmetry about which all pts are mirrored
    def axis_of_symmetry(self):
        abstract
        
    def make_asymm(self):
        if not self.symmetric:
            return self
        pts = []
        pts.extend(self.vertices)
        """
        for pt1_name in self.variable_pt_names():
            if pt1_name in self.mirrored_pts().keys():
                pt2_name = self.mirrored_pts()[pt1_name]
                pt2 = self.__getattr__(pt2_name)()
                pts.append(pt2)
        """
        for pt1_name,pt2_name in sorted(self.mirrored_pts().items(),key = lambda (k,v): v):
            pt2 = self.__getattr__(pt2_name)()
            pts.append(pt2)
            
        params = []
        params.extend(self.scalar_params)
        for param1_name,param2_name in sorted(self.mirrored_params().items(),key = lambda (k,v): v):
            param2 = self.__getattr__(param2_name)()
            params.append(param2)
        init_args = pts + params    
        asymm = self.__class__(False,*init_args)
        asymm.set_image(self.image)
        return asymm
        
    def free(self):
        model = Point_Model_Contour_Only_Asymm(*self.vertices_full())
        model.set_image(self.image)
        return model
    
    def clone(self,init_args):
        myclone = self.__class__(self.symmetric,*init_args)
        myclone.set_image(self.image)
        return myclone
        
###
# Defining some models
###

class Model_Towel(Point_Model_Variable_Symm):
    def vertices_full(self):
        return [self.bottom_left(),self.top_left(),self.top_right(),self.bottom_right()]
        
    def symmetric_variable_pt_names(self):
        return ["bottom_left","top_left","top_right","bottom_right"]
        
    def axis_of_symmetry(self):
        return make_ln_from_pts(pt_scale(pt_sum(self.bottom_left(),self.bottom_right()),0.5),pt_scale(pt_sum(self.top_left(),self.top_right()),0.5))



class Model_Pants_Skel(Point_Model_Variable_Symm):

        
    def vertices_full(self):
        return [self.left_leg_right(),self.left_leg_left(),self.top_left(),self.top_right(),self.right_leg_right(),self.right_leg_left(),self.crotch()]
        
    def symmetric_variable_pt_names(self):
        return ["mid_center","top_center","mid_left","left_leg_center","left_leg_left"]
        
    def mirrored_pts(self):
        return {"mid_left":"mid_right", "left_leg_center":"right_leg_center","left_leg_left":"right_leg_right"}
        
    def axis_of_symmetry(self):
        return make_ln_from_pts(self.mid_center(),self.top_center())
    
    """
    Defining other points
    """    
    def crotch(self):
        ln = perpendicular(make_ln_from_pts(self.top_center(),self.mid_center()),self.mid_center())
        return mirror_pt(self.top_center(),ln)
        
    def left_leg_right(self):
        ln = perpendicular(make_ln_from_pts(self.left_leg_left(),self.left_leg_center()),self.left_leg_center())
        return mirror_pt(self.left_leg_left(),ln)
        
    def right_leg_left(self):
        ln = perpendicular(make_ln_from_pts(self.right_leg_right(),self.right_leg_center()),self.right_leg_center())
        return mirror_pt(self.right_leg_right(),ln)   
    
    def top_left(self):
        displ = pt_sum( pt_diff(self.top_center(),self.mid_center()), pt_diff(self.left_leg_left(),self.left_leg_center()))
        return translate_pt(self.mid_left(),displ)
        
    def top_right(self):
        displ = pt_sum( pt_diff(self.top_center(),self.mid_center()), pt_diff(self.right_leg_right(),self.right_leg_center()))
        return translate_pt(self.mid_right(),displ)
        
    def allow_intersections(self):
        return False
        
    def structural_penalty(self):
        penalty = Point_Model_Variable_Symm.structural_penalty(self)
        """
        penalty += self.constrain(pt_distance(self.mid_left(),self.mid_right()),pt_distance(self.top_left(),self.top_right()),UPPER,0.0)
        if seg_intercept(make_seg(self.top_left(),self.left_leg_left()),make_seg(self.mid_left(),self.mid_right())):
            penalty += 1
        if seg_intercept(make_seg(self.top_right(),self.right_leg_right()),make_seg(self.mid_left(),self.mid_right())):
            penalty += 1
        """
        return penalty
        
        
    def draw_to_image(self,img,color):
        Point_Model_Variable_Symm.draw_to_image(self,img,color)
        
        #Draw skeletal frame
        self.draw_point(img,self.crotch(),color)
        self.draw_point(img,self.top_center(),color)
        self.draw_line(img,self.crotch(),self.top_center(),color)
        self.draw_point(img,self.mid_left(),color)
        self.draw_point(img,self.mid_right(),color)
        self.draw_line(img,self.mid_left(),self.mid_right(),color)
        self.draw_point(img,self.left_leg_center(),color)
        self.draw_point(img,self.right_leg_center(),color)
        self.draw_line(img,self.mid_left(),self.left_leg_center(),color)
        self.draw_line(img,self.mid_right(),self.right_leg_center(),color)
        
      
class Model_Pants_Contour_Only(Point_Model_Contour_Only_Asymm):

    def variable_pt_names(self):
        #return ["pt_%d"%i for i in range(self.num_variable_pts)]
        return ["pt_%d"%i for i in range(7)]

        
class Model_Pants_Skel_Extended(Model_Pants_Skel):
    
    def vertices_full(self):
        return [self.left_leg_right(),self.left_leg_left(),self.top_left(),self.top_right(),self.right_leg_right(),self.right_leg_left(),self.crotch()]
        
    def symmetric_variable_pt_names(self):
        return ["mid_center","top_center","mid_left","left_leg_center","left_leg_left","top_left"]
        
    def mirrored_pts(self):
        return {"mid_left":"mid_right", "left_leg_center":"right_leg_center","left_leg_left":"right_leg_right"}
    
    def relative_pts(self):
        return {"left_leg_left":"left_leg_center","right_leg_right":"right_leg_center","top_left":"top_center","top_right":"top_center"}
        
    def top_left(self):
        return self.__getattr__("top_left")()
        
    def top_right(self):
        ln = perpendicular(make_ln_from_pts(self.top_left(),self.top_center()),self.top_center())
        return mirror_pt(self.top_left(),ln)
        #return mirror_pt(self.top_left(),self.axis_of_symmetry())
        
    def structural_penalty(self):
        penalty = Model_Pants_Skel.structural_penalty(self)
        #penalty += self.constrain( pt_distance(pt_scale(pt_sum(self.top_left(),self.top_right()),0.5),self.top_center()),pt_distance(self.top_center(),self.top_left())*0.15,UPPER,0.0)    
        return penalty
        



#Generic class which makes no assertions about what the variable points are
class Model_Shirt_Generic(Point_Model_Variable_Symm):
    
    def vertices_full(self):
        return [self.bottom_left(),self.left_armpit(),self.left_sleeve_bottom(),self.left_sleeve_top(),self.left_shoulder_top(),self.left_collar(),self.spine_top()
               ,self.right_collar(),self.right_shoulder_top(),self.right_sleeve_top(),self.right_sleeve_bottom(),self.right_armpit(),self.bottom_right()]
               
        
    
    #def right_collar(self):
    #    return mirror_pt(self.left_collar(),self.axis_of_symmetry())
        
    def axis_of_symmetry(self):
        return make_ln_from_pts(self.spine_bottom(),self.spine_top())
    """
    Defining other points
    """
    def shoulder_spine_junction(self):
        return intercept(make_ln_from_pts(self.left_shoulder_joint(),self.right_shoulder_joint()),make_ln_from_pts(self.spine_bottom(),self.spine_top()))
    
    def left_armpit(self):
        ln = perpendicular(make_ln_from_pts(self.left_shoulder_top(),self.left_shoulder_joint()),self.left_shoulder_joint())
        return mirror_pt(self.left_shoulder_top(),ln)
        #return mirror_pt(self.left_shoulder_top(),self.horiz_frame())
        
    def right_armpit(self):
        ln = perpendicular(make_ln_from_pts(self.right_shoulder_top(),self.right_shoulder_joint()),self.right_shoulder_joint())
        return mirror_pt(self.right_shoulder_top(),ln)
        #return mirror_pt(self.right_shoulder_top(),self.horiz_frame())
        
    def left_sleeve_bottom(self):
        ln = perpendicular(make_ln_from_pts(self.left_sleeve_top(),self.left_sleeve_center()),self.left_sleeve_center())
        return mirror_pt(self.left_sleeve_top(),ln)
        #return mirror_pt(self.left_sleeve_top(),self.left_sleeve_axis())
        
    def right_sleeve_bottom(self):
        ln = perpendicular(make_ln_from_pts(self.right_sleeve_top(),self.right_sleeve_center()),self.right_sleeve_center())
        return mirror_pt(self.right_sleeve_top(),ln)
        #return mirror_pt(self.right_sleeve_top(),self.right_sleeve_axis())
        
    
        
    def bottom_right(self):
        #displ = pt_diff(self.right_shoulder_joint(),self.left_shoulder_joint())
        #return pt_sum(self.spine_bottom(),pt_scale(displ,0.5))
        ln = perpendicular(make_ln_from_pts(self.bottom_left(),self.spine_bottom()),self.spine_bottom())
        return mirror_pt(self.bottom_left(),ln)
        
    def horiz_frame(self):
        return make_ln_from_pts(self.left_shoulder_joint(),self.right_shoulder_joint())
        
    def bottom_edge(self):
        return make_ln(offset=self.spine_bottom(), vect= line_vector(self.horiz_frame()))
        
    def left_sleeve_axis(self):
        return make_ln_from_pts(self.left_sleeve_center(),self.left_shoulder_joint())
        
    def right_sleeve_axis(self):
        return make_ln_from_pts(self.right_sleeve_center(),self.right_shoulder_joint())
        
    """
    Defining drawing
    """
    def draw_to_image(self,img,color):
        Point_Model_Variable_Symm.draw_to_image(self,img,color)
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
        
        
    def illegal(self):
        return Point_Model_Variable_Symm.illegal(self)

class Model_Tee_Generic(Model_Shirt_Generic):
    def structural_penalty(self):
        penalty = 0
        penalty += Model_Shirt_Generic.structural_penalty(self)
        
        #Compute a few useful values
        spine_axis = pt_diff(self.spine_top(),self.spine_bottom())
        horiz_axis = pt_diff(self.right_shoulder_joint(),self.left_shoulder_joint())
        l_shoulder_axis = pt_diff(self.left_shoulder_top(),self.left_armpit())
        r_shoulder_axis = pt_diff(self.right_shoulder_top(),self.right_armpit())
        l_side_axis = pt_diff(self.left_armpit(),self.bottom_left())
        r_side_axis = pt_diff(self.right_armpit(),self.bottom_right())
        l_sleeve_axis = pt_diff(self.left_shoulder_joint(),self.left_sleeve_center())
        r_sleeve_axis = pt_diff(self.right_shoulder_joint(),self.right_sleeve_center())
        l_sleeve_side = pt_diff(self.left_sleeve_top(),self.left_sleeve_bottom())
        r_sleeve_side = pt_diff(self.right_sleeve_top(),self.right_sleeve_bottom())
        bottom_axis = pt_diff(self.bottom_left(),self.bottom_right())
        l_collar_side = pt_diff(self.left_collar(),self.spine_top())
        r_collar_side = pt_diff(self.right_collar(),self.spine_top())
        ANGULAR_SIGMA = 0.0
        DOT_PROD_SIGMA = 0.0
        PROPORTIONAL_SIGMA = 0.0
        penalty += self.constrain(angle_between(l_side_axis,l_shoulder_axis),pi/8,UPPER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(r_side_axis,r_shoulder_axis),pi/8,UPPER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(spine_axis,horiz_axis),3*pi/8,LOWER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(spine_axis,horiz_axis),5*pi/8,UPPER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(l_shoulder_axis,spine_axis),pi/15,UPPER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(r_shoulder_axis,spine_axis),pi/15,UPPER,ANGULAR_SIGMA)
        penalty += self.constrain(vect_length(l_shoulder_axis)/vect_length(horiz_axis),0.1,LOWER,PROPORTIONAL_SIGMA)
        penalty += self.constrain(vect_length(r_shoulder_axis)/vect_length(horiz_axis),0.1,LOWER,PROPORTIONAL_SIGMA)
        penalty += self.constrain(dot_prod(self.shoulder_spine_junction(),spine_axis),dot_prod(self.spine_top(),spine_axis),UPPER,DOT_PROD_SIGMA)
        penalty += self.constrain(dot_prod(self.left_shoulder_joint(),horiz_axis),dot_prod(self.right_shoulder_joint(),horiz_axis),UPPER,DOT_PROD_SIGMA)
        penalty += self.constrain(dot_prod(self.right_armpit(),spine_axis),dot_prod(self.right_shoulder_joint(),spine_axis),UPPER,DOT_PROD_SIGMA)
        penalty += self.constrain(dot_prod(self.left_armpit(),spine_axis),dot_prod(self.left_shoulder_joint(),spine_axis),UPPER,DOT_PROD_SIGMA)
        penalty += self.constrain(dot_prod(self.bottom_left(),horiz_axis),dot_prod(self.bottom_right(),horiz_axis),UPPER,DOT_PROD_SIGMA)
        #Make sleeve widths proportional
        penalty += self.constrain(vect_length(l_sleeve_side)/vect_length(r_sleeve_side),0.8,LOWER,PROPORTIONAL_SIGMA)
        penalty += self.constrain(vect_length(l_sleeve_side)/vect_length(r_sleeve_side),1.2,UPPER,PROPORTIONAL_SIGMA)
        
        #Sleeve width can't be more than half its length for sweaters
        #penalty += self.constrain(vect_length(l_sleeve_side)/vect_length(l_sleeve_axis),0.75,UPPER,PROPORTIONAL_SIGMA)
        #penalty += self.constrain(vect_length(r_sleeve_side)/vect_length(r_sleeve_axis),0.75,UPPER,PROPORTIONAL_SIGMA)
        
        #Make collar go upwards
        penalty += self.constrain(dot_prod(self.left_collar(),spine_axis),dot_prod(self.spine_top(),spine_axis),LOWER,DOT_PROD_SIGMA)
        penalty += self.constrain(dot_prod(self.right_collar(),spine_axis),dot_prod(self.spine_top(),spine_axis),LOWER,DOT_PROD_SIGMA)
        penalty += self.constrain(dot_prod(self.left_collar(),horiz_axis),dot_prod(self.right_collar(),horiz_axis),UPPER,DOT_PROD_SIGMA)

        #Constrain bottom corners to be roughly 90 degrees
        penalty += self.constrain(angle_between(l_side_axis,bottom_axis),3*pi/8,LOWER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(l_side_axis,bottom_axis),5*pi/8,UPPER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(r_side_axis,bottom_axis),3*pi/8,LOWER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(r_side_axis,bottom_axis),5*pi/8,UPPER,ANGULAR_SIGMA)
        #Sleeve angles should be close to 90 degrees
        penalty += self.constrain(angle_between(l_sleeve_axis,l_sleeve_side),2*pi/8,LOWER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(l_sleeve_axis,l_sleeve_side),6*pi/8,UPPER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(r_sleeve_axis,r_sleeve_side),2*pi/8,LOWER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(r_sleeve_axis,r_sleeve_side),6*pi/8,UPPER,ANGULAR_SIGMA)
        #Don't let the sleeve collapse
        penalty += self.constrain(angle_between(l_sleeve_axis,l_side_axis),pi/8,LOWER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(r_sleeve_axis,r_side_axis),pi/8,LOWER,ANGULAR_SIGMA)
        #Make sleeve angles similar
        #penalty += self.constrain(angle_between(l_sleeve_axis,l_sleeve_side),angle_between(r_sleeve_axis,r_sleeve_side)+pi/12,UPPER,ANGULAR_SIGMA)
        #penalty += self.constrain(angle_between(l_sleeve_axis,l_sleeve_side),angle_between(r_sleeve_axis,r_sleeve_side)-pi/12,LOWER,ANGULAR_SIGMA)
        #Make shoulder always below collar
        penalty += self.constrain(dot_prod(self.left_shoulder_top(),spine_axis),dot_prod(self.left_collar(),spine_axis),UPPER,PROPORTIONAL_SIGMA)
        penalty += self.constrain(dot_prod(self.right_shoulder_top(),spine_axis),dot_prod(self.right_collar(),spine_axis),UPPER,PROPORTIONAL_SIGMA)
        #Make distance from armpit to shoulder at least as great as sleeve width
        penalty += self.constrain(vect_length(l_shoulder_axis),0.75*vect_length(l_sleeve_side),LOWER,DOT_PROD_SIGMA)
        penalty += self.constrain(vect_length(r_shoulder_axis),0.75*vect_length(r_sleeve_side),LOWER,DOT_PROD_SIGMA)

        #Make the center be roughtly...well...centered
        penalty += self.constrain(pt_distance(self.left_shoulder_top(),self.spine_top()) / pt_distance(self.right_shoulder_top(),self.spine_top()),0.5,LOWER,PROPORTIONAL_SIGMA)
        penalty += self.constrain(pt_distance(self.left_shoulder_top(),self.spine_top()) / pt_distance(self.right_shoulder_top(),self.spine_top()),1.5,UPPER,PROPORTIONAL_SIGMA)
        return penalty

class Model_Long_Shirt_Generic(Model_Shirt_Generic):
    def structural_penalty(self):
        penalty = 0
        penalty += Model_Shirt_Generic.structural_penalty(self)
        
        #Compute a few useful values
        spine_axis = pt_diff(self.spine_top(),self.spine_bottom())
        horiz_axis = pt_diff(self.right_shoulder_joint(),self.left_shoulder_joint())
        l_shoulder_axis = pt_diff(self.left_shoulder_top(),self.left_armpit())
        r_shoulder_axis = pt_diff(self.right_shoulder_top(),self.right_armpit())
        l_side_axis = pt_diff(self.left_armpit(),self.bottom_left())
        r_side_axis = pt_diff(self.right_armpit(),self.bottom_right())
        l_sleeve_axis = pt_diff(self.left_shoulder_joint(),self.left_sleeve_center())
        r_sleeve_axis = pt_diff(self.right_shoulder_joint(),self.right_sleeve_center())
        l_sleeve_side = pt_diff(self.left_sleeve_top(),self.left_sleeve_bottom())
        r_sleeve_side = pt_diff(self.right_sleeve_top(),self.right_sleeve_bottom())
        bottom_axis = pt_diff(self.bottom_left(),self.bottom_right())
        l_collar_side = pt_diff(self.left_collar(),self.spine_top())
        r_collar_side = pt_diff(self.right_collar(),self.spine_top())
        ANGULAR_SIGMA = 0.0
        DOT_PROD_SIGMA = 0.0
        PROPORTIONAL_SIGMA = 0.0
        penalty += self.constrain(angle_between(l_side_axis,l_shoulder_axis),pi/8,UPPER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(r_side_axis,r_shoulder_axis),pi/8,UPPER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(spine_axis,horiz_axis),3*pi/8,LOWER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(spine_axis,horiz_axis),5*pi/8,UPPER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(l_shoulder_axis,spine_axis),pi/15,UPPER,ANGULAR_SIGMA)
        #penalty += self.constrain(angle_between(l_shoulder_axis,l_sleeve_axis),pi/8,LOWER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(r_shoulder_axis,spine_axis),pi/15,UPPER,ANGULAR_SIGMA)
        #penalty += self.constrain(angle_between(r_shoulder_axis,r_sleeve_axis),pi/8,LOWER,ANGULAR_SIGMA)
        penalty += self.constrain(vect_length(l_shoulder_axis)/vect_length(horiz_axis),0.1,LOWER,PROPORTIONAL_SIGMA)
        penalty += self.constrain(vect_length(r_shoulder_axis)/vect_length(horiz_axis),0.1,LOWER,PROPORTIONAL_SIGMA)
        penalty += self.constrain(dot_prod(self.shoulder_spine_junction(),spine_axis),dot_prod(self.spine_top(),spine_axis),UPPER,DOT_PROD_SIGMA)
        penalty += self.constrain(dot_prod(self.left_shoulder_joint(),horiz_axis),dot_prod(self.right_shoulder_joint(),horiz_axis),UPPER,DOT_PROD_SIGMA)
        penalty += self.constrain(dot_prod(self.right_armpit(),spine_axis),dot_prod(self.right_shoulder_joint(),spine_axis),UPPER,DOT_PROD_SIGMA)
        penalty += self.constrain(dot_prod(self.left_armpit(),spine_axis),dot_prod(self.left_shoulder_joint(),spine_axis),UPPER,DOT_PROD_SIGMA)
        penalty += self.constrain(dot_prod(self.bottom_left(),horiz_axis),dot_prod(self.bottom_right(),horiz_axis),UPPER,DOT_PROD_SIGMA)
        #Make sleeve widths proportional
        penalty += self.constrain(vect_length(l_sleeve_side)/vect_length(r_sleeve_side),0.8,LOWER,PROPORTIONAL_SIGMA)
        penalty += self.constrain(vect_length(l_sleeve_side)/vect_length(r_sleeve_side),1.2,UPPER,PROPORTIONAL_SIGMA)
        #Sleeve width can't be more than half its length for sweaters
        penalty += self.constrain(vect_length(l_sleeve_side)/vect_length(l_sleeve_axis),0.75,UPPER,PROPORTIONAL_SIGMA)
        penalty += self.constrain(vect_length(r_sleeve_side)/vect_length(r_sleeve_axis),0.75,UPPER,PROPORTIONAL_SIGMA)
        #Make sleeve lengths proportional
        #penalty += self.constrain(vect_length(l_sleeve_axis)/vect_length(r_sleeve_axis),0.5,LOWER,PROPORTIONAL_SIGMA)
        #penalty += self.constrain(vect_length(l_sleeve_axis)/vect_length(r_sleeve_axis),1.5,UPPER,PROPORTIONAL_SIGMA)
        #Make collar go upwards
        penalty += self.constrain(dot_prod(self.left_collar(),spine_axis),dot_prod(self.spine_top(),spine_axis),LOWER,DOT_PROD_SIGMA)
        penalty += self.constrain(dot_prod(self.right_collar(),spine_axis),dot_prod(self.spine_top(),spine_axis),LOWER,DOT_PROD_SIGMA)
        penalty += self.constrain(dot_prod(self.left_collar(),horiz_axis),dot_prod(self.right_collar(),horiz_axis),UPPER,DOT_PROD_SIGMA)
        #penalty += self.constrain(vect_length(l_collar_side)/vect_length(bottom_axis),0.05,LOWER,PROPORTIONAL_SIGMA)
        #penalty += self.constrain(vect_length(r_collar_side)/vect_length(bottom_axis),0.05,LOWER,PROPORTIONAL_SIGMA)
        #Constrain bottom corners to be roughly 90 degrees
        penalty += self.constrain(angle_between(l_side_axis,bottom_axis),3*pi/8,LOWER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(l_side_axis,bottom_axis),5*pi/8,UPPER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(r_side_axis,bottom_axis),3*pi/8,LOWER,ANGULAR_SIGMA)
        penalty += self.constrain(angle_between(r_side_axis,bottom_axis),5*pi/8,UPPER,ANGULAR_SIGMA)
        #Sleeve angles should be close to 90 degrees
        #penalty += self.constrain(angle_between(l_sleeve_axis,l_sleeve_side),2*pi/8,LOWER,ANGULAR_SIGMA)
        #penalty += self.constrain(angle_between(l_sleeve_axis,l_sleeve_side),6*pi/8,UPPER,ANGULAR_SIGMA)
        #penalty += self.constrain(angle_between(r_sleeve_axis,r_sleeve_side),2*pi/8,LOWER,ANGULAR_SIGMA)
        #penalty += self.constrain(angle_between(r_sleeve_axis,r_sleeve_side),6*pi/8,UPPER,ANGULAR_SIGMA)
        #Make sleeve angles similar
        #penalty += self.constrain(angle_between(l_sleeve_axis,l_sleeve_side),angle_between(r_sleeve_axis,r_sleeve_side)+pi/12,UPPER,ANGULAR_SIGMA)
        #penalty += self.constrain(angle_between(l_sleeve_axis,l_sleeve_side),angle_between(r_sleeve_axis,r_sleeve_side)-pi/12,LOWER,ANGULAR_SIGMA)
        #Make shoulder always below collar
        penalty += self.constrain(dot_prod(self.left_shoulder_top(),spine_axis),dot_prod(self.left_collar(),spine_axis),UPPER,PROPORTIONAL_SIGMA)
        penalty += self.constrain(dot_prod(self.right_shoulder_top(),spine_axis),dot_prod(self.right_collar(),spine_axis),UPPER,PROPORTIONAL_SIGMA)
        #Make distance from armpit to shoulder at least as great as sleeve width
        penalty += self.constrain(vect_length(l_shoulder_axis),0.75*vect_length(l_sleeve_side),LOWER,DOT_PROD_SIGMA)
        penalty += self.constrain(vect_length(r_shoulder_axis),0.75*vect_length(r_sleeve_side),LOWER,DOT_PROD_SIGMA)
        #Make shoulder sides be equivalent
        """
        penalty += self.constrain(vect_length(l_shoulder_axis)/vect_length(r_shoulder_axis),1.5,UPPER,PROPORTIONAL_SIGMA)
        penalty += self.constrain(vect_length(r_shoulder_axis)/vect_length(l_shoulder_axis),1.5,UPPER,PROPORTIONAL_SIGMA)
        """
        #Make the center be roughtly...well...centered
        penalty += self.constrain(pt_distance(self.left_shoulder_top(),self.spine_top()) / pt_distance(self.right_shoulder_top(),self.spine_top()),0.5,LOWER,PROPORTIONAL_SIGMA)
        penalty += self.constrain(pt_distance(self.left_shoulder_top(),self.spine_top()) / pt_distance(self.right_shoulder_top(),self.spine_top()),1.5,UPPER,PROPORTIONAL_SIGMA)
        #Make sure the sleeve doesn't collapse
        penalty += self.constrain(pt_distance(self.left_sleeve_top(),self.left_shoulder_top())/pt_distance(self.left_sleeve_bottom(),self.left_armpit()),0.66,LOWER,PROPORTIONAL_SIGMA)
        penalty += self.constrain(pt_distance(self.left_sleeve_top(),self.left_shoulder_top())/pt_distance(self.left_sleeve_bottom(),self.left_armpit()),1.5,UPPER,PROPORTIONAL_SIGMA)
        return penalty

class Model_Tee_Skel(Model_Tee_Generic):
     
    def symmetric_variable_pt_names(self):
        return ["spine_bottom","spine_top","left_collar","left_shoulder_joint","left_shoulder_top","left_sleeve_center","left_sleeve_top","bottom_left"]
        
    def mirrored_pts(self):
        return {"left_shoulder_joint":"right_shoulder_joint","left_shoulder_top":"right_shoulder_top","left_sleeve_center":"right_sleeve_center","left_sleeve_top":"right_sleeve_top","left_collar":"right_collar"}

    def relative_pts(self):
        return {"left_collar":"spine_top","left_shoulder_top":"left_shoulder_joint","right_shoulder_top":"right_shoulder_joint",
        "left_sleeve_top":"left_sleeve_center","right_sleeve_top":"right_sleeve_center","bottom_left":"spine_bottom","bottom_right":"spine_bottom"}

    def allow_intersections(self):
        return False
        
    def left_sleeve_bottom(self):
        #(dx,dy) = pt_diff(self.left_sleeve_center(),self.left_sleeve_top())
        #return pt_sum(self.left_sleeve_center(),(dx,dy))
        ln = perpendicular(make_ln_from_pts(self.left_sleeve_top(),self.left_sleeve_center()),self.left_sleeve_center())
        return mirror_pt(self.left_sleeve_top(),ln)
           
    def right_sleeve_bottom(self):
        #(dx,dy) = pt_diff(self.right_sleeve_center(),self.right_sleeve_top())
        #return pt_sum(self.right_sleeve_center(),(dx,dy))
        ln = perpendicular(make_ln_from_pts(self.right_sleeve_top(),self.right_sleeve_center()),self.right_sleeve_center())
        return mirror_pt(self.right_sleeve_top(),ln)
        
    #def bottom_left(self):
    #    return pt_sum(self.spine_bottom(),pt_diff(self.left_shoulder_joint(),self.shoulder_spine_junction()))
        
    def preferred_delta(self):
        return 10.0
        
class Model_Tee_Skel_No_Skew(Model_Tee_Skel):
    def symmetric_variable_pt_names(self):
        return ["spine_bottom","spine_top","left_collar","left_shoulder_joint","left_shoulder_top","left_sleeve_center","bottom_left"]
    
    def symmetric_variable_param_names(self):
        return ["left_sleeve_width"]
    
    def mirrored_pts(self):
        return {"left_shoulder_joint":"right_shoulder_joint","left_shoulder_top":"right_shoulder_top","left_sleeve_center":"right_sleeve_center","left_collar":"right_collar"}
        
    def mirrored_params(self):
        return {"left_sleeve_width":"right_sleeve_width"}
        
    def left_sleeve_axis(self):
        angle = self.left_sleeve_angle()
        
        horiz_axis = make_ln_from_pts(self.left_shoulder_joint(),self.shoulder_spine_junction())
        straight_pt = extrapolate(horiz_axis,-1)
        new_pt = rotate_pt(straight_pt,-1*angle,self.left_shoulder_joint())
        return make_ln_from_pts(self.left_shoulder_joint(),new_pt)
        
    def right_sleeve_axis(self):
        angle = self.right_sleeve_angle()
        
        horiz_axis = make_ln_from_pts(self.right_shoulder_joint(),self.shoulder_spine_junction())
        straight_pt = extrapolate(horiz_axis,-1)
        new_pt = rotate_pt(straight_pt,angle,self.right_shoulder_joint())
        return make_ln_from_pts(self.right_shoulder_joint(),new_pt)
    """
    def left_sleeve_center(self):
        return extrapolate(self.left_sleeve_axis(),abs(self.left_sleeve_length()))
    
    def right_sleeve_center(self):
        return extrapolate(self.right_sleeve_axis(),abs(self.right_sleeve_length()))
    """    
    def left_sleeve_top(self):
        straight_pt = extrapolate(self.left_sleeve_axis(),abs(self.left_sleeve_length()) + abs(self.left_sleeve_width())/2.0)
        return rotate_pt(straight_pt,pi/2,self.left_sleeve_center())
        
    def right_sleeve_top(self):
        straight_pt = extrapolate(self.right_sleeve_axis(),abs(self.right_sleeve_length()) + abs(self.right_sleeve_width())/2.0)
        return rotate_pt(straight_pt,-pi/2,self.right_sleeve_center())   
        
    def left_sleeve_angle(self):
        return angle_between(pt_diff(self.left_shoulder_joint(),self.shoulder_spine_junction()),pt_diff(self.left_sleeve_center(),self.left_shoulder_joint()))
        
    def right_sleeve_angle(self):
        return angle_between(pt_diff(self.right_shoulder_joint(),self.shoulder_spine_junction()),pt_diff(self.right_sleeve_center(),self.right_shoulder_joint()))
        
    def left_sleeve_length(self):
        return pt_distance(self.left_sleeve_center(),self.left_shoulder_joint())
    def right_sleeve_length(self):
        return pt_distance(self.right_sleeve_center(),self.right_shoulder_joint())
        
    def make_tunable(self):
        init_model = self
        return Model_Tee_Tunable(init_model,self.left_armpit(),self.left_sleeve_bottom(),self.left_sleeve_top(),self.right_armpit(),self.right_sleeve_bottom(),self.right_sleeve_top())

class Model_Tee_Tunable(Model_Tee_Generic):
    def __init__(self,init_model,*pts):
        self.initial_model = init_model
        self.image = init_model.image
        Model_Tee_Generic.__init__(self,False,*pts)
        
    def symmetric_variable_pt_names(self):
        return ["left_armpit","left_sleeve_bottom","left_sleeve_top","right_armpit","right_sleeve_bottom","right_sleeve_top"]
        
    def vertices_full(self):
        return Model_Tee_Generic.vertices_full(self)
        
    def symmetric_variable_param_names(self):
        return []
        
    def left_sleeve_bottom(self):
        return self.__getattr__("left_sleeve_bottom")()
    def right_sleeve_bottom(self):
        return self.__getattr__("right_sleeve_bottom")()
        
    def left_armpit(self):
        return self.__getattr__("left_armpit")()
    def right_armpit(self):
        return self.__getattr__("right_armpit")()
    
    def left_sleeve_center(self):
        return pt_scale(pt_sum(self.left_sleeve_top(),self.left_sleeve_bottom()),0.5)
    def right_sleeve_center(self):
        return pt_scale(pt_sum(self.right_sleeve_top(),self.right_sleeve_bottom()),0.5)
        
    def final(self):
        self.initial_model.image = None
        self.image = None
        return self
        
    def __getattr__(self,attr):
        try:
            val = Model_Tee_Generic.__getattr__(self,attr)
            return val
        except Exception,e:
            #print "Getting attr %s from the initial_model"%attr
            val = self.initial_model.__getattr__(attr)
            return val
            
    def clone(self,init_args):
        
        myclone = self.__class__(self.initial_model,*init_args)
        myclone.set_image(self.image)
        return myclone


class Model_Shirt_Skel(Model_Long_Shirt_Generic):
     
    def symmetric_variable_pt_names(self):
        return ["spine_bottom","spine_top","left_collar","left_shoulder_joint","left_shoulder_top","left_sleeve_center","left_sleeve_top"]
        
    def mirrored_pts(self):
        #return {"left_collar":"right_collar","left_shoulder_joint":"right_shoulder_joint","left_shoulder_top":"right_shoulder_top","left_sleeve_center":"right_sleeve_center","left_sleeve_top":"right_sleeve_top"}
        return {"left_shoulder_joint":"right_shoulder_joint","left_shoulder_top":"right_shoulder_top","left_sleeve_center":"right_sleeve_center","left_sleeve_top":"right_sleeve_top","left_collar":"right_collar"}
    def bottom_left(self):
        displ = pt_diff(self.right_shoulder_joint(),self.left_shoulder_joint())
        return pt_sum(self.spine_bottom(),pt_scale(displ,-0.5))
        
    def allow_intersections(self):
        return False
    
        
class Model_Shirt_Skel_New(Model_Long_Shirt_Generic):
               
    def symmetric_variable_pt_names(self):
        return ["spine_bottom","spine_top","left_collar","left_shoulder_joint","left_shoulder_top","left_sleeve_center","left_sleeve_top","bottom_left"]
        
    def mirrored_pts(self):
        #return {"left_collar":"right_collar","left_shoulder_joint":"right_shoulder_joint","left_shoulder_top":"right_shoulder_top","left_sleeve_center":"right_sleeve_center","left_sleeve_top":"right_sleeve_top"}
        return {"left_shoulder_joint":"right_shoulder_joint","left_shoulder_top":"right_shoulder_top","left_sleeve_center":"right_sleeve_center","left_sleeve_top":"right_sleeve_top","left_collar":"right_collar"}
    
        
    def allow_intersections(self):
        return False
            
    def arm_pts_to_angles(self,sleeve_center,sleeve_top,spine_junction,shoulder_joint,arm):
        sleeve_len = vect_length(pt_diff(sleeve_center,shoulder_joint))
        angle = angle_between(pt_diff(shoulder_joint,spine_junction),pt_diff(sleeve_center,shoulder_joint))
        sleeve_width = vect_length(pt_diff(sleeve_top,sleeve_center))*2
        if sleeve_top[0] > sleeve_center[0]:
            skew = -1*(angle_between(pt_diff(sleeve_top,sleeve_center),pt_diff(sleeve_center,shoulder_joint))-pi/2)
        else:
            skew = angle_between(pt_diff(sleeve_top,sleeve_center),pt_diff(sleeve_center,shoulder_joint)) - pi/2
        if arm == "l":
            skew *= -1
                
        return (sleeve_len,sleeve_width,angle,skew)
        
    def angles_to_arm_pts(self,sleeve_len,sleeve_width,angle,skew,spine_junction,shoulder_joint,arm="l"):
        arm_ln = make_ln_from_pts(shoulder_joint,spine_junction)
        straight_pt = extrapolate(arm_ln,-1*sleeve_len)
        if arm == "r":
            multiplier = 1
        else:
            multiplier = -1
        sleeve_center = rotate_pt(straight_pt,multiplier*angle,shoulder_joint)
        straight_top = extrapolate(make_ln_from_pts(sleeve_center,shoulder_joint),-1*sleeve_width/2.0)
        straight_top_rot = rotate_pt(straight_top,-1*multiplier*pi/2,sleeve_center)
        sleeve_top = rotate_pt(straight_top_rot,multiplier*skew,sleeve_center)
        return (sleeve_center,sleeve_top)
        
    #Parameters are simply all of my vertices
    def params(self):
        output = []
        vertices = []
        vertices.extend(self.vertices)
        (l_sleeve_len,l_sleeve_width,l_angle,l_skew) = self.arm_pts_to_angles(self.left_sleeve_center(),self.left_sleeve_top(),self.shoulder_spine_junction(),self.left_shoulder_joint(),"l")
        #print "Sleeve length is %f, width is %f, angle is %f, skew is %f"%(l_sleeve_len,l_sleeve_width,l_angle,l_skew)
        #exit()
        if not self.symmetric:
            r_sleeve_center_index = self.variable_pt_names().index("right_sleeve_center")
            r_sleeve_top_index = self.variable_pt_names().index("right_sleeve_top")
            remove_ith_element(r_sleeve_top_index,vertices)
            remove_ith_element(r_sleeve_center_index,vertices)
        l_sleeve_center_index = self.variable_pt_names().index("left_sleeve_center")
        l_sleeve_top_index = self.variable_pt_names().index("left_sleeve_top")
        remove_ith_element(l_sleeve_top_index,vertices)
        remove_ith_element(l_sleeve_center_index,vertices)

        output.extend([l_sleeve_len,l_sleeve_width,l_angle,l_skew])
        if not self.symmetric:
            (r_sleeve_len,r_sleeve_width,r_angle,r_skew) = self.arm_pts_to_angles(self.right_sleeve_center(),self.right_sleeve_top(),self.shoulder_spine_junction(),self.right_shoulder_joint(),"r")
            output.extend([r_sleeve_len,r_sleeve_width,r_angle,r_skew])
        for pt in vertices:
            output.append(pt_x(pt))
            output.append(pt_y(pt))
        return output
    
    #Reads in a list of x,y values, and creates a new instance of myself with those points    
    def from_params(self,params):
        params = list(params)
        #print "Len params: %d"%len(params)
        pts = []
        l_sleeve_len = params.pop(0)
        l_sleeve_width = params.pop(0)
        l_angle = params.pop(0)
        l_skew = params.pop(0)
        if not self.symmetric:
            r_sleeve_len = params.pop(0)
            r_sleeve_width = params.pop(0)
            r_angle = params.pop(0)
            r_skew = params.pop(0)
        x = None
        for i,p in enumerate(params):
            if i%2 == 0:
                x = p
            else:
                y = p
                pts.append((x,y))
        #First create a rough model to get all other information        
        l_sleeve_center_index = self.variable_pt_names().index("left_sleeve_center")
        l_sleeve_top_index = self.variable_pt_names().index("left_sleeve_top")
        assert l_sleeve_top_index > l_sleeve_center_index
        pts.insert(l_sleeve_center_index,self.left_sleeve_center())
        pts.insert(l_sleeve_top_index,self.left_sleeve_top())
        if not self.symmetric:
            r_sleeve_center_index = self.variable_pt_names().index("right_sleeve_center")
            r_sleeve_top_index = self.variable_pt_names().index("right_sleeve_top")
            assert r_sleeve_center_index > l_sleeve_top_index
            assert r_sleeve_top_index > r_sleeve_center_index
            pts.insert(r_sleeve_center_index,self.right_sleeve_center())
            pts.insert(r_sleeve_top_index,self.right_sleeve_top())
        #print "Len pts: %d"%len(pts)
        intermed_model = self.clone(pts)
        
        (l_sleeve_center,l_sleeve_top) = self.angles_to_arm_pts(l_sleeve_len,l_sleeve_width,l_angle,l_skew,intermed_model.shoulder_spine_junction(),intermed_model.left_shoulder_joint())
        pts[l_sleeve_center_index] = l_sleeve_center
        pts[l_sleeve_top_index] = l_sleeve_top
        if not self.symmetric:
            (r_sleeve_center,r_sleeve_top) = self.angles_to_arm_pts(r_sleeve_len,r_sleeve_width,r_angle,r_skew,intermed_model.shoulder_spine_junction(),intermed_model.right_shoulder_joint(),arm="r")
            pts[r_sleeve_center_index] = r_sleeve_center
            pts[r_sleeve_top_index] = r_sleeve_top
        real_model = self.clone(pts)
        """
        img = cv.CloneImage(self.image)
        real_model.draw_to_image(img,cv.CV_RGB(0,0,255))
        cv.NamedWindow("intermed")
        cv.ShowImage("intermed",img)
        cv.WaitKey(100)
        """
        return real_model
        
    
class Model_Shirt_Skel_Restricted(Model_Long_Shirt_Generic):
    def symmetric_variable_pt_names(self):
        return ["spine_bottom","spine_top","left_collar","left_shoulder_joint","left_shoulder_top","bottom_left"]
        
    def mirrored_pts(self):
        return {"left_shoulder_joint":"right_shoulder_joint","left_shoulder_top":"right_shoulder_top","left_collar":"right_collar"}
        
    def symmetric_variable_param_names(self):
        return ["left_sleeve_length","left_sleeve_width","left_sleeve_angle"]
        
    def mirrored_params(self):
        return {"left_sleeve_length":"right_sleeve_length","left_sleeve_width":"right_sleeve_width","left_sleeve_angle":"right_sleeve_angle"}
        
    def left_sleeve_axis(self):
        angle = self.left_sleeve_angle()
        
        horiz_axis = make_ln_from_pts(self.left_shoulder_joint(),self.shoulder_spine_junction())
        straight_pt = extrapolate(horiz_axis,-1)
        new_pt = rotate_pt(straight_pt,-1*angle,self.left_shoulder_joint())
        return make_ln_from_pts(self.left_shoulder_joint(),new_pt)
        
    def right_sleeve_axis(self):
        angle = self.right_sleeve_angle()
        
        horiz_axis = make_ln_from_pts(self.right_shoulder_joint(),self.shoulder_spine_junction())
        straight_pt = extrapolate(horiz_axis,-1)
        new_pt = rotate_pt(straight_pt,angle,self.right_shoulder_joint())
        return make_ln_from_pts(self.right_shoulder_joint(),new_pt)
    
    def left_sleeve_center(self):
        return extrapolate(self.left_sleeve_axis(),abs(self.left_sleeve_length()))
    
    def right_sleeve_center(self):
        return extrapolate(self.right_sleeve_axis(),abs(self.right_sleeve_length()))
        
    def left_sleeve_top(self):
        straight_pt = extrapolate(self.left_sleeve_axis(),abs(self.left_sleeve_length()) + abs(self.left_sleeve_width())/2.0)
        return rotate_pt(straight_pt,pi/2,self.left_sleeve_center())
        
    def right_sleeve_top(self):
        straight_pt = extrapolate(self.right_sleeve_axis(),abs(self.right_sleeve_length()) + abs(self.right_sleeve_width())/2.0)
        return rotate_pt(straight_pt,-pi/2,self.right_sleeve_center())   
        
    def allow_intersections(self):
        return False
        
    def allow_flipping(self):
        return False
        
    def illegal(self):
        sides = self.sides()
        for i in range(len(sides)):
            for j in range(i,len(sides)):
                if i != j:
                    if seg_intercept(sides[i],sides[j]) != None:
                        if i==1 and j==3 or i ==10 and j ==12:
                            #print "Sleeve intersection"
                            pass
                        else:
                            #print "Self intersection!"
                            return True
                
        return False

class Model_Shirt_Skel_Less_Restricted(Model_Long_Shirt_Generic):

    
    def symmetric_variable_pt_names(self):
        return ["spine_bottom","spine_top","left_collar","left_shoulder_joint","left_shoulder_top","left_sleeve_center","bottom_left"]
        
    def mirrored_pts(self):
        return {"left_shoulder_joint":"right_shoulder_joint","left_shoulder_top":"right_shoulder_top","left_sleeve_center":"right_sleeve_center","bottom_left":"bottom_right"}
        
    def symmetric_variable_param_names(self):
        return ["left_sleeve_width"]
        
    def mirrored_params(self):
        return {"left_sleeve_width":"right_sleeve_width"}
        
    def relative_pts(self):
        return {"left_collar":"spine_top","left_shoulder_top":"left_shoulder_joint","right_shoulder_top":"right_shoulder_joint","bottom_left":"spine_bottom","bottom_right":"spine_bottom"}
    
    def left_sleeve_angle(self):
        return angle_between(pt_diff(self.left_shoulder_joint(),self.shoulder_spine_junction()),pt_diff(self.left_sleeve_center(),self.left_shoulder_joint()))
        
    def right_sleeve_angle(self):
        return angle_between(pt_diff(self.right_shoulder_joint(),self.shoulder_spine_junction()),pt_diff(self.right_sleeve_center(),self.right_shoulder_joint()))
        
    def left_sleeve_length(self):
        return vect_length(pt_diff(self.left_sleeve_center(),self.left_shoulder_joint()))
        
    def right_sleeve_length(self):
        return vect_length(pt_diff(self.right_sleeve_center(),self.right_shoulder_joint()))
        
    def left_sleeve_axis(self):
        angle = self.left_sleeve_angle()
        
        horiz_axis = make_ln_from_pts(self.left_shoulder_joint(),self.shoulder_spine_junction())
        straight_pt = extrapolate(horiz_axis,-1)
        new_pt = rotate_pt(straight_pt,-1*angle,self.left_shoulder_joint())
        return make_ln_from_pts(self.left_shoulder_joint(),new_pt)
        
    def right_sleeve_axis(self):
        angle = self.right_sleeve_angle()
        
        horiz_axis = make_ln_from_pts(self.right_shoulder_joint(),self.shoulder_spine_junction())
        straight_pt = extrapolate(horiz_axis,-1)
        new_pt = rotate_pt(straight_pt,angle,self.right_shoulder_joint())
        return make_ln_from_pts(self.right_shoulder_joint(),new_pt)
    
        return extrapolate(self.right_sleeve_axis(),abs(self.right_sleeve_length()))
        
    def left_sleeve_top(self):
        straight_pt = extrapolate(self.left_sleeve_axis(),abs(self.left_sleeve_length()) + abs(self.left_sleeve_width())/2.0)
        return rotate_pt(straight_pt,pi/2,self.left_sleeve_center())
        
    def right_sleeve_top(self):
        straight_pt = extrapolate(self.right_sleeve_axis(),abs(self.right_sleeve_length()) + abs(self.right_sleeve_width())/2.0)
        return rotate_pt(straight_pt,-pi/2,self.right_sleeve_center())   
    
    def right_collar(self):
        return mirror_pt(self.left_collar(),self.axis_of_symmetry())
        
    #def bottom_left(self):
    #    displ = pt_diff(self.right_shoulder_joint(),self.left_shoulder_joint())
    #    return pt_sum(self.spine_bottom(),pt_scale(displ,-0.5))
        
    def allow_intersections(self):
        return False
        
    def allow_flipping(self):
        return False
    
    """    
    def illegal(self):
        sides = self.sides()
        for i in range(len(sides)):
            for j in range(i,len(sides)):
                if i != j:
                    if seg_intercept(sides[i],sides[j]) != None:
                        if i==1 and j==3 or i ==10 and j ==12:
                            #print "Sleeve intersection"
                            pass
                        else:
                            #print "Self intersection!"
                            return True
                
        return False
    """ 
    
    def bottom_right(self):
        return self.__getattr__("bottom_right")()
        
    def spine_bottom(self):
        if self.symmetric:
            return self.__getattr__("spine_bottom")()
    
        else:
            return pt_scale(pt_sum(self.bottom_left(),self.bottom_right()),0.5)


class Model_Shirt_Skel_Restricted_Arms_Down(Model_Shirt_Skel_Restricted):
    def illegal(self):
        sides = self.sides()
        for i in range(len(sides)):
            for j in range(i,len(sides)):
                if i != j:
                    if not (i==1 and (j ==2 or j == 3)) and not (j == 12 and (i ==10 or i==11)):
                        if seg_intercept(sides[i],sides[j]) != None:
                            print "Self intersection!"
                            return True
                
        return False

def remove_ith_element(i,lst):
    copylst = list(lst)
    lst.pop(i)
        

