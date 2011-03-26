import math
import random
from numpy import *
import math
import cv
from Vector2D import *
import inspect
import pyflann
import rospy
from appearance_utils.srv import *
from appearance_utils.msg import PatchResponse
import ImageUtils
import RosUtils
import os
import os.path
import pickle

#Global variables -- need to think about this if it ever starts parallelizing
nn_solver = pyflann.FLANN()
patch_responses = None
black_image = None
image = None
contour = None
sparse_contour = None
extra_sparse_contour = None

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

    def save_pts(self):
        return self.polygon_vertices()
    
    def preferred_delta(self):
        return 35.0

    def polygon_vertices(self):
        abstract
        
    def set_image(self,image):
        self.image = image
    
    def set_image_size(self,image_size):
        self.image_size = image_size

    def sides(self):
        verts =  self.polygon_vertices()
        segs = []
        for i,v in enumerate(verts):
            segs.append(make_seg(verts[i-1],v))
        return segs
        
    def vertices_dense(self,density=10,length=10,constant_length=False,contour_mode=False):
        vertices = self.polygon_vertices()
        
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
    
    def score(self,contour=None, image=None, contourOnly=False):
        beta = self.realBeta(contourOnly)
        if beta == 1:
            score = self.contour_score()
        elif beta == 0:
            score = self.appearance_score()
        else:
            score = beta*self.contour_score() + (1 - beta)*self.appearance_score()
        score += self.structural_penalty()
        return score
   
    def realBeta(self,contourOnly):
        if contourOnly:
            return 1
        else:
            return self.beta()

    def initialize_appearance(self,image,set=1):
        pass

    def initialize_contour(self,contour_new):
        global contour
        contour = contour_new

    def contour_score(self,new_contour=None):
        model_dist_param = 0.5
        contour_dist_param = 0.5
        global contour
        global sparse_contour
        if not sparse_contour:
            sparse_contour = make_sparse(contour,1000)
        num_model_pts = 30*len(self.sides())
        
        nn=self.nearest_neighbors_fast
        global extra_sparse_contour
        if not extra_sparse_contour:
            extra_sparse_contour = make_sparse(contour,num_model_pts)
        model_contour = self.vertices_dense(constant_length=False,density=30)
        
        nn_model = nn(model_contour,sparse_contour)
        model_dist_energy = sum([self.dist_fxn(dist) for dist in nn_model]) / float(len(nn_model))
        #Normalize
        #model_dist_energy /= float(self.dist_fxn(max(self.image.width,self.image.height)))
        model_dist_energy /= float(self.dist_fxn(max(self.image_size)))
    
        nn_contour = nn(extra_sparse_contour,model_contour)
        contour_dist_energy = sum([self.dist_fxn(dist) for dist in nn_contour]) / float(len(nn_contour))
        #Normalize
        #contour_dist_energy /= float(self.dist_fxn(max(self.image.width,self.image.height)))
        contour_dist_energy /= float(self.dist_fxn(max(self.image_size)))
        
        energy = model_dist_param * model_dist_energy + contour_dist_param * contour_dist_energy
        return energy
        
    def nearest_neighbors_fast(self,model_contour,sparse_contour):
        global nn_solver
        model_arr = array(model_contour)
        contour_arr = array(sparse_contour)
        result,dists = nn_solver.nn(sparse_contour,model_contour, num_neighbors=1,algorithm="kmeans",branching=32, iterations=3, checks=16);
        return [sqrt(dist) for dist in dists]

    def dist_fxn(self,val):
        return val**2

    def beta(self):
        return 1 

    def structural_penalty(self):
        return 0
        
    def contour_mode(self):
        return False

    def draw_to_image(self,img,color):
        self.draw_skeleton(img,color)
        self.draw_contour(img,color)
   
    def draw_skeleton(self,img,color,thickness):
        abstract

    def draw_contour(self,img,color,thickness):
        abstract 

    #Simple hack to get the outer contour: draw it on a white background and find the largest contour            
    def get_silhouette(self,vertices,num_pts):
        storage = cv.CreateMemStorage(0)
        #black_image = cv.CreateImage(cv.GetSize(self.image),8,1)
        #black_image = cv.CreateImage((self.image.width*2,self.image.height*2),8,1)
        (width,height) = self.image_size
        global black_image
        if not black_image:
            black_image = cv.CreateImage((width*2,height*2),8,1)
        cv.Set(black_image,cv.CV_RGB(0,0,0))
        self.draw_contour(black_image,cv.CV_RGB(255,255,255),2)
        #cv.PolyLine(black_image,[vertices],4,cv.CV_RGB(255,255,255),0)
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
        xs = [x for (x,y) in self.polygon_vertices()]
        ys = [y for (x,y) in self.polygon_vertices()]
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
        cv.PolyLine(img,[self.polygon_vertices()],1,color,3)
        
    def draw_point(self,img,pt,color):
        cv.Circle(img,self.snap(pt,img),5,color,-1)
        
    def draw_line(self,img,pt1,pt2,color,thickness=2):
        cv.Line(img,self.snap(pt1,img),self.snap(pt2,img),color,thickness=thickness)
        
    def snap(self,pt,img):
        return (max( min(pt[0],img.width),0), max( min(pt[1],img.height), 0))

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
        self.scalar_params = [p*amt for p in self.scalar_params]
    
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
                self.vertices[i/2] = (x,y)
        for i,s in enumerate(scalar_params):
            self.scalar_params[i] = s
            return self

    def clone(self,init_args):
        myclone = self.__class__(*init_args)
        #myclone.set_image(self.image)
        myclone.set_image_size(self.image_size)
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
                            return True
        if not self.allow_flipping():
            sides=  self.sides()
            for i in range(len(sides)):
                for j in range(i-2,i+1):
                    if seg_intercept(sides[i],sides[j]) != None:
                            return True
        return False

        
class Point_Model_Contour_Only_Asymm(Point_Model):
    def __init__(self,*vertices_and_params):
        vertices = list(vertices_and_params)
        self.num_variable_pts = len(vertices)
        Point_Model.__init__(self,*vertices_and_params)
        
    def variable_pt_names(self):
        return ["pt_%d"%i for i in range(self.num_variable_pts)]
        #return ["pt_%d"%i for i in range(13)]
        
    def polygon_vertices(self):
        return self.vertices
        
    def __getattr__(self,attr):
        if attr == "num_variable_pts":
            return self.__dict__["num_variable_pts"]
        else:
            return Point_Model.get_attr(self,attr)

# Phases of optimization
class Orient_Model(Point_Model):
    def __init__(self,initial_model,*params):
        self.initial_model = initial_model
        #self.image = initial_model.image
        self.image_size = initial_model.image_size
        Point_Model.__init__(self,*params)
        
    def polygon_vertices(self):
        return self.transformed_model().polygon_vertices()
    
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
        #myclone.set_image(self.image)
        myclone.set_image_size(self.image_size)
        return myclone

# A model which is defined by fixed points and one foldline  
class Point_Model_Folded(Point_Model):
    #For now, we can easily assume all folds are left to right, and work a sign in later to fix it
    def __init__(self,initial_model,*pts):
        self.initial_model = initial_model
        #self.image = None
        self.image_size = None
        Point_Model.__init__(self,*pts)
        
   # def variable_param_names(self):
   #     return ["fold_angle","fold_displ"]
    
    def set_image(self,img):
        self.image = img
        self.initial_model.set_image(img)

    def set_image_size(self,image_size):
        self.image_size = image_size
        self.initial_model.set_image_size(self.image_size)
    
    def variable_pt_names(self):
        return ["fold_bottom","fold_top"]
        
    def relative_pts(self):
        return {"fold_top":"fold_bottom"}
    
    def set_image(self,image):
        self.initial_model.set_image(image)
        Point_Model.set_image(self,image)
    
    def polygon_vertices(self):

        init_polygon_vertices = self.initial_model.polygon_vertices()
        foldline = self.foldline()
        foldseg = self.foldseg()
        perp = perpendicular(foldline,line_offset(foldline))
        pts = []

        dot_prods = [dot_prod(p,line_vector(foldline)) for p in (self.fold_top(),self.fold_bottom())]
        epsilon = 0.25 * (max(dot_prods) - min(dot_prods))
        
        for i,pt in enumerate(init_polygon_vertices):
            if dot_prod(pt,line_vector(perp)) < dot_prod(line_offset(perp),line_vector(perp)):
                #Check if I'm within the bounds

                if min(dot_prods)-epsilon <= dot_prod(pt,line_vector(foldline)) <= max(dot_prods)+epsilon:
                    pts.append(mirror_pt(pt,foldline))
                else:
                    touching_sides = [make_seg(init_polygon_vertices[i-1],pt),make_seg(pt,init_polygon_vertices[(i+1)%len(init_polygon_vertices)])]
                    if len([seg for seg in touching_sides if seg_intercept(seg,foldseg)]) > 0:
                        pts.append(mirror_pt(pt,foldline))
                    else:
                        pts.append(pt)
                #pts.append(pt)
            else:
                pts.append(pt)
        last_inter = None
        offset = 0
        #dot_prods = [dot_prod(p,line_vector(foldline)) for p in (self.fold_top(),self.fold_bottom())]
        for i,seg in enumerate(self.initial_model.sides()):
            inter = seg_intercept(seg,foldseg)
            if inter != None and min(dot_prods)-epsilon <= dot_prod(inter,line_vector(foldline)) <= max(dot_prods)+epsilon:
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
        #return make_seg(self.fold_bottom(),self.fold_top())
    def foldseg(self):
        #return make_ln_from_pts(self.fold_bottom(),self.fold_top())
        return make_seg(self.fold_bottom(),self.fold_top())
         
    def structural_penalty(self):
        if cv.PointPolygonTest(self.initial_model.vertices_dense(),self.fold_bottom(),0) >= 0:
            return 1
        if cv.PointPolygonTest(self.initial_model.vertices_dense(),self.fold_top(),0) >= 0:
            return 1
        return 0
    
    def allow_intersections(self):
        return True
        
    def draw_to_image(self,img,color):
        self.draw_line(img,intercept(self.foldline(),horiz_ln(y=0.0)),intercept(self.foldline(),horiz_ln(y=img.height)),color)
        val = [self.draw_point(img,pt,color) for pt in self.polygon_vertices()]
        self.draw_point(img,self.fold_bottom(),cv.CV_RGB(0,255,0))
        self.draw_point(img,self.fold_top(),cv.CV_RGB(0,0,255))
        Point_Model.draw_to_image(self,img,color)
        
    def clone(self,init_args):
        myclone = self.__class__(self.initial_model,*init_args)
        #myclone.set_image(self.image)
        myclone.set_image_size(self.image_size)
        return myclone
        
    def preferred_delta(self):
        return 1.0
        
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
        #newmodel.set_image(self.image)
        newmodel.set_image_size(self.image_size)
        return newmodel

    def structural_penalty(self):
        return self.initial_model.structural_penalty()


class Point_Model_Variable_Symm(Point_Model):
    
    def __init__(self,symmetric,*vertices_and_params):
        self.symmetric = symmetric
        Point_Model.__init__(self,*vertices_and_params)
    
    
    def variable_pt_names(self):
        if 'symmetric' in self.__dict__.keys() and self.symmetric:
            return self.symmetric_variable_pt_names()
        else:
            return self.symmetric_variable_pt_names() + sorted(self.mirrored_pts().values())
            
    def variable_param_names(self):
        if 'symmetric' in self.__dict__.keys() and self.symmetric:
            return self.symmetric_variable_param_names()
        else:
            return self.symmetric_variable_param_names() + sorted(self.mirrored_params().values())
    
    #Modifies __getattr__ to lookup mirrored points and params
    def __getattr__(self,attr):
        if 'symmetric' in self.__dict__.keys() and self.symmetric:
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
        #asymm.set_image(self.image)
        asymm.set_image_size(self.image_size)
        return asymm
        
    def free(self):
        model = Point_Model_Contour_Only_Asymm(*self.polygon_vertices())
        #model.set_image(self.image)
        model.set_image_size(self.image_size)
        return model
    
    def clone(self,init_args):
        myclone = self.__class__(self.symmetric,*init_args)
        #myclone.set_image(self.image)
        myclone.set_image_size(self.image_size)
        return myclone
        
###
# Defining some clothing models
###
class AppearanceInfo:
    def __init__(self):
        self.modelfile = None
        self.responses = None
        self.mask = None
        self.maskfile = None
        ##self.visfile = None
        ##self.vis = None

class Model_Sock_Generic(Point_Model_Variable_Symm):

    def closest_n_patch_responses(self,point,n):
        mins = patch_responses[:n]
        mins.sort(key=lambda pr: pt_distance(point,(pr.x,pr.y)))
        for p in patch_responses[n:]:
            if pt_distance((p.x,p.y),point) < pt_distance((mins[-1].x,mins[-1].y),point): 
                mins.append(p)
                mins.sort(key=lambda pr: pt_distance(point,(pr.x,pr.y)))
                mins = mins[:n] 
        return mins

    def appearance_model(self,set=1):
        abstract

    def appearance_type(self):
        abstract

    def appearance_mode(self):
        abstract

    def response(self,point,type):
        #Do weighted sum
        (pr1,pr2) = self.closest_n_patch_responses(point,2) #Only horizontal neighbors matter here
        if type not in pr2.response_types:
            return 0
        type_index = pr1.response_types.index(type)
        d1 = pt_distance((pr1.x,pr1.y),point)
        d2 = pt_distance((pr2.x,pr2.y),point)
        response = (d1 * pr2.responses[type_index] + d2*pr1.responses[type_index]) / (d1+d2)
        #Scale by the distance to the nearest
    
        return response
        #return (1 - exp(-5*response)) / (1 - exp(-5))
    
    def beta(self):
        return 0.99

    def contour_mode(self):
        return False 
    
    
    def get_cache_directory(self,imagefile):
        real_path = os.path.realpath(imagefile).replace("socks_data","socks_data_cached").replace(".","-")
        dir = real_path
        if not os.path.exists(dir):
            os.makedirs(dir)
        return dir

    def get_cache_experiment_directory(self,imagefile,set):
        dir = self.get_cache_directory(imagefile)
        subdir = "set_%d"%set
        fulldir = "%s/%s"%(dir,subdir)
        if not os.path.exists(fulldir):
            os.makedirs(fulldir)
        return fulldir

    def get_cache_response_file(self,imagefile,set,modelfile):
        dir = self.get_cache_experiment_directory(imagefile,set)
        filename = "response_" + os.path.realpath(modelfile).split("/home/stephen/")[1].split(".")[0].replace("/","-") + ".pickle"
        return "%s/%s"%(dir,filename)

    def get_mask_file(self,imagefile):
        dir = self.get_cache_directory(imagefile)
        filename = "mask.png"
        return "%s/%s"%(dir,filename)

    def get_cache_model_directory(self,imagefile,set,shapemodelfile):
        dir = self.get_cache_experiment_directory(imagefile,set)
        subdir =  "model_" + os.path.realpath(shapemodelfile).split("/home/stephen/")[1].split(".")[0].replace("/","-")
        fulldir = "%s/%s"%(dir,subdir)
        if not os.path.exists(fulldir):
            os.makedirs(fulldir)
        return fulldir

    def lookup_appearance_cached(self,imagefile,set,modelfile):
        response_name = self.get_cache_response_file(imagefile,set,modelfile)
        if os.path.exists(response_name):
            f = open(response_name)
            appearance_info = pickle.load(f)
            f.close()
            return appearance_info
        else:
            return None

    def lookup_mask_cached(self,imagefile):
        maskfile = self.get_mask_file(imagefile)
        if os.path.exists(maskfile):
            return cv.LoadImage(maskfile,cv.CV_LOAD_IMAGE_GRAYSCALE)
        else:
            return None

    def cache_appearance(self,info,imagefile,set,modelfile):
        picklename = self.get_cache_response_file(imagefile,set,modelfile)
        if os.path.exists(picklename):
            val = raw_input("Appearance already exists! Overwrite?")
            if not (len(val) > 0 and val[0] == y):
                print "Quitting."
                exit()
        f = open(picklename,'w')
        pickle.dump(info,f)
        f.close()
        return info

    def cache_mask(self,mask,imagefile):
        filename = self.get_mask_file(imagefile)
        cv.SaveImage(filename,mask)

    def initialize_appearance_cached(self,imagefile,set=1):
        modelfile = self.appearance_model(set)
        cached_info = self.lookup_appearance_cached(imagefile,set,modelfile)
        if cached_info:
            print "HIT CACHE!"
            appearance_info = cached_info
        else:
            print "DIDN'T HIT CACHE"
            appearance_info = self.initialize_appearance(imagefile,modelfile,set=set)
            self.cache_appearance(appearance_info,imagefile,set,modelfile)
        mask = self.lookup_mask_cached(imagefile)
        if mask:
            print "HIT MASK CACHE"
        else:
            if cached_info:
                print "Didn't have mask but had cached info. That makes no sense!"
                assert False
            mask = self.initialize_mask(set)
            self.cache_mask(mask,imagefile)
        global patch_responses
        patch_responses = appearance_info.responses
        return mask
    
    def initialize_appearance(self,imagefile,modelfile,getMask=True,set=1):
        image = cv.LoadImage(imagefile)
        RosUtils.call_service("%s/load_image"%self.landmark_service(set), LoadImage, 
                image = ImageUtils.cv_to_imgmsg(image),
                mode = self.appearance_mode())
        res = RosUtils.call_service("%s/landmark_response_all"%self.landmark_service(set), LandmarkResponseAll, 
                type=self.appearance_type(),model_file = modelfile)

        info = AppearanceInfo()
        #info.vis = ImageUtils.imgmsg_to_cv(vis_res.image,"bgr8")
        info.responses = res.patch_responses
        return info

    def initialize_mask(self,set=1):
            mask_res = RosUtils.call_service("%s/get_mask"%self.landmark_service(set), GetMask)
            return ImageUtils.imgmsg_to_cv(mask_res.mask,"mono8")

    def landmark_service(self,set):
        return "landmark_service_node_%d"%set



class Model_Sock_Skel(Model_Sock_Generic):
    
    def save_pts(self):
        return [self.ankle_center(),self.heel(),self.toe_center()]

    def appearance_model(self,set=1):
        return "/home/stephen/socks_data/model/normal_model/model_CHI_%d.txt"%set

    def appearance_type(self):
        return 3

    def appearance_mode(self):
        return LoadImageRequest.CONTOUR

    def polygon_vertices(self):
       #return [self.ankle_bottom(), self.ankle_center(), self.ankle_top(), self.heel() if self.flipped() else self.bend_point(), self.toe_top(), self.toe_center(), self.toe_bottom(), self.bend_point() if self.flipped() else self.heel()]
       #interpolating ellipse for speed
        return [self.ankle_bottom(), self.ankle_center(), self.ankle_top(), 
                self.heel() if self.flipped() else self.bend_point(), 
                self.toe_top(), self.toe_at(0.25), self.toe_center(), self.toe_at(0.75), self.toe_bottom(), 
                self.bend_point() if self.flipped() else self.heel()]

    def symmetric_variable_pt_names(self):
       return ["ankle_center","ankle_joint","toe_center"]

    def symmetric_variable_param_names(self):
        return ["sock_width","toe_radius"]

    def ankle_top(self): 
        straight_pt = extrapolate(self.ankle_axis(),self.ankle_length() + self.sock_width()/2.0)
        return rotate_pt(straight_pt,pi/2,self.ankle_center())

    def ankle_bottom(self): 
        straight_pt = extrapolate(self.ankle_axis(),self.ankle_length() + self.sock_width()/2.0)
        return rotate_pt(straight_pt,-pi/2,self.ankle_center())

    def toe_top(self): 
        straight_pt = extrapolate(self.toe_axis(),self.toe_length() + self.sock_width()/2.0)
        pt = rotate_pt(straight_pt,-pi/2,self.toe_center())
        return translate_pt(pt,pt_diff(extrapolate(self.toe_axis(),self.toe_length() - self.toe_radius()),self.toe_center()))

    def toe_bottom(self): 
        straight_pt = extrapolate(self.toe_axis(),abs(self.toe_length()) + abs(self.sock_width())/2.0)
        pt = rotate_pt(straight_pt,pi/2,self.toe_center())
        return translate_pt(pt,pt_diff(extrapolate(self.toe_axis(),self.toe_length() - self.toe_radius()),self.toe_center()))

    def toe_at(self,pct):
        ctr = pt_center(self.toe_top(),self.toe_bottom())
        angle = (pct-0.5)*pi
        a = self.toe_radius()
        b = self.sock_width()/2.0
        radius = a*b / sqrt((b*cos(angle))**2 + (a*sin(angle))**2)
        straight_pt = extrapolate(self.toe_axis(),abs(pt_distance(self.ankle_joint(),ctr) + radius))
        pt = rotate_pt(straight_pt,angle,ctr)
        #return translate_pt(pt,pt_diff(extrapolate(self.toe_axis(),self.toe_length() - self.toe_radius()),self.toe_center()))
        return pt

    def ankle_length(self):
        return pt_distance(self.ankle_center(),self.ankle_joint())
    
    def toe_length(self):
        return pt_distance(self.toe_center(),self.ankle_joint())

    def ankle_axis(self):
        return make_seg(self.ankle_joint(),self.ankle_center())

    def toe_axis(self):
        return make_seg(self.ankle_joint(),self.toe_center())
    
    def flipped(self):
        return False


    def heel(self):
        straight_pt = extrapolate(self.ankle_axis(),self.sock_width()/2)
        if self.flipped():    
            return rotate_pt(straight_pt,pi/2,self.ankle_joint())
        else:
            return rotate_pt(straight_pt,-pi/2,self.ankle_joint())
   
    def bend_point(self):
        straight_pt = extrapolate(self.ankle_axis(),self.sock_width()/2)
        if self.flipped():    
            return rotate_pt(straight_pt,-pi/2,self.ankle_joint())
        else:
            return rotate_pt(straight_pt,pi/2,self.ankle_joint())

    def heel_and_bend_point(self):
        straight_pt = extrapolate(self.ankle_axis(),self.sock_width()/2)
        return (rotate_pt(straight_pt,-pi/2,self.ankle_joint()),rotate_pt(straight_pt,pi/2,self.ankle_joint()))

    def toe_angle(self):
        return math.atan2(self.ankle_joint()[1]-self.toe_center()[1],-1*(self.ankle_joint()[0]-self.toe_center()[0]))*180/pi

    def heel_angle(self):
        return math.atan2(self.ankle_joint()[1]-self.heel_center()[1],-1*(self.ankle_joint()[0]-self.heel_center()[0]))*180/pii

    def ankle_angle(self):
        return math.atan2(self.ankle_joint()[1]-self.ankle_center()[1],-1*(self.ankle_joint()[0]-self.ankle_center()[0]))*180/pi

    def vertices_dense(self,density=10,length=10,constant_length=False,contour_mode=False):
        output = []
        vertices = [self.toe_bottom(), self.bend_point() if self.flipped() else self.heel(),
                    self.ankle_bottom(), self.ankle_center(), self.ankle_top(),
                    self.heel() if self.flipped() else self.bend_point(),
                    self.toe_top()] 
        for i,vert in enumerate(vertices):
                output.append(vert)
                if i == len(vertices)-1:
                    pass
                else:
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
        for i in range(density):
            dt = 1/float(density+1)
            output.append(self.toe_at((i+1)*dt))
        return output
    
    def draw_to_image(self,img,color): 
        #Draw polygon points
        self.draw_point(    img,    self.ankle_bottom(),            color)
        self.draw_point(    img,    self.ankle_top(),               color)
        self.draw_point(    img,    self.bend_point(),              color)
        self.draw_point(    img,    self.toe_top(),                 color)
        self.draw_point(    img,    self.toe_bottom(),              color)
        self.draw_point(    img,    self.heel(),                    cv.CV_RGB(0,255,0))
        ##for i in range(100):
        ##    self.draw_point(img, self.toe_at(i*0.01), color)
        #Draw outline
        self.draw_contour(img,color,2)
        #Draw skeletal frame
        self.draw_point(img,self.ankle_center(),color)
        self.draw_point(img,self.ankle_joint(),color)
        self.draw_point(img,self.toe_center(),color)
        self.draw_line(img,self.ankle_center(),self.ankle_joint(),color)
        self.draw_line(img,self.ankle_joint(),self.toe_center(),color)

    def draw_contour(self,img,color,thickness=2):
        #Draw outline
        top_joint = self.heel() if self.flipped() else self.bend_point()
        bottom_joint = self.bend_point() if self.flipped() else self.heel()
        self.draw_line(      img,    self.ankle_bottom(),    self.ankle_top(),      color,  thickness)
        self.draw_line(      img,    self.ankle_top(),       top_joint,     color,  thickness)
        self.draw_line(      img,    top_joint,              self.toe_top(),        color,  thickness)
        cv.Ellipse(
            img,    pt_center(self.toe_top(),self.toe_bottom()),
            (max(self.toe_radius(),1),max(self.sock_width()/2,1)),
            -1*self.toe_angle(),
            -90,    90,     
            color, thickness)
        self.draw_line(      img,    self.toe_bottom(),       bottom_joint,          color,  thickness)
        self.draw_line(      img,    bottom_joint,             self.ankle_bottom(),  color,  thickness)
    
    def concave_heel(self):
        vert_axis = pt_diff(self.bend_point(),self.heel())
        if not self.flipped():
            return  dot_prod(self.heel(),vert_axis) > dot_prod(self.toe_bottom(),vert_axis)
        else:
            return  dot_prod(self.heel(),vert_axis) > dot_prod(self.toe_top(),vert_axis)

    def structural_penalty(self):
        penalty = 0
        if self.toe_radius() <= 0:
            penalty += 1
        if self.sock_width() <= 0:
            penalty += 1
        if self.concave_heel():
            penalty += 1
        return penalty

    
    
    def appearance_score(self,new_image = None):
        appearance_reward = dot(array(self.appearance_weights()), array(self.appearance_responses()))
        return 1 - appearance_reward
    
    def appearance_responses(self):
        return (self.ankle_score(), self.toe_score(), self.heel_score(), self.nothing_score())

    def appearance_weights(self):
        return (0.4, 0.35, 0.20, 0.05)


    def response_to_prob(self,responses,type):
        return responses[type] / sum(responses)

    def heel_score(self):
        #return min([self.response(pt,PatchResponse.HEEL) for pt in (self.heel(), self.bend_point())])
        pt = self.heel()
        return self.response(pt,PatchResponse.HEEL)

    def toe_score(self):
        pt = self.toe_center()
        return self.response(pt,PatchResponse.TOE)

    def ankle_score(self):
        pt = self.ankle_center()
        return self.response(pt,PatchResponse.OPENING)

    def nothing_score(self):
        score = 0
        pts_list = (self.toe_top(),self.toe_bottom(),self.ankle_top(),self.ankle_bottom(),self.bend_point())
        for pt in pts_list:
            score += self.response(pt,PatchResponse.OTHER)
        return score / len(pts_list)

class Model_Sock_Skel_Flipped(Model_Sock_Skel):
    def flipped(self):
        return True

class Model_Sock_Skel_Vert(Model_Sock_Skel):
    def save_pts(self):
        return [self.ankle_center(),self.toe_center()]
    
    def appearance_model(self, set=1):
        return "/home/stephen/socks_data/model/heel_model/model_CHI_%d.txt"%set
    
    def appearance_weights(self):
        return (0.5, 0.4, 0.0, 0.1)

    def concave_heel(self):
        return False

class Model_Sock_Skel_Vert_Flipped(Model_Sock_Skel_Vert):
    def flipped(self):
        return True

# A model for a bunched sock
class Model_Bunch(Model_Sock_Generic):
    def save_pts(self):
        return [self.seam_top(), self.seam_bottom()]
    
    def polygon_vertices(self):
        return [self.bottom_left(),self.top_left(),self.seam_top(),self.top_right(),self.bottom_right(),self.seam_bottom()]

    def symmetric_variable_pt_names(self):
        return ["left_center","right_center"]

    def symmetric_variable_param_names(self):
        return ["sock_width","seam_distance"]

    def top_left(self):
        return pt_sum(self.left_center(), pt_scale(line_vector(self.seam_axis()),self.sock_width()*0.5))
    def top_right(self):
        return pt_sum(self.right_center(), pt_scale(line_vector(self.seam_axis()),self.sock_width()*0.5))
    def bottom_left(self):
        return pt_sum(self.left_center(), pt_scale(line_vector(self.seam_axis()),-1*self.sock_width()*0.5))
    def bottom_right(self):
        return pt_sum(self.right_center(), pt_scale(line_vector(self.seam_axis()),-1*self.sock_width()*0.5))

    def horiz_axis(self):
        return make_seg(self.left_center(),self.right_center())

    def seam_axis(self):
        return perpendicular(make_ln_from_pts(self.left_center(),self.right_center()),self.seam_location())

    def seam_location(self):
        return extrapolate(self.horiz_axis(),self.seam_distance())

    def seam_bottom(self):
        #return intercept(           make_ln_from_pts(self.bottom_left(),self.bottom_right()),
        #                            self.seam_axis())
        return extrapolate(make_ln_from_pts(self.bottom_left(),self.bottom_right()),self.seam_distance())


    def seam_top(self):
        #return intercept(           make_ln_from_pts(self.top_left(),self.top_right()),
        #                            self.seam_axis())
        return extrapolate(make_ln_from_pts(self.top_left(),self.top_right()),self.seam_distance())

    def get_angle(self):
        left_center = self.left_center()
        right_center = self.right_center()
        return math.atan2(-1*(right_center[1]-left_center[1]),right_center[0]-left_center[0])

    def beta(self):
        return 0.99


    def draw_to_image(self,img,color,thickness=2):
        self.draw_contour(img,color,thickness)

    def draw_contour(self,img,color,thickness=2):
        Model_Sock_Generic.draw_to_image(self,img,color)
        
        self.draw_point(img,self.seam_top(),color)
        self.draw_point(img,self.seam_bottom(),color)
        self.draw_line(img,self.seam_top(),self.seam_bottom(),color)

    def structural_penalty(self):
        penalty = 0
        if self.seam_distance() <= 0:
            penalty += 1
        if self.seam_distance() >= pt_distance(self.left_center(),self.right_center()):
            penalty += 1
        return penalty



class Model_Bunch_Locate_Seam(Model_Bunch):

    def appearance_score(self):
        pt = self.seam_location()
        theta = -self.get_angle()
        res = RosUtils.call_service(   "landmark_service_node/landmark_response", LandmarkResponse, 
                                        x=pt[0],   y=pt[1], theta = theta,
                                        mode=LandmarkResponseRequest.BUNCH_SEAM,
                                        use_nearest = False)

        return res.response

class Model_Bunch_Locate_Inside_Out(Model_Bunch):

    def appearance_model(self,set=1):
        return "/home/stephen/socks_data/model/bunch_model/model_CHI_%d.txt"%set

    def appearance_type(self):
        return 0

    def appearance_mode(self):
        return LoadImageRequest.INSIDE
    
    def appearance_score(self,image=None):
        vals = [0,0]
        val_left = 0
        val_right = 0
        vertical_axis = pt_diff(self.right_center(), self.left_center())
        for ltr in (True,False):
            for p in patch_responses:
                if dot_prod((p.x,p.y),line_vector(self.horiz_axis()))  < dot_prod(self.seam_location(),line_vector(self.horiz_axis())):
                    vals[0 if ltr else 1] += p.responses[0 if ltr else 1]
                else:
                    vals[0 if ltr else 1] += p.responses[1 if ltr else 0]
        return 1 - max(vals)/len(patch_responses)

class Model_Towel(Point_Model_Variable_Symm):
    def polygon_vertices(self):
        return [self.bottom_left(),self.top_left(),self.top_right(),self.bottom_right()]
        
    def symmetric_variable_pt_names(self):
        return ["bottom_left","top_left","top_right","bottom_right"]
        
    def axis_of_symmetry(self):
        return make_ln_from_pts(pt_scale(pt_sum(self.bottom_left(),self.bottom_right()),0.5),pt_scale(pt_sum(self.top_left(),self.top_right()),0.5))

class Model_Pants_Generic(Point_Model_Variable_Symm):
    def polygon_vertices(self):
        return [self.left_leg_right(),self.left_leg_left(),self.top_left(),self.top_right(),self.right_leg_right(),self.right_leg_left(),self.crotch()]

    def axis_of_symmetry(self):
        return make_ln_from_pts(self.mid_center(),self.top_center())

    def crotch(self):
        ln = perpendicular(make_ln_from_pts(self.top_center(),self.mid_center()),self.mid_center())
        return mirror_pt(self.top_center(),ln)
        
    def left_leg_right(self):
        ln = perpendicular(make_ln_from_pts(self.left_leg_left(),self.left_leg_center()),self.left_leg_center())
        return mirror_pt(self.left_leg_left(),ln)
        
    def right_leg_left(self):
        ln = perpendicular(make_ln_from_pts(self.right_leg_right(),self.right_leg_center()),self.right_leg_center())
        return mirror_pt(self.right_leg_right(),ln)   
        
    def top_right(self):
        displ = pt_sum( pt_diff(self.top_center(),self.mid_center()), pt_diff(self.right_leg_right(),self.right_leg_center()))
        return translate_pt(self.mid_right(),displ)
        
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
        

class Model_Pants_Skel(Model_Pants_Generic):

        
    
        
    def symmetric_variable_pt_names(self):
        return ["mid_center","top_center","mid_left","left_leg_center","left_leg_left"]
        
    def mirrored_pts(self):
        return {"mid_left":"mid_right", "left_leg_center":"right_leg_center","left_leg_left":"right_leg_right"}
        
    
    """
    Defining other points
    """    
        
    
    
    def top_left(self):
        displ = pt_sum( pt_diff(self.top_center(),self.mid_center()), pt_diff(self.left_leg_left(),self.left_leg_center()))
        return translate_pt(self.mid_left(),displ)
        
    
        
        
    def structural_penalty(self):
        penalty = Point_Model_Variable_Symm.structural_penalty(self)
        """
        penalty += self.constrain(pt_distance(self.mid_left(),self.mid_right()),pt_distance(self.top_left(),self.top_right()),UPPER,0.0)
        if seg_intercept(make_seg(self.top_left(),self.left_leg_left()),make_seg(self.mid_left(),self.mid_right())):
            penalty += 1
        if seg_intercept(make_seg(self.top_right(),self.right_leg_right()),make_seg(self.mid_left(),self.mid_right())):
            penalty += 1
        """
        print "blah"
        if self.crotch_length() / ((self.left_leg_length() + self.right_leg_length())/2.0) > 0.5:
            penalty += 1
        

        
      
class Model_Pants_Contour_Only(Point_Model_Contour_Only_Asymm):

    def variable_pt_names(self):
        #return ["pt_%d"%i for i in range(self.num_variable_pts)]
        return ["pt_%d"%i for i in range(7)]
        return penalty
        

        
class Model_Pants_Skel_Extended(Model_Pants_Skel):
    
    def polygon_vertices(self):
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
        
class Model_Pants_Skel_New(Model_Pants_Generic):
    
    def symmetric_variable_pt_names(self):
        return ["mid_center","top_center","mid_left","left_leg_center","top_left"]
        
    def symmetric_variable_param_names(self):
        return ["left_leg_width"]
        
    def mirrored_pts(self):
        return {"mid_left":"mid_right","left_leg_center":"right_leg_center"}
        
    def mirrored_params(self):
        return {"left_leg_width":"right_leg_width"}
    
    def left_leg_axis(self):
        #return make_ln_from_pts(self.mid_left(),self.left_leg_center())
        return make_seg(self.mid_left(),self.left_leg_center())
        
    def right_leg_axis(self):
        #return make_ln_from_pts(self.mid_right(),self.right_leg_center())
        return make_seg(self.mid_right(),self.right_leg_center())
        
    def left_leg_length(self):
        return pt_distance(self.mid_left(),self.left_leg_center())
        
    def right_leg_length(self):
        return pt_distance(self.mid_right(),self.right_leg_center())
        
    def crotch_length(self):
        return pt_distance(self.top_center(),self.crotch())
    
    def left_leg_left(self):
        straight_pt = extrapolate(self.left_leg_axis(),abs(self.left_leg_length()) + abs(self.left_leg_width())/2.0)
        return rotate_pt(straight_pt,pi/2,self.left_leg_center())
        
    def right_leg_right(self):
        straight_pt = extrapolate(self.right_leg_axis(),abs(self.right_leg_length()) + abs(self.right_leg_width())/2.0)
        return rotate_pt(straight_pt,-pi/2,self.right_leg_center())
        
    def top_right(self):
        #displ = pt_sum( pt_diff(self.top_center(),self.mid_center()), pt_diff(self.right_leg_right(),self.right_leg_center()))
        #return translate_pt(self.mid_right(),displ)
        return pt_sum( pt_diff(self.top_center(),self.top_left()), self.top_center())
        
    def crotch(self):
        ln = make_ln_from_pts(self.mid_left(),self.mid_right())
        return mirror_pt(self.top_center(),ln)

    def structural_penalty(self):
        penalty = Model_Pants_Generic.structural_penalty(self)
        #penalty += self.constrain(pt_distance(self.mid_left(),self.mid_right()),pt_distance(self.top_left(),self.top_right()),UPPER,0.0)  
        skel_sides = [make_seg(self.mid_left(),self.mid_right())]
        """
        for s in skel_sides:
            for side in self.sides():
                if seg_intercept(s,side):
                    penalty += 1
        #penalty += self.constrain(
        #    abs(angle_between(pt_diff(self.mid_left(),self.mid_right()),pt_diff(self.top_center(),self.crotch())) - pi/2), pi/5, UPPER,0.0)
        penalty += self.constrain(pt_distance(self.top_center(),self.crotch())/pt_distance(self.top_left(),self.top_right()),0.25,LOWER,0.0)
        
        if seg_intercept(make_seg(self.mid_left(),self.mid_right()),make_seg(self.top_left(),self.left_leg_left())):
            penalty += 1
        if seg_intercept(make_seg(self.mid_left(),self.mid_right()),make_seg(self.top_right(),self.right_leg_right())):
            penalty += 1
        """
        if seg_intercept(self.left_leg_axis(),self.right_leg_axis()):
            penalty += 1

        if self.crotch_length() / (self.left_leg_length() + self.right_leg_length()/ 2.0) > 0.8:
            penalty += 1
            
        if pt_distance(self.left_leg_left(),self.left_leg_right()) < pt_distance(self.top_left(),self.top_right())/5.0:
            penalty += 1
        if pt_distance(self.left_leg_left(),self.left_leg_right()) < pt_distance(self.top_left(),self.top_right())/5.0:
            penalty += 1
        if pt_distance(self.mid_left(),self.mid_right()) > 2 * pt_distance(self.top_left(),self.top_right()):
            penalty += 1
        if pt_distance(self.top_left(),self.top_right()) > 0.75 * pt_distance(self.top_left(),self.left_leg_left()):
            penalty += 1
        if self.left_leg_width() > pt_distance(self.left_leg_right(),self.crotch()):
            penalty += 1
        if self.right_leg_width() > pt_distance(self.right_leg_left(),self.crotch()):
            penalty += 1
        if self.left_leg_width()/self.right_leg_width() < 0.5:
            penalty += 1    
        if self.right_leg_width()/self.left_leg_width() < 0.5:
            penalty += 1
            
        return penalty

#Generic class which makes no assertions about what the variable points are
class Model_Shirt_Generic(Point_Model_Variable_Symm):
    
    def polygon_vertices(self):
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
    
    def left_sleeve_length(self):
        return pt_distance(self.left_sleeve_center(),self.left_shoulder_joint())
        
    def right_sleeve_length(self):
        return pt_distance(self.right_sleeve_center(),self.right_shoulder_joint())
    
    def shirt_width(self):
        return pt_distance(self.bottom_left(),self.bottom_right())
        
    def shirt_height(self):
        return pt_distance(self.spine_bottom(),self.spine_top())
        
    
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
        
    def structural_penalty(self):
        penalty = Point_Model_Variable_Symm.structural_penalty(self)
        if self.shirt_width() > self.shirt_height() * 1.25:
            penalty += 1
        if pt_distance(self.left_shoulder_joint(),self.right_shoulder_joint()) > self.shirt_height() * 1.25:
            penalty += 1
        if self.left_sleeve_width() < 0.05*self.shirt_height():
            penalty += 1
        if self.right_sleeve_width() < 0.05*self.shirt_height():
            penalty += 1
        if pt_distance(self.left_shoulder_top(),self.left_armpit()) > self.shirt_height() * 0.5:
            penalty += 1
        if pt_distance(self.right_shoulder_top(),self.right_armpit()) > self.shirt_height() * 0.5:
            penalty += 1
        return penalty
        
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
        
        if self.left_sleeve_length()/self.shirt_width() < 0.05:
            penalty += 1 
        if self.right_sleeve_length()/self.shirt_width() < 0.05:
            penalty += 1
        if pt_distance(self.left_sleeve_bottom(),self.left_armpit())/self.shirt_width() < 0.05:
            penalty += 1
        if pt_distance(self.right_sleeve_bottom(),self.right_armpit())/self.shirt_width() < 0.05:
            penalty += 1
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
        if self.left_sleeve_length()/self.shirt_width() < 0.3:
            penalty += 1 
        if self.right_sleeve_length()/self.shirt_width() < 0.3:
            penalty += 1
        if pt_distance(self.left_sleeve_bottom(),self.left_armpit())/self.shirt_width() < 0.2:
            penalty += 1
        if pt_distance(self.right_sleeve_bottom(),self.right_armpit())/self.shirt_width() < 0.2:
            penalty += 1
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
        #self.image = init_model.image
        self.image_size = init_model.image_size
        Model_Tee_Generic.__init__(self,False,*pts)
        
    def symmetric_variable_pt_names(self):
        return ["left_armpit","left_sleeve_bottom","left_sleeve_top","right_armpit","right_sleeve_bottom","right_sleeve_top"]
        
    def polygon_vertices(self):
        return Model_Tee_Generic.polygon_vertices(self)
        
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
        #self.image = None
        self.image_size = None
        return self
        
    def __getattr__(self,attr):
        if attr == "symmetric":
            return self.__dict__["symmetric"]
        try:
            val = Model_Tee_Generic.__getattr__(self,attr)
            return val
        except Exception,e:
            val = self.initial_model.__getattr__(attr)
            return val
            
    def clone(self,init_args):
        
        myclone = self.__class__(self.initial_model,*init_args)
        #myclone.set_image(self.image)
        myclone.set_image_size(self.image_size)
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
        
