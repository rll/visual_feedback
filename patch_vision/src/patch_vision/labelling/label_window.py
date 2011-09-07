import roslib
roslib.load_manifest("patch_vision")
import cv
import re
from patch_vision.labelling.label_set import LabelSet
from patch_vision.utils.zoom_window import ZoomWindow

class LabelWindowParams:
    def __init__(self):
        self.label_set = LabelSet()
        self.default_brush_size = 5
        self.save_mat_path = None
        self.save_image_path = None
        self.default_zoom = 1



class LabelWindow (ZoomWindow):
    def __init__(self, image, params):
        self.params = params
        gray_image_1d = cv.CreateImage((image.width,image.height),image.depth,1)
        cv.CvtColor(image,gray_image_1d,cv.CV_RGB2GRAY)
        self.gray_image = cv.CreateImage((image.width,image.height),image.depth,3)
        cv.CvtColor(gray_image_1d, self.gray_image, cv.CV_GRAY2RGB)
        self.label_image = cv.CreateImage((image.width,image.height),image.depth,3)
        self.label_mat = cv.CreateMat(image.height,image.width,cv.CV_8UC1)
        self.visible_image = cv.CreateImage((image.width,image.height),image.depth,3)

        self.brush_size = params.default_brush_size
        self.brush_label = 0
        
        self.clear()
        
        ZoomWindow.__init__(self, "Labeler", 100, params.default_zoom)

    def handleEventsUnzoomed(self,event,x,y,flags,param):
        if flags-32 == cv.CV_EVENT_FLAG_LBUTTON:
           label_color = self.params.label_set.get_label_color(self.brush_label)
           label_number = self.brush_label
           if label_color:
               cv.Circle(self.label_image, (x,y), self.brush_size, cv.RGB(label_color[0], label_color[1], label_color[2]), -1)
           cv.Circle(self.label_mat,   (x,y), self.brush_size, label_number, -1)

    def image_to_show( self ):
        cv.Copy(self.gray_image,self.visible_image)
        cv.Copy(self.label_image,self.visible_image,self.label_mat)
        return self.visible_image


    def handle_keypress(self, char_str):
        go_on = True
        if char_str == '=':
            self.increment_brush_size()
        elif char_str == '-':
            self.decrement_brush_size()
        elif char_str in ['0','1','2','3','4','5','6','7','8','9']:
            self.set_label(int(char_str))
        elif char_str == 'e':
            self.set_label(0)
        elif char_str == 'c':
            self.clear()
        elif char_str == 'p':
            self.print_label_info()
        elif char_str == 'h':
            self.help()
        elif char_str == 's':
            self.save()
            self.quit()
            return False
        elif char_str == 'q':
            self.quit()
            go_on = False
        elif char_str == 'i':
            self.zoom_in_more()
        elif char_str == 'o':
            self.zoom_out_more()
        return go_on

    def increment_brush_size(self):
        self.brush_size += 1
        print "Brush size: %d"%self.brush_size

    def decrement_brush_size(self):
        if self.brush_size > 1:
            self.brush_size -= 1
            print "Brush size: %d"%self.brush_size

    def set_label(self, label):
        if label in self.params.label_set.get_labels():
            print "Now using label: %s"%self.params.label_set.get_label_name(label)
            self.brush_label = label
        elif label == 0:
            print "Now using the eraser"
            self.brush_label = 0
        else:
            print "%d is not a valid label"%label

    def print_label_info(self):
        print "Label\tName\t\tColor (RGB)"
        for label in sorted( self.params.label_set.get_labels() ):
            label_name = self.params.label_set.get_label_name( label )
            label_color = self.params.label_set.get_label_color( label )
            print ("[%d]:\t%s\t\t%03d%03d%03d"%
                    (label, label_name,
                     label_color[0], label_color[1], label_color[2] ))

    def clear(self):
        cv.Set(self.label_image,(0,0,0))
        cv.Set(self.label_mat,0)

    def save(self):
        cv.Save(self.params.save_mat_path, self.label_mat)
        if self.params.save_image_path:
            cv.SaveImage(self.params.save_image_path, self.visible_image)


    def help(self):
        print "Key\tDescription"
        print "[h]\tDisplay this help screen"
        print "[-]\tDecrements brush size"
        print "[=]\tIncrements brush size"
        print "[1-9]\tSets brush to a given label"
        print "[e/0]\tSets to brush to an eraser"
        print "[c]\tClear all labels"
        print "[p]\tPrint label information"
        print "[s]\tSave and quit"
        print "[q]\tQuit without saving"



        
