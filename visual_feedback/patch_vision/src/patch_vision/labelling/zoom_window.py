import roslib
roslib.load_manifest("patch_vision")
import cv
import re
from patch_vision.labelling.label_set import LabelSet

cvkeymappings = {}
for i,charcode in enumerate(range(ord('a'),ord('z')+1)):
    cvkeymappings[1048673+i] = chr(charcode)
for i,charcode in enumerate(range(ord('0'),ord('9')+1)):
    cvkeymappings[1048624+i] = chr(charcode)
cvkeymappings[1113937] = 'LEFT'
cvkeymappings[1113938] = 'UP'
cvkeymappings[1113939] = 'RIGHT'
cvkeymappings[1113940] = 'DOWN'
cvkeymappings[1048621] = '-'
cvkeymappings[1048637] = '='

class ZoomWindow:
    def __init__(self, name = "Viewer", update_period = 100, zoom_out = 1):
        self.name = name
        self.update_period = update_period
        self.zoom_out = zoom_out
        self.current_top_left = (0,0)
        cv.NamedWindow(self.name)
        cv.SetMouseCallback(self.name,self.handleEventsZoomed,0)
        if update_period > 0:
            while(True):
                if not self.update():
                    break
    def handleEventsZoomed(self, event, x, y, flags, param):
        new_x = self.current_top_left[0] + x*self.zoom_out
        new_y = self.current_top_left[1] + y*self.zoom_out
        self.handleEvents( event, new_x, new_y, flags, param)


    def update(self,keycode=None):
        go_on = True
        self.show_image()
        if not keycode:
            keycode = cv.WaitKey(self.update_period)
        if keycode in cvkeymappings.keys():
            char_str = cvkeymappings[keycode]
            go_on = self.handle_keypress( char_str )
        return go_on
        
    def show_image(self):
        unscaled_image = self.image_to_show()
        if self.zoom_out == 1:
            scaled_image = unscaled_image
        else:
            scaled_image = cv.CreateImage(
                            ( unscaled_image.width/self.zoom_out, 
                              unscaled_image.height/self.zoom_out ),
                            unscaled_image.depth, 3 )
            cv.Resize(unscaled_image, scaled_image)
        cv.ShowImage(self.name,scaled_image)

    def quit(self):
        cv.DestroyWindow(self.name)
    
    def image_to_show( self ):
        abstract

    def handle_keypress( self, char_str ):
        if char_str == 'q':
            self.quit()
            return False
        return True

    def zoom_in_more(self):
        if self.zoom_out >= 2:
            self.zoom_out /= 2

    def zoom_out_more(self):
        self.zoom_out *= 2

    def handleEvents( self, event, x, y, flags, param ):
        pass
