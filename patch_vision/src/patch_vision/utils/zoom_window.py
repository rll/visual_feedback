import roslib
roslib.load_manifest("patch_vision")
import cv
import re
import inspect

#different keymappings for different systems
cvkeymappings = {}

#first set of keymappings
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
cvkeymappings[1048586] = 'ENTER'

#second set
for i in range(0, 128):
    cvkeymappings[i] = chr(i)
cvkeymappings[65361] = 'LEFT'
cvkeymappings[65362] = 'UP'
cvkeymappings[65363] = 'RIGHT'
cvkeymappings[65364] = 'DOWN'


class KeyCommandInfo:
    def __init__(self, key, description, exits):
        self.key = key
        self.description = description
        self.exits = exits

def keycommand( key, description="", exits=False):
    def wrap(f):
        f.kc_info = KeyCommandInfo( key, description, exits )
        return f
    return wrap


def update_all_windows( period=100 ):
    keycode = cv.WaitKey( period )
    if not keycode in cvkeymappings.keys():
        char_str = "NONE"
    else:
        char_str = cvkeymappings[keycode]
    cont = True
    cont &= global_handle_keypress( char_str )
    for obj in key_commanded_objects:
        cont &= obj.update( char_str )
    return cont
    


class KeyCommandedObject(object):
    def __init__(self):
        if not hasattr(self, "keys_to_methods"):
            self.keys_to_methods = {}
        for name,member in inspect.getmembers(self):
            if hasattr(member,"kc_info"):
                self.keys_to_methods[member.kc_info.key] = member
        if not 'key_commanded_objects' in globals().keys():
            global key_commanded_objects
            key_commanded_objects = []
        key_commanded_objects.append(self)

    def update( self, char_str ):
        self.handle_keypress( char_str )

    def handle_keypress( self, char_str ):
        if not char_str in self.keys_to_methods.keys():
            return True
        fxn = self.keys_to_methods[char_str]
        fxn()
        return not fxn.kc_info.exits

class InteractiveWindow(KeyCommandedObject):
    def __init__(self, name, update_period = 100):
        KeyCommandedObject.__init__(self)
        self.name = name
        self.update_period = update_period
        cv.NamedWindow(self.name)
        cv.SetMouseCallback(self.name,self.handleEvents,0)
        if update_period > 0:
            #If I am given an update period, I update til I am closed
            cont = True
            while(cont):
                cont = self.update()

    def update(self,char_str=None):
        go_on = True
        self.show_image()
        if not char_str:
            keycode = cv.WaitKey(self.update_period)
            if keycode in cvkeymappings.keys():
                char_str = cvkeymappings[keycode]
            else:
                return True
        go_on = self.handle_keypress( char_str )
        return go_on


    @keycommand('q', "Quits without saving", exits=True )
    def quit(self):
        cv.DestroyWindow(self.name)

    @keycommand('h', description="Prints help", exits=False)
    def print_help(self):
        print "Commands for Window: %s"%self.name
        print "Key\tDescription"
        for key in sorted(self.keys_to_methods.keys()):
            fxn = self.keys_to_methods[key]
            print "%s\t%s"%(key, fxn.kc_info.description)
    

    def show_image(self):
        abstract
    
    def handleEvents( self, event, x, y, flags, param ):
        pass


class ZoomWindow(InteractiveWindow):
    def __init__(self, name = "Viewer", update_period = 100, zoom_out = 1):
        self.zoom_out = zoom_out
        self.current_top_left = (0,0)
        InteractiveWindow.__init__(self, name, update_period)

    def handleEvents(self, event, x, y, flags, param):
        new_x = self.current_top_left[0] + x*self.zoom_out
        new_y = self.current_top_left[1] + y*self.zoom_out
        self.handleEventsUnzoomed( event, new_x, new_y, flags, param)

    def handleEventsUnzoomed(self, event, x, y, flags, param):
        pass
        
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

    
    def image_to_show( self ):
        abstract

    @keycommand('i', "Zoom in more" )
    def zoom_in_more(self):
        if self.zoom_out >= 2:
            self.zoom_out /= 2

    @keycommand('o', "Zoom out more" )
    def zoom_out_more(self):
        self.zoom_out *= 2

## Optionally, can have 'global' key commands which aren't assigned to a window. Not recommended. ##
def globalkeycommand( key, description="", exits=False):
    if not "keys_to_methods" in globals().keys():
        global keys_to_methods
        keys_to_methods = {}
    def wrap(f):
        keys_to_methods[key] = f
        f.kc_info = KeyCommandInfo(key, description, exits) 
        return f
    return wrap

def global_update( keycode ):
    if not keycode in cvkeymappings.keys():
        return True
    char_str = cvkeymappings[keycode]
    return global_handle_keypress( char_str )

def global_handle_keypress( char_str ):
    if not "keys_to_methods" in globals().keys():
        return True
    if not char_str in keys_to_methods.keys():
        return True
    fxn = keys_to_methods[char_str]
    fxn()
    return not fxn.kc_info.exits


@globalkeycommand( 'h', "Outputs help for global commands", exits=False)
def global_print_help():
    print "Global commands:"
    print "Key\tDescription"
    for key in sorted(keys_to_methods.keys()):
        fxn = keys_to_methods[key]
        print "%s\t%s"%(key, fxn.kc_info.description)
