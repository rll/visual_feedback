import interpolation
import numpy as np

a = np.array(range(25),'float').reshape(5,5)
print "Original values :"
print a
print "Shift (0.5,0.5)"
def_map = np.ones((5,5,2))*0.5
print interpolation.interpolate_map(a,def_map)
print "Shift (0.25,0.25)"
def_map = np.ones((5,5,2))*0.25
print interpolation.interpolate_map(a,def_map)

coords = np.random.rand(10,2)
print "Sampling at positions :"
print coords
print interpolation.interpolate_list(a,coords)


