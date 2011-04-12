from numpy.distutils.core import setup
from numpy.distutils.misc_util import Configuration
 	
def configuration(parent_package='', top_path=None):
 	
    config = Configuration('interpolation', parent_package, top_path)
 	
    config.add_extension("_interpolation",
 	        sources=["interpolation.cpp"],
 	        include_dirs=['../../../include/'],
 	    )
	
    return config

if __name__ == '__main__':
    setup(**configuration(top_path='').todict())

