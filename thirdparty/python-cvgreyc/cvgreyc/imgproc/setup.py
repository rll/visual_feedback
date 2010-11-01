from numpy.distutils.core import setup
from numpy.distutils.misc_util import Configuration
 	
version = 0.1

def configuration(parent_package='', top_path=None):	
	config = Configuration('imgproc', parent_package, top_path)
	config.add_subpackage('interpolation')

	return config

if __name__ == '__main__':
    setup(**configuration().todict())
