from numpy.distutils.core import setup
from numpy.distutils.misc_util import Configuration
 	
name = "cvgreyc"
version = '0.1'

def configuration(parent_package='', top_path=None):	
	config = Configuration(name, parent_package, top_path)
	config.add_subpackage('graphs')
	config.add_subpackage('imgproc')
	config.add_subpackage('features')
	config.add_subpackage('misc')

	return config

if __name__ == '__main__':
    setup(
		version = version,
		maintainer = "Alexis Mignon",
		maintainer_email = "alexis.mignon@info.unicaen.fr",
		url = "http://code.google.com/p/python-cvgreyc",
		configuration = configuration
	)
