from numpy.distutils.core import setup
from numpy.distutils.misc_util import Configuration
import os
name = "features"

# These variables should be modified if opencv2.x is not installed 
# with packages on your system
OPENCV_CFLAGS = os.popen("pkg-config --cflags opencv").read().strip()
OPENCV_LIBS = os.popen("pkg-config --libs opencv").read().strip()

def configuration(parent_package='', top_path=None):
 	
	config = Configuration(name, parent_package, top_path)
 	
	config.add_extension("_lbp",
			sources=["LBP.cpp"],
			include_dirs=['.','../../include'],
 		)
 	
	config.add_extension("_sift",
			sources=["siftmodule.cpp","../../src/siftpp/sift.cpp"],
			include_dirs=['.','../../src/siftpp/'],
			extra_compile_args = ['-D VL_USEFASTMATH'],
 		)

	if OPENCV_CFLAGS != '' :
		SURF_DIR = '../../src/OpenSURF/'
		SURF_SRC = ['fasthessian.cpp','integral.cpp','surf.cpp','utils.cpp','ipoint.cpp' ]
		config.add_extension("_surf",
	 			sources=["surf.cpp"]+[SURF_DIR+src for src in SURF_SRC],
	 			include_dirs=['.',"../../include/",SURF_DIR],
	 			extra_compile_args = [OPENCV_CFLAGS,'-D LINUX'],
	 			extra_link_args    = [OPENCV_LIBS],
	 		)

		config.add_extension("_surf2",
	 			sources=["surf2.cpp"],
	 			include_dirs=['.',"../../include/"],
	 			extra_link_args    = [OPENCV_LIBS],
	 			extra_compile_args = [OPENCV_CFLAGS],
	 		)
	return config

if __name__ == '__main__':
	setup(**configuration('cvgreyc',top_path='').todict())

