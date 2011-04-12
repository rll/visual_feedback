from numpy.distutils.core import setup
from numpy.distutils.misc_util import Configuration

name = "graphs"

def configuration(parent_package='', top_path=None):
 	
    config = Configuration(name, parent_package, top_path)
 	
    config.add_extension("_neighbours",
 	        sources=["_neighbours.cpp","neighbours.cpp"],
 	        include_dirs=['.','../../include'],
 	    )
    config.add_extension("_pLaplacian",
 	        sources=["pLaplacian.cpp","../../src/graphs/pLaplacian.cpp","../../src/graphs/Graph.cpp"],
 	        include_dirs=['.','../../include'],
 	    )
	
    return config

if __name__ == '__main__':
    setup(**configuration(top_path='').todict())
