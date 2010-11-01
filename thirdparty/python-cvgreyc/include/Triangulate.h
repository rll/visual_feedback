/**
 *  This module wraps the C++ code for Delaunay triangulation from
 *  the matlib.delaunay module.
 *  
 *  We just want to give an interface consistent with CVGreyc framework.
 */

#include "NDArray.h"
#include "VoronoiDiagramGenerator.h"

//#include <iostream>
//using namespace std;
#include <typeinfo>

#define EDGE0(node) ((node + 1) % 3)
#define EDGE1(node) ((node + 2) % 3)



namespace CVGreyc {
namespace Triangulate {

template<typename T>
inline bool _on_right( T x0, T y0, T x1, T y1, T x, T y) {
	return  ((y0-y)*(x1-x) > (x0-x)*(y1-y));
}

class Triangulation {
	private :
	size_t _nnodes;
	size_t _nedges;
	size_t _ntriangles;
	const NDArray::ArrayBase<double>&  _positions;
	NDArray::ArrayBase<int>*     _edges;
	
	NDArray::ArrayBase<double>*  _centers;
	NDArray::ArrayBase<int>*     _nodes;
	NDArray::ArrayBase<int>*     _neighbours;
		
	void _reorder_edges();
	void _compute_delaunay();
		
	public:
	
	Triangulation(const NDArray::ArrayBase<double>& positions);
	~Triangulation();
		
	size_t nnodes() const { return _nnodes; }
	size_t ntriangles() const { return _ntriangles; }
	size_t nedges() const { return _nedges; }
		
	const NDArray::ArrayBase<double>& positions()  const { return _positions; }
	NDArray::ArrayBase<int>&    edges()      const { return *_edges; }
	NDArray::ArrayBase<double>& centers()    const { return *_centers; }
	NDArray::ArrayBase<int>&    nodes()      const { return *_nodes; }
	NDArray::ArrayBase<int>&    neighbours() const { return *_neighbours; }

};


Triangulation::Triangulation(const NDArray::ArrayBase<double>& positions):
	_nnodes(positions.shape(0)),_ntriangles(0),
	_positions(positions),
	_edges(0), _centers(0), _nodes(0), _neighbours(0)
	{_compute_delaunay();}

Triangulation::~Triangulation(){
	delete _edges;	
	delete _centers;
	delete _nodes;
	delete _neighbours;
}

/**
 * method : void _reorder_edges()
 * 
 * Order the edges in trianlge counter-clockwise and
 * reorder neighbours correspondingly
 * 
 * This code is once again adapted from matlib.delaunay.
 * Original comments are reported as they are.
 * My own comments are prefixed by **.
 */

void Triangulation::_reorder_edges(){
	int neighbours[3], nodes[3];
	int i, tmp;
	int case1, case2;
	
	for (i=0; i<(int)_ntriangles; i++){
		nodes[0] = (*_edges)((*_nodes)(i,0),0); // ** the first node (origin) of the first edge of the current triangle
		nodes[1] = (*_edges)((*_nodes)(i,0),1); // ** the second node (destination) of the same edge.
		tmp = (*_edges)((*_nodes)(i,1),0);      // ** the first node of the second edge in the triangle.
		if (tmp == nodes[0] ) { // ** the 2 edges have the same origin
			case1 = 1;
			nodes[2] = (*_edges)((*_nodes)(i,1),1) ; // ** The end of the second edge
		} else if (tmp == nodes[1] ) { // ** the second edge starts at the end of the first one
			case1 = 0;
			nodes[2] = (*_edges)((*_nodes)(i,1),1) ; // ** The end extremity of the second edge
		} else if ( (*_edges)((*_nodes)(i,1),1) == nodes[0] ){ // ** the second edge ends at the origin of the first one
			case1 = 1;
			nodes[2] = tmp;
		} else {	// ** both edges have the same desination node.
			case1 = 0 ;
			nodes[2] = tmp;
		}
		if (_on_right(_positions(nodes[0],0),_positions(nodes[0],1),
		              _positions(nodes[1],0),_positions(nodes[1],1),
					  _positions(nodes[2],0),_positions(nodes[2],1)
					 )
		   ){
			// flip to make counter-clockwise
			tmp = nodes[2];
			nodes[2] = nodes[1];
			nodes[1] = tmp;	   
			case2 = 1;
		} else case2 = 0;
		
		// I worked it out on paper. You're just gonna have to trust me
		// on this.
        if (!case1 && !case2) {
            neighbours[0] = (*_neighbours)( i, 1);
            neighbours[1] = (*_neighbours)( i, 2);
            neighbours[2] = (*_neighbours)( i, 0);
        } else if (case1 && !case2) {
            neighbours[0] = (*_neighbours)( i, 2);
            neighbours[1] = (*_neighbours)( i, 1);
            neighbours[2] = (*_neighbours)( i, 0);
        } else if (!case1 && case2) {
            neighbours[0] = (*_neighbours)( i, 1);
            neighbours[1] = (*_neighbours)( i, 0);
            neighbours[2] = (*_neighbours)( i, 2);
        } else {
            neighbours[0] = (*_neighbours)( i, 2);
            neighbours[1] = (*_neighbours)( i, 0);
            neighbours[2] = (*_neighbours)( i, 1);
        }

        // Not trusting me? Okay, let's go through it:
        // We have three edges to deal with and three nodes. Without loss
        // of generality, let's label the nodes A, B, and C with (A, B) 
        // forming the first edge in the order they arrive on input.
        // Then there are eight possibilities as to how the other edge-tuples
        // may be labeled, but only two variations that are going to affect the
        // output:
        //
        // AB         AB
        // BC (CB)    AC (CA)
        // CA (AC)    BC (CB)
        //
        // The distinction is whether A is in the second edge or B is.
        // This is the test "case1" above.
        //
        // The second test we need to perform is for counter-clockwiseness.
        // Again, there are only two variations that will affect the outcome:
        // either ABC is counter-clockwise, or it isn't. In the former case,
        // we're done setting the node order, we just need to associate the 
        // appropriate neighbor triangles with their opposite nodes, something
        // which can be done by inspection. In the latter case, to order the
        // nodes counter-clockwise, we only have to switch B and C to get
        // nodes ACB. Then we simply set the neighbor list by inspection again.
        //
        //          CCW     CW
        // AB
        // BC       120     102 -+
        // CA                    |
        //                       +- neighbor order
        // AB                    |
        // AC       210     201 -+
        // BC
        //          ABC     ACB -+- node order


        (*_nodes)(i,0) = nodes[0];
        (*_nodes)(i,1) = nodes[1];
        (*_nodes)(i,2) = nodes[2];
        (*_neighbours)(i,0) = neighbours[0];
        (*_neighbours)(i,1) = neighbours[1];
        (*_neighbours)(i,2) = neighbours[2];
	}
}

inline void compute_plane_coefs(double x0, double y0, double f0,
                                double x1, double y1, double f1,
                                double x2, double y2, double f2,
								double& a, double& b, double& c){
	double df10, df02, df21 ;
	double denom;
								
	df10 = f1-f0;
	df02 = f0-f2;
	df21 = f2-f1;
	
	denom = (x1-x0)*y2 + (x0-x2)*y1 + (x2-x1)*y0 ;
	if (denom == 0.0) {
		a = 0;
		b = 0;
		c = 0;
	}
	else {
		a =  ( df10*y2 + df02*y1 + df21*y0 ) / denom ;
		b = -( df10*x2 + df02*x1 + df21*x0 ) / denom ;
		c = ( (f0*x1 - f1*x0)*y2 + (f2*x0 - f0*x2)*y1 + (f1*x2 - f2*x1)*y0 ) / denom;
	}
}

void Triangulation::_compute_delaunay(){
	double *xs;
	double *ys;
	bool own_pos = false; // Do the current method own the position vectors ?
	
	// We have to first make sure that position data are store the good way.
	// If array is a CArray with XYInder or FIndexer (equivalent for 2D)
	// then we can use the wrapped array directly. Otherwise we create
	// explicitly the arrays.
	
	try { // If we use FIndexer
		const NDArray::CArray<double,NDArray::FIndexer>& pos = dynamic_cast<const NDArray::CArray<double,NDArray::FIndexer>&>(_positions);
		xs = &pos.data()[pos.index()(0,0)];
		ys = &pos.data()[pos.index()(0,1)];
	}
	catch (std::bad_cast){
		try { // If we use XYIndexer
			
			const NDArray::CArray<double,NDArray::XYIndexer>& pos = dynamic_cast<const NDArray::CArray<double,NDArray::XYIndexer>&>(_positions);
			xs = &pos.data()[pos.index()(0,0)];
			ys = &pos.data()[pos.index()(0,1)];
		}
		catch (std::bad_cast) { // Otherwise

			xs = new double[_nnodes];
			ys = new double[_nnodes];
			for (size_t i = 0; i < _positions.shape(0); i++){
				xs[i] = _positions(i,0);
				ys[i] = _positions(i,1);
			}
			own_pos = true;
		}
	}

	double xmin,xmax,ymin,ymax;
	double x,y;
	xmin = xmax = xs[0];
	ymin = ymax = ys[0];

	for (size_t i = 1 ; i < _positions.shape(0) ; i++){
		x = _positions(i,0);
		y = _positions(i,1);
		if      (x > xmax) xmax = x;
		else if (x < xmin) xmin = x;
		
		if      (y > ymax) ymax = y;
		else if (y < ymin) ymin = y;
	}

	// We use VoronoiDiagramGenerator to build the Voronoi
	// diagram and then build the delaunay triangulation

	VoronoiDiagramGenerator vdg;
	vdg.generateVoronoi(xs,ys,_nnodes,xmin,xmax,ymin,ymax);

	int dummy, ntriangles; 
	vdg.getNumbers(dummy,ntriangles);
	_ntriangles = (size_t) ntriangles;

	double tri0x,tri0y,tri1x,tri1y;
	int    tri0,tri1;
	int    reg0,reg1;

	_nedges = 0;	
	vdg.resetEdgeListIter();
	// Count the number of edges.
	while ( vdg.getNextDelaunay(tri0,tri0x,tri0y,tri1,tri1x,tri1y,reg0,reg1) ) _nedges++ ;

	_edges      = new NDArray::CArray<int>   (_nedges,2);
	_centers    = new NDArray::CArray<double>(_ntriangles,2);
	_nodes      = new NDArray::CArray<int>   (_ntriangles,3);
	_neighbours = new NDArray::CArray<int>   (_ntriangles,3);

	_nodes->setValue(-1);
	_edges->setValue(-1);
	_neighbours->setValue(-1);

	vdg.resetEdgeListIter();
	int iedge = -1 ;
	// Code here is adapted from the matlib.delaunay package in file
	// _delaunay.cpp
	while ( vdg.getNextDelaunay(tri0,tri0x,tri0y,tri1,tri1x,tri1y,reg0,reg1) ){
		iedge++;
		// We get the nodes at each end of the edge
		(*_edges)(iedge,0) = reg0;
		(*_edges)(iedge,1) = reg1;
		if ( tri0 > -1 ) { // There is a triangle there (this side of
		                   // the edge is not outside the hull)
			(*_centers)(tri0,0) = tri0x;
			(*_centers)(tri0,1) = tri0y;
			for (size_t j = 0; j<3 ; j++){
				if ( (*_nodes)(tri0,j) == iedge) break; // ** How can it occur ? **
				if ( (*_nodes)(tri0,j) == -1 ) { // Not filled yet
					 (*_nodes)(tri0,j) = iedge;
					 (*_neighbours)(tri0,j) = tri1;
					 break;
				}
			}
		}
		
		if (tri1 > -1 ) {
				
			(*_centers)(tri1,0) = tri1x;
			(*_centers)(tri1,1) = tri1y;
			for (size_t j = 0; j<3 ; j++){
				if ( (*_nodes)(tri1,j) == iedge) break; // ** How can it occur ? **
				if ( (*_nodes)(tri1,j) == -1 ) { // Not filled yet
					 (*_nodes)(tri1,j) = iedge;
					 (*_neighbours)(tri1,j) = tri0;
					 break;
				}
			}
		}
	}

	_reorder_edges();

	if (own_pos) {
		delete [] xs;
		delete [] ys;	
	}
}
/**
 *  Computes linear interpolation of a function over the nodes of
 *  a triangulation.
 */

class LinearInterpolator {
	private :
	const Triangulation& _triangulation;
	const NDArray::ArrayBase<double>& _function;
	NDArray::ArrayBase<double>* _planes;
	
	void _compute_planes();
	int _find_containing_triangle(double targeti, double targetj);
	
	public:
	LinearInterpolator(const Triangulation& triangulation, const NDArray::ArrayBase<double>& function);
	~LinearInterpolator();
	
	NDArray::ArrayBase<double>& planes() const { return *_planes ;}
	
	double interpolate_one( double i, double j, size_t channel);
	void   interpolate_many(const NDArray::ArrayBase<double>& positions, NDArray::ArrayBase<double>& result);
	void   interpolate_grid(double imin, double imax, double jmin, double jmax,
							 
							 NDArray::ArrayBase<double>& result);		
	
	
};

LinearInterpolator::LinearInterpolator(const Triangulation& triangulation, const NDArray::ArrayBase<double>& function):
	_triangulation(triangulation), _function(function) ,
	_planes(new NDArray::CArray<double>(triangulation.ntriangles(),3,function.shape(1))){
	_compute_planes();
}

LinearInterpolator::~LinearInterpolator(){
	delete _planes;	
}

void LinearInterpolator::_compute_planes() {
	// We sovle 
	// a*x0 + b*y0 + c = f0
	// a*x1 + b*y1 + c = f1
	// a*x2 + b*y2 + c = f2
	//
	
	double a,b,c ;	
	double f0,f1,f2;
	int n0,n1,n2;
	double x0,x1,x2;
	double y0,y1,y2;
	size_t index[2];
	
	for (size_t i = 0; i < _triangulation.ntriangles() ; i++){
		n0 = _triangulation.nodes()(i,0);
		n1 = _triangulation.nodes()(i,1);
		n2 = _triangulation.nodes()(i,2);
		
		x0 = _triangulation.positions()(n0,0);
		y0 = _triangulation.positions()(n0,1);
		x1 = _triangulation.positions()(n1,0);
		y1 = _triangulation.positions()(n1,1);
		x2 = _triangulation.positions()(n2,0);
		y2 = _triangulation.positions()(n2,1);
		
		for (size_t j = 0 ; j< _function.shape(1) ; j++){
			// '_function' can be 3 or 2-dimensional so we use an index
			// array to avoid checking the dimension of '_function'
			index[1] = j;	
			index[0] = n0;
			f0 = _function(index);
			index[0] = n1;
			f1 = _function(index);
			index[0] = n2;
			f2 = _function(index);
			compute_plane_coefs(x0,y0,f0,x1,y1,f1,x2,y2,f2,a,b,c);
			// _planes is always 3-dimensional
			(*_planes)(i,0,j) = a;
			(*_planes)(i,1,j) = b;
			(*_planes)(i,2,j) = c;
		}
	}

	
}



int LinearInterpolator::_find_containing_triangle(double itarget, double jtarget){
    int i, j, k, t;
	
	// for convenience
	const NDArray::ArrayBase<double>& pos   = _triangulation.positions();
	const NDArray::ArrayBase<int>&    nodes = _triangulation.nodes();
	
    t = 0;
    while (1) {
        for (i=0; i<3; i++) {
            j = EDGE0(i);
            k = EDGE1(i);
            if (_on_right(pos(nodes(t,j),0), pos(nodes(t,j),1),
                          pos(nodes(t,k),0), pos(nodes(t,k),1),
                        itarget, jtarget)) {
                t = _triangulation.neighbours()(t, i);
                if (t < 0) return t; // outside the convex hull
                break;
            }
        }
        if (i == 3) break;
    }
    return t;
}

double LinearInterpolator::interpolate_one(double inti, double intj , size_t channel = 0){
	int t = _find_containing_triangle(inti,intj);
	if (t<0) return 0.;
	return (*_planes)(t,0,channel)*inti + (*_planes)(t,1,channel)*intj + (*_planes)(t,2,channel);
}

void LinearInterpolator::interpolate_many(const NDArray::ArrayBase<double>& positions, NDArray::ArrayBase<double>& result){
	size_t index[2];
	for (size_t i = 0 ; i<positions.shape(0); i++){
		index[0] = 0;
		for (size_t c = 0 ; c<_function.shape(1) ; c++){
			index[1] = c ;
			result(index) = interpolate_one(positions(i,0),positions(i,1),c);
		}
	}	
}

void LinearInterpolator::interpolate_grid(double imin, double imax,
                                           double jmin, double jmax,
										   NDArray::ArrayBase<double>& result){
	size_t isteps = result.shape(0);
	size_t jsteps = result.shape(1);
	double di = (imax-imin)/(isteps-1);
	double dj = (jmax-jmin)/(jsteps-1);
	double inti,intj ;
	size_t index[3];

	for (size_t i = 0 ; i< isteps ; i++){
		inti = imin + di*i;
		index[0] = i;
		for (size_t j = 0 ; j < jsteps ; j++){
			intj = jmin + dj*j ;
			index[1] = j;
			for (size_t c = 0 ; c<_function.shape(1) ; c++){
				index[2] = c ;
				result(index) = interpolate_one(inti,intj,c);
			}
		}
	}	
}


} // namespace Triangulation
} // namespace CVGreyc
