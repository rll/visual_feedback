/*
 * =====================================================================================
 *
 *       Filename:  feature_io.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/09/2011 05:56:18 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */
#include <patch_vision/slicing/pt_io.h>

using std::ofstream;
using std::ifstream;
using std::endl;
using std::cout;

namespace patch_vision{

PointSet :: PointSet( ){
    clear();
}

PointSet :: ~PointSet( ){ }

void PointSet :: add_point( float x, float y ){
    add_point( PointT(x,y) );
}

void PointSet :: add_point( const PointT point ){
    _points.push_back(point);
}

void PointSet :: get_points( vector<PointT> points ) const{
    for( size_t i = 0; i < _points.size(); i++ ){
        points.push_back( _points[i] );
    }
}

PointT PointSet :: get_point( int index ) const{
    return _points[index];
}

int PointSet :: num_points ( ) const{
    return _points.size();
}

void PointSet :: save_to_file( string filename ) const{
    ofstream output_file( filename.c_str() );
    output_file << _points.size() << endl;
    for( size_t i = 0; i < _points.size(); i++ ){
        PointT pt = _points[i];
        output_file <<  pt.first  << " " << pt.second << endl;
    }
    output_file.close();
}


void PointSet :: read_from_file( string filename ){
    ifstream input_file( filename.c_str() );
    size_t n_points;
    input_file >> n_points;
    for ( size_t i = 0; i < n_points; i++ ){
        PointT pt;
        input_file >> pt.first;
        input_file >> pt.second;
        add_point( pt );
    }
}



void PointSet :: clear(){
    _points.clear();
}

};
