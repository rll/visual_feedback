#ifndef __PT_IO_H__
#define __PT_IO_H__
/*
 * =====================================================================================
 *
 *       Filename:  feature_io.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/09/2011 05:46:06 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <iostream>
#include <fstream>
#include <vector>

namespace patch_vision{

using std::string;
using std::pair;
using std::vector;

typedef pair<float, float> PointT;

class PointSet{
    public:
        PointSet();
        ~PointSet();

        void add_point( float x, float y );
        void add_point( const PointT point );
        
        void get_points( vector<PointT> points ) const;
        PointT get_point( int index ) const;
        int num_points( ) const;
        
        void save_to_file( string filename ) const;
        void read_from_file( string filename );

    private:
        void clear();

        vector<PointT> _points;
        

};

};
#endif
