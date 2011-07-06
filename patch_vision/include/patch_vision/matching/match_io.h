#ifndef __MATCH_IO_H__
#define __MATCH_IO_H__
/*
 * =====================================================================================
 *
 *       Filename:  match_io.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  07/06/2011 07:52:56 AM
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

using std::string;
using std::pair;
using std::vector;

typedef pair<float, float> PointT;

typedef struct Match{
    PointT point_c;
    PointT point_r;
    float  strength;
    int ref_idx;
} Match;

class MatchSet{
    public:
        MatchSet();
        ~MatchSet(){};

        void add_match( float x_c, float y_c, float x_r, float y_r, float strength=1, int ref_idx=-1 );
        void add_match( const PointT point_c, const PointT point_r, float strength=1, int ref_idx=-1 );

        int num_matches( ) const;

        void get_matches( vector<Match> &matches ) const;

        void save_to_file( string filename ) const;
        void read_from_file( string filename );

    private:
        void clear();
        vector<Match> _matches;

};

#endif
