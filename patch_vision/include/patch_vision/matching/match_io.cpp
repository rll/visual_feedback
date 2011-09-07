/*
 * =====================================================================================
 *
 *       Filename:  match_io.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  07/06/2011 07:53:11 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */
#include <patch_vision/matching/match_io.h>

namespace patch_vision{

using std::ofstream;
using std::ifstream;
using std::endl;
using std::cout;

MatchSet :: MatchSet( ){
    clear();
}


void MatchSet :: add_match( float x_c, float y_c, float x_r, float y_r, float strength, int ref_idx ){
    add_match( PointT(x_c,y_c), PointT(x_r,y_r), strength );
}
void MatchSet :: add_match( const PointT point_c, const PointT point_r, float strength, int ref_idx ){
    Match match;
    match.point_c = PointT(point_c);
    match.point_r = PointT(point_r);
    match.strength = strength;
    match.ref_idx = ref_idx;
    _matches.push_back(match);
}

int  MatchSet :: num_matches( ) const{
    return _matches.size();
}

void MatchSet :: get_matches( vector<Match> &matches ) const{
    for (unsigned int i = 0; i < _matches.size(); i++ ){
        Match match;
        match.point_c = PointT(_matches[i].point_c);
        match.point_r = PointT(_matches[i].point_r);
        match.strength = _matches[i].strength;
        match.ref_idx = _matches[i].ref_idx;
        matches.push_back(match);
    }
}

void MatchSet :: save_to_file( string filename ) const{
    ofstream output_file( filename.c_str() );
    output_file << _matches.size() << endl;
    for( size_t i = 0; i < _matches.size(); i++ ){
        PointT pt_c = _matches[i].point_c;
        PointT pt_r = _matches[i].point_r;
        float strength = _matches[i].strength;
        int ref_idx = _matches[i].ref_idx;
        output_file <<  pt_c.first  << " " << pt_c.second << "\t"
                    <<  pt_r.first  << " " << pt_r.second << "\t"
                    <<  strength << " " << ref_idx << endl;
    }
    output_file.close();
}
void MatchSet :: read_from_file( string filename ){
    ifstream input_file( filename.c_str() );
    size_t num_matches;
    input_file >> num_matches;
    for( size_t i = 0; i < num_matches; i++ ){
        PointT pt_c;
        PointT pt_r;
        float strength;
        int ref_idx;
        input_file  >>  pt_c.first  >> pt_c.second 
                    >>  pt_r.first  >> pt_r.second
                    >>  strength    >> ref_idx;
        add_match( pt_c, pt_r, strength, ref_idx );
    }
    input_file.close();

}

void MatchSet :: clear(){
    _matches.clear();
}

};
