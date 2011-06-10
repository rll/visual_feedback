#ifndef __DESCRIPTORS_COMMON_H__
#define __DESCRIPTORS_COMMON_H__
/*
 * =====================================================================================
 *
 *       Filename:  descriptor.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/07/2011 04:06:27 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <patch_vision/extraction/descriptor.h>
#include <patch_vision/thirdparty/LBP.h>
class LBPDescriptor : public Descriptor{
    public:

        LBPDescriptor( int patch_size );
        ~LBPDescriptor( );

        void process_patch( const Mat &patch, vector<double> &feature );
        string name( ) const;
        int descriptor_size( ) const;
        int patch_size( ) const;

        int required_channels( ) const;
        
    private:
        int _patch_size;
};

#endif
