// file:        siftmodule.cpp
// author:      Dylan Sale
// description: Sift python interface
// Year: 2008
// License: This code licensed under the following:
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//1. Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//2. Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//3. The name of the author may not be used to endorse or promote products
//   derived from this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
//IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
//OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
//NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
//THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#include "sift.hpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cstdio>

using namespace std;

/* keypoint list */
typedef vector< pair<VL::Sift::Keypoint,VL::float_t> > Keypoints ;

/* predicate used to order keypoints by increasing scale */
bool cmpKeypoints (Keypoints::value_type const&a,
				   Keypoints::value_type const&b) {
					   return a.first.sigma < b.first.sigma ;
}

const int maxval = 255; //the max value that a pixel can have

static float log2(float x)
{
	static float ln2 = log(2.0);
	return log(x)/ln2;
}

struct tempKeypoint {
	tempKeypoint(float x_,float y_,float sig_,float th_):x(x_),y(y_),sig(sig_),th(th_){}
	float x,y,sig,th;
};

typedef std::vector<tempKeypoint> keyVector;

const int initLevels = 3;

#include <iostream>
using namespace std;

extern "C" {


float* run_sift(
	int height,
	int width,
	float* image,
	int* nkeypoints,
	float* keypoints,

	int    first          ,
	int    octaves        ,
	int    levels         ,
	float  threshold      ,
	float  edgeThreshold  ,
	float  magnif         ,
	int    noorient       ,
	int    stableorder    ,
	int    savegss        ,
	int    verbose        ,
	int    binary         ,
	int    haveKeypoints  ,
	int    unnormalized   
/*	int    first          = -1 ,
	int    octaves        = -1 ,
	int    levels         = initLevels ,
	float  threshold      = 0.04f / levels / 2.0f ,
	float  edgeThreshold  = 10.0f,
	float  magnif         = 3.0 ,
	int    noorient       = 0 ,
	int    stableorder    = 0 ,
	int    savegss        = 0 ,
	int    verbose        = 0 ,
	int    binary         = 0 ,
	int    haveKeypoints  = 0 ,
	int    unnormalized   = 0*/
	){
		
	float* outKeypoints;

	VL::PgmBuffer buffer;
	buffer.width = width;
	buffer.height = height;
	buffer.data = image;

	keyVector keypointList;
	if ( *nkeypoints > 0){
		haveKeypoints = true;
		float x,y,sigma,th;
		
		for (int i = 0; i< *nkeypoints; i++){
			x     = keypoints[4*i];
			y     = keypoints[4*i + 1];
			sigma = keypoints[4*i + 2];
			th    = keypoints[4*i + 3];
			tempKeypoint k(x,y,sigma,th);
			keypointList.push_back(k);
		}
	}
	if(levels != initLevels && threshold == 0.04f / initLevels / 2.0f) //only recalculate it if it wasnt set manually
		threshold = 0.04f / levels / 2.0f ;	

	int         O      = octaves ;    
	int const   S      = levels ;
	int const   omin   = first ;
	float const sigman = .5 ;
	float const sigma0 = 1.6 * powf(2.0f, 1.0f / S) ;

	// optionally autoselect the number number of octaves
	// we downsample up to 8x8 patches
	if(O < 1) {
		O = std::max
		(int
		 (std::floor
		  (log2
		   ((double)std::min(buffer.width,buffer.height))) - omin -3), 1) ;
	}
	if(verbose) printf("init scale space\n");



	// initialize scalespace
	VL::Sift sift(buffer.data, buffer.width, buffer.height, 
		sigman, sigma0,
		O, S,
		omin, -1, S+1) ;

	if(!haveKeypoints)
	{
		if(verbose) printf("detecting keypoints\n");

		sift.detectKeypoints(threshold, edgeThreshold) ;
	}

	// -------------------------------------------------------------
	//                  Run SIFT orientation detector and descriptor
	// -------------------------------------------------------------    

	/* set descriptor options */
	sift.setNormalizeDescriptor( ! unnormalized ) ;
	sift.setMagnification( magnif ) ;

	if(verbose) printf("compute descriptors\n");


	if( haveKeypoints ) {

		if(verbose) printf("using keypoints\n");
		// -------------------------------------------------------------
		//                 Reads keypoint from file, compute descriptors
		// -------------------------------------------------------------
		Keypoints keypoints ;

		for(keyVector::iterator keypointIterator = keypointList.begin();
			keypointIterator != keypointList.end();
			keypointIterator ++)
		{
			tempKeypoint k = *keypointIterator;

			/* compute integer components */
			VL::Sift::Keypoint key 
				= sift.getKeypoint(k.x,k.y,k.sig) ;

			Keypoints::value_type entry ;
			entry.first  = key ;
			entry.second = k.th ;
			keypoints.push_back(entry) ;
		}

		/* sort keypoints by scale if not required otherwise */
		if(! stableorder)
			sort(keypoints.begin(), keypoints.end(), cmpKeypoints) ;

		*nkeypoints = keypoints.size();
		outKeypoints = (float*)malloc(sizeof(float)* *nkeypoints * (4+128));

		// process in batch
		int i = 0;
		for(Keypoints::const_iterator iter = keypoints.begin() ;
			iter != keypoints.end() ;
			++iter) {
				VL::Sift::Keypoint const& key = iter->first ;
				VL::float_t th = iter->second ;

				///* write keypoint */
				//out << setprecision(2) << key.x     << " "
				//	<< setprecision(2) << key.y     << " "
				//	<< setprecision(2) << key.sigma << " "
				//	<< setprecision(3) << th ;

				/* compute descriptor */
				VL::float_t descr [128] ;
				sift.computeKeypointDescriptor(descr, key, th) ;

				outKeypoints[132*i    ] = key.x;
				outKeypoints[132*i + 1] = key.y;
				outKeypoints[132*i + 2] = key.sigma;
				outKeypoints[132*i + 3] = th;
				for (int di = 0; di < 128; di++) outKeypoints[132*i + 4 + di] = descr[di];
				i++;
		} // next keypoint

	} else {
		// -------------------------------------------------------------
		//            Run detector, compute orientations and descriptors
		// -------------------------------------------------------------
				
		vector< float* > all_keypoints;
		
		for( VL::Sift::KeypointsConstIter iter = sift.keypointsBegin() ;
			iter != sift.keypointsEnd() ; ++iter ) {

				// detect orientations
				VL::float_t angles [4] ;
				int nangles ;
				if( ! noorient ) {
					nangles = sift.computeKeypointOrientations(angles, *iter) ;
				} else {
					nangles = 1;
					angles[0] = VL::float_t(0) ;
				}

				// compute descriptors
				for(int a = 0 ; a < nangles ; ++a) {

					//std::cout << std::setprecision(2) << iter->x << ' '
					//	<< std::setprecision(2) << iter->y << ' '
					//	<< std::setprecision(2) << iter->sigma << ' ' 
					//	<< std::setprecision(3) << angles[a] ;

					/* compute descriptor */
					VL::float_t descr_pt [128] ;
					sift.computeKeypointDescriptor(descr_pt, *iter, angles[a]) ;

					/* save descriptor to appropriate file */	      
					float* kp = new float[132];
					kp[0] = iter->x;
					kp[1] = iter->y;
					kp[2] = iter->sigma;
					kp[3] = angles[a];
					for (int di = 0; di<128; di++) kp[di+4] = descr_pt[di];
					all_keypoints.push_back(kp);
				} // next angle
		} // next keypoint
		
		
		*nkeypoints = all_keypoints.size();
		outKeypoints = new float[(128+4)* (*nkeypoints)];
		for (int i = 0; i< *nkeypoints; i++){
			for (int di = 0; di<132; di++) 	outKeypoints[132*i+di] = all_keypoints[i][di];	
			delete [] all_keypoints[i];
		}
	}
	
	return outKeypoints;
}

} // extern "C"
