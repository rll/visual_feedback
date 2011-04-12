#ifndef _CVG_RANDOM_H_
#define _CVG_RANDOM_H_

#include <cstdlib>
#include <ctime>

namespace CVGreyc {
namespace Random {

class Random {
	public :
	typedef unsigned long random_t;
	
	Random(random_t seed = 0);	
	random_t integer() ;
	random_t integer(random_t imax);
	long int intrange(long int 	a, long int b);
	double uniform() ;
	double uniform(double a);
	double uniform(double a, double b);
	bool binary();
};
	
Random::Random(random_t seed){
	if (seed == 0){
		srandom(time(NULL));
	}
	else {
		srandom(seed);	
	}
}

inline Random::random_t Random::integer() {
	return 	random();
}

inline Random::random_t Random::integer(random_t imax){
	return random()%imax;	
}

inline long int Random::intrange(long int a,long int b){
	long int r = ((int)integer())%(b-a);
	return a+r;
}

inline double Random::uniform() {
	return (float)random()/RAND_MAX;
}

inline double Random::uniform(double a) {
	return a*uniform();	
}

inline double Random::uniform(double a, double b){
	return a+(b-a)*uniform();	
}

inline bool Random::binary() {
	return uniform()<0.5;
}

}		
}



#endif
