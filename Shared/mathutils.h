//-*-c++-*-
#ifndef __mathutils_h__
#define __mathutils_h__

#include <cmath>
#include <stdexcept>
#include <algorithm>

#ifdef PLATFORM_APERIOS
//missing/broken isnan
namespace std {
	inline bool isnan(float x) {
		const int EXP  = 0x7f800000;
		const int FRAC = 0x007fffff;
		const int y = *((int*)(&x));
		return ((y&EXP)==EXP && (y&FRAC)!=0);
	}
}
#endif

//! a variety of handy mathematical functions, many of which are templated
namespace mathutils {
	//! euclidean distance of two points, see squareDistance()
	inline float distance(float x1, float y1, float x2, float y2) {
		return ::hypotf(x1-x2, y1-y2);
	}
	
	//! euclidean distance of two points, see squareDistance()
	inline double distance(double x1, double y1, double x2, double y2) {
		return ::hypot(x1-x2, y1-y2);
	}
	
	//! Clips @a n to a minimum of @a low or maximum of @a high.
	/*! If @a low and @a high are inverted, high is returned. */
	template <class num>
	inline num limitRange(num n, num low, num high) {
		if (n<=low) return low;
		if (n>=high) return high;
		return n;
	}
	
	//! Returns the log base 2 of a number
	/*! This template implementation does a bit shifting method appropriate for
	 *  integers.  Specializations are provided for float and double to use the 'real' log() */
	template <class num>
	inline num log2t(num x) {
		num ans=0;
		for(unsigned int mag=sizeof(num)*4; mag>0; mag/=2) {
			num y=x>>mag;
			if(y>0) {
				x=y;
				ans+=mag;
			}
		}
		return ans;
	}
	//! returns the log base 2 for a 'float' value
	template <>
	inline float log2t(float x) {
		return std::log(x)/(float)M_LN2;
	}
	//! returns the log base 2 for a 'double' value
	template <>
	inline double log2t(double x) {
		return std::log(x)/M_LN2;
	}
	
	//! converts from degrees to radians
	inline double deg2rad(double x) {
		return x*M_PI/180;
	}
	
	//! converts from radians to degrees
	inline double rad2deg(double x) {
		return x/M_PI*180;
	}
	
	//! converts from degrees to radians
	inline float deg2rad(float x) {
		return x*static_cast<float>(M_PI)/180;
	}
	
	//! converts from radians to degrees
	inline float rad2deg(float x) {
		return x/static_cast<float>(M_PI)*180;
	}
	
	//! Will set a to be between (-pi,pi) (inclusive), just like atan2()
	/*! This is only efficient if we expect that the angle is already close to the range... otherwise use fmod...
	 *  See also Measures.h, this normalize is identical to the AngSignPi::normalize() */
	template<class num>
	num normalizeAngle(num value) {
		const num Pi=static_cast<num>(M_PI);
		const num TwoPi=static_cast<num>(2*M_PI);
		if ( value > Pi ) {
			if ( value <= TwoPi ) {
				value -= TwoPi;
			} else { // use fmod only if necessary
				value = std::fmod(value,TwoPi);
				if( value > Pi )
					value -= TwoPi;
			}
		} else if ( value <= -Pi ) {
			if ( value >= -TwoPi ) {
				value += TwoPi;
			} else { // use fmod only if necessary
				value = std::fmod(value,TwoPi);
				if( value <= -Pi )
					value += TwoPi;
			}
		}
		return value;
	}
	
	// Aperios doesn't provide a random(), only rand()...
	// For testing consistency, Aibo target models always use rand(), regardless of local/aperios platform.
#if defined(TGT_ERS210) || defined(TGT_ERS220) || defined(TGT_ERS2xx) || defined(TGT_ERS7) || defined(PLATFORM_APERIOS)
	
	//! Chooses a number in the range [min,max) (exclusive upper bound)
	inline float sampleRange(float min, float max) {
		return rand() / (float)RAND_MAX * (max-min) + min;
	}
	//! Chooses a number in the range [min,max) (exclusive upper bound)
	inline double sampleRange(double min, double max) {
		return rand() / (double)RAND_MAX * (max-min) + min;
	}
	//! Chooses a number in the range [min,max) (exclusive upper bound)
	template<class num>
	num sampleInteger(num min, num max) {
		return rand() % (max-min) + min;
	}
	
#else
	
	//! Chooses a number in the range [min,max) (exclusive upper bound)
	inline float sampleRange(float min, float max) {
		return random() / (float)(1U<<31) * (max-min) + min;
	}
	//! Chooses a number in the range [min,max) (exclusive upper bound)
	inline double sampleRange(double min, double max) {
		return random() / (double)(1U<<31) * (max-min) + min;
	}
	//! Chooses a number in the range [min,max) (exclusive upper bound)
	template<class num>
	num sampleInteger(num min, num max) {
		return random() % (max-min) + min;
	}
	
#endif
	
	//! Pass a collection of std::pair<value,weight> (map, vector, etc.), returns a value based on a weighted sample
	/*! Throws an exception if the collection is empty; if all weights are zero, performs a uniform sampling. */
	template<class C>
	typename C::value_type::first_type weightedPick(const C& c) {
		typedef typename C::value_type::first_type T;
		typedef typename C::value_type::second_type W;
		typedef std::pair<W,const T*> pair;
		if(c.size()==0)
			throw std::runtime_error("mathutils::weightedPick passed empty collection");
		std::vector<pair> activation;
		activation.reserve(c.size());
		W curAct=0;
		for(typename C::const_iterator it=c.begin(); it!=c.end(); ++it)
			activation.push_back(pair(curAct += it->second, &it->first));
		//ASSERTRETVAL(curAct>0, "mathutils::weightedPick given zero total weight",c.begin()->first);
		if(curAct<=0) {
			// all zero weight, just pick uniformly
			return *activation[sampleInteger<size_t>(0, activation.size())].second;
		}
		pair pick(sampleRange(0, curAct), NULL);
		typename std::vector<pair>::const_iterator it = std::upper_bound(activation.begin(),activation.end(),pick);
		return *it->second;
	}
	
}

#endif
