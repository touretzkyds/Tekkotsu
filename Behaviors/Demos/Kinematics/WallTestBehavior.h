//-*-c++-*-
#ifndef INCLUDED_WallTestBehavior_h_
#define INCLUDED_WallTestBehavior_h_

#include "Behaviors/BehaviorBase.h"
#include <vector>

namespace NEWMAT {
	class Matrix;
	class ColumnVector;
};

//! measures the relative angle of any walls to the front, left, or right
/*! Making a special cameo appearance in solve1() and solve2(), linear least squares solution using newmat library */
class WallTestBehavior : public BehaviorBase {
public:
	//! constructor
	WallTestBehavior() : BehaviorBase("WallTestBehavior"), d(), a(), usedNear() {}

	virtual void doStart();
	virtual void doStop();
	virtual void doEvent();

	//! Takes a series of measurements, returns slope and intercept of linear least square fit (use QR)
	/*! Uses QR factorization to solve \f$ax+b=y\f$ where we know @e x and @e y, and solve for @e a and @e b */
	void solve1(const std::vector<float>& x, const std::vector<float>& y, float& x0, float& x1);
	//! Takes a series of measurements, returns slope and intercept of linear least square fit (uses SVD)
	/*! Uses SVD factorization to solve \f$ax+by=1\f$ where we know @e x and @e y, and solve for @e a and @e b */
	void solve2(const std::vector<float>& x, const std::vector<float>& y, float& x0, float& x1);
	//! Takes a series of measurements, returns slope and intercept of linear least square fit (uses pseudo-inverse via least_squares.h)
	void solve3(const std::vector<float>& x, const std::vector<float>& y, float& x0, float& x1);

	static std::string getClassDescription() { return "Measures the relative angle of surrounding walls"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	
protected:
	std::vector<float> d; //!< log of distance measurement for each sample
	std::vector<float> a; //!< log of head angle for each sample
	std::vector<bool> usedNear; //!< only used with ERS-7, records whether the sample is from near or far IR sensor

	static const unsigned int reposTime=750; //!< time to stand up
	static const unsigned int panTime=3000;  //!< time for actual panning
	static const unsigned int lagTime=128;   //!< time between when the panning is supposed to get to extremes and when it actually does
};

/*! @file
 * @brief Declares WallTestBehavior, which measures the relative angle of any walls to the front, left, or right
 * @author ejt (Creator)
 */

#endif
