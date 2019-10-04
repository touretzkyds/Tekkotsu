//-*-c++-*-
#ifndef INCLUDED_BallDetectionGenerator_h_
#define INCLUDED_BallDetectionGenerator_h_

#include "Events/EventGeneratorBase.h"
#include <cmath>

class FilterBankEvent;
class RegionGenerator;

//! Uses segmented color region information to detect round objects
/*! This expects its events to come from a RegionGenerator (or a
 *  compatable subclass)
 *  
 *  Sends a VisionObjectEvent only for the largest ball found (if one
 *  @e is found)
 *
 *  You can set the index of the color of the ball to look for in the
 *  constructor, so you can have several of these running looking for
 *  balls of different colors.
 *
 *  This is one of our oldest code segments, and has been hacked on a
 *  lot, so apologies for a bit of a mess...
 */
class BallDetectionGenerator : public EventGeneratorBase {
public:
	//! constructor
	BallDetectionGenerator(unsigned int mysid, const RegionGenerator * rg, unsigned int colorIdx, unsigned int threshmapChan, unsigned int noiseFiltering, float confidence);

	static std::string getClassDescription() { return "Detects round-ish regions"; }

	//! see class notes above for what data this can handle
	virtual void doEvent();

protected:
	typedef unsigned char uchar; //!< shorthand

	//!@name Edge masks
	static const uchar OFF_EDGE_LEFT   = 1<<1; //!< bitmask for calcEdgeMask results
	static const uchar OFF_EDGE_RIGHT  = 1<<2;
	static const uchar OFF_EDGE_TOP    = 1<<3;
	static const uchar OFF_EDGE_BOTTOM = 1<<4;
	//@}

	static const unsigned int NUM_CHECK = 10; //!< the number of regions to check (from largest to smallest)

	//! High level vision ouput structure for detected objects
	struct VObject {
		double confidence; //!< [0,1] Estimate of certainty
		//vector3d loc;      //!< Relative to front of robot (on ground)
		//double left,right; //!< Angle to left and right of object (egocentric)
		//double distance;   //!< Distance of object (on ground)
		//uchar edge;        //!< Is object on edge of image (bitmasks above)
	};

	//! decides wether to actually send the event based on confidence threshold.
	void testSendEvent(const FilterBankEvent& ev, float conf, int regX1,int regX2,int regY1, int regY2, int area);
	//! does the actual event sending
	void createEvent(EventBase::EventTypeID_t etid, float bbX1,float bbX2,float bbY1,float bbY2,float area,float rx,float ry ,unsigned int frame) const;
	//! returns a bit mask corresponding to edges touched by the coordinates passed
	static int calcEdgeMask(int x1,int x2,int y1,int y2, int width, int height);
	//! returns @f[ \left|\frac{a-b}{a+b}\right| @f]
	inline static float pct_from_mean(float a,float b) {
		float s = (a - b) / (a + b);
		return std::abs(s);
	}

	unsigned int clrIdx;  //!< the index of the color of the ball we're looking for
	unsigned int tmIdx;   //!< the index of the theshold map (channel) of the FilterBankEvent
	VObject ball;         //!< information about the best ball found
	bool present;         //!< if true, we think we have a ball in front of us
	unsigned int count;   //!< for each frame where we don't agree with present's value, this is incremented and compared against noiseFilter.
	unsigned int noiseThreshold; //!< the number of frames to wait to make sure an object has dissappeared/reappeared	
	float confidenceThreshold; //!< how sure we should be it's a ball before declaring it as such.
	
private:
	BallDetectionGenerator(const BallDetectionGenerator& fbk); //!< don't call
	const BallDetectionGenerator& operator=(const BallDetectionGenerator& fbk); //!< don't call
};

/*! @file 
 * @brief Describes BallDetectionGenerator, which uses segmented color region information to detect round objects
 * @author alokl (Creator)
 * @author ejt (reorganized)
 * @author Ignacio Herrero Reder < nhr at dte uma es > (VisionObjectInfo Boundary Box - bug 74)
 *
 * History is old, may have grown from CMPack (CMU Robosoccer) roots?
 * I think if there's any of their code left, it's probably *mostly*
 * the commented out stuff I (ejt) left for posterity when
 * reorganizing.  But alokl didn't flag this as CMPack's prior to
 * inital release, and they didn't request credit for it when they
 * reviewed the code, so I guess it's all ours...
 */

#endif
