//-*-c++-*-
#ifndef INCLUDED_MoCapEvent_h_
#define INCLUDED_MoCapEvent_h_

#include "EventBase.h"
#include "Shared/fmat.h"
#include <map>

//! Provides notification of new external localization data
/*! Could be feedback from simulation, GPS, or a full motion-capture system.
 *  May not include data for all reference frames, or may only provide one of position or orientation...
 *  You can probably assume at least BaseFrame will be included e.g. getPosition(BaseFrame)/getOrientation(BaseFrame),
 *  otherwise you can also access #positions or #orientations directly for lookup/iteration.
 *
 *  Mirage allows some control over what frames are reported, see MoCapPos and MoCapOri
 *  in EnvConfig.h (assigned via command line or a .mirage world configuration file).
 *  By default only the base frame is reported for both position and orientation. */
class MoCapEvent : public EventBase {
public:
	typedef std::map<unsigned int, fmat::Column<3> >::const_iterator position_iterator;
	typedef std::map<unsigned int, fmat::Quaternion >::const_iterator orientation_iterator;
	
	//! Constructor
	explicit MoCapEvent(const std::string& srcName, size_t sid=0) : EventBase(mocapEGID,sid,statusETID,0,srcName), positions(), orientations() {}
	
	//! Clone implementation
	virtual EventBase* clone() const { return new MoCapEvent(*this); }
	
	//! Attempts to look up a position reading for the specified reference frame, throws std::out_of_range if not found
	const fmat::Column<3>& getPosition(unsigned int frameIdx) const;
	
	//! Attempts to look up an orientation reading for the specified reference frame, throws std::out_of_range if not found
	const fmat::Quaternion& getOrientation(unsigned int frameIdx) const;
	
	//! Constructs a transformation matrix for the specified reference frame, throws std::out_of_range if not found for either position or orientation
	/*! The transformation matrix can be right-multiplied by a point relative to the frame
	 *  to obtain the corresponding world-frame position.  In a perfect simulation, this
	 *  transform should be equivalent to <tt>getPose(BaseFrameOffset) * kine->getT(frameIdx)</tt>
	 *  but in both simulation and the real world, gravity and obstacles may pull
	 *  a joint away from its ideal target pose. */
	const fmat::Transform getPose(unsigned int frameIdx) const {
		return fmat::Transform(getOrientation(frameIdx),getPosition(frameIdx));
	}
	
	std::map<unsigned int, fmat::Column<3> > positions; //!< position data (may not cover all reference frames!)
	std::map<unsigned int, fmat::Quaternion > orientations; //!< orientation data (may not cover all reference frames!)
};

/*! @file
 * @brief Describes MoCapEvent, an event for communicating localization information
 * @author ejt (Creator)
 */

#endif
