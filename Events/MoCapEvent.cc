#include "MoCapEvent.h"
#include "Shared/RobotInfo.h"
#include <stdexcept>

const fmat::Column<3>& MoCapEvent::getPosition(unsigned int frameIdx) const {
	std::map<unsigned int, fmat::Column<3> >::const_iterator it=positions.find(frameIdx);
	if(it==positions.end())
		throw std::out_of_range(std::string("MoCapEvent::getPosition() failed for ")+outputNames[frameIdx]);
	return it->second;
}

const fmat::Quaternion& MoCapEvent::getOrientation(unsigned int frameIdx) const {
	std::map<unsigned int, fmat::Quaternion >::const_iterator it=orientations.find(frameIdx);
	if(it==orientations.end())
		throw std::out_of_range(std::string("MoCapEvent::getOrientation() failed for ")+outputNames[frameIdx]);
	return it->second;
}

/*! @file
 * @brief Implements MoCapEvent, an event for communicating localization information
 * @author ejt (Creator)
 */
