#include <cmath>

#include "SensorInfo.h" 
#include "Shared/FamilyFactory.h"

using namespace std; 

PLIST_CLONE_IMP(SensorRangeFinder,new SensorRangeFinder(*this));
PLIST_CLONE_IMP(SensorContact,new SensorContact(*this));
PLIST_CLONE_IMP(SensorFeedback,new SensorFeedback(*this));
PLIST_CLONE_IMP(GPSSensor,new GPSSensor(*this));
PLIST_CLONE_IMP(OdometrySensor,new OdometrySensor(*this));



const std::string SensorRangeFinder::autoRegister = FamilyFactory<SensorInfo>::getRegistry().registerType<SensorRangeFinder>("RangeFinder");
const std::string SensorContact::autoRegister = FamilyFactory<SensorInfo>::getRegistry().registerType<SensorContact>("Contact");
const std::string SensorFeedback::autoRegister = FamilyFactory<SensorInfo>::getRegistry().registerType<SensorFeedback>("Feedback");
const std::string GPSSensor::autoRegister = FamilyFactory<SensorInfo>::getRegistry().registerType<GPSSensor>("GPS");
const std::string OdometrySensor::autoRegister = FamilyFactory<SensorInfo>::getRegistry().registerType<OdometrySensor>("Odometry");



bool SensorContact::testPoint(float x, float y, float z) {
	if(lx[0]!=lx[1]) {
		if(x<lx[0] || lx[1]<x)
			return false;
	}
	if(ly[0]!=ly[1]) {
		if(y<ly[0] || ly[1]<y)
			return false;
	}
	if(lz[0]!=lz[1]) {
		if(z<lz[0] || lz[1]<z)
			return false;
	}
	if(ax[0]!=ax[1]) {
		float a = std::atan2(z, y);
		if(a<ax[0] || ax[1]<a)
			return false;
	}
	if(ay[0]!=ay[1]) {
		float a = std::atan2(x, z);
		if(a<ay[0] || ay[1]<a)
			return false;
	}
	if(az[0]!=az[1]) {
		float a = std::atan2(y, x);
		if(a<az[0] || az[1]<a)
			return false;
	}
	return true;
}

namespace plist {
	template<> SensorInfo* loadXML(xmlNode* node) {
		plist::Dictionary d;
		plist::Primitive<std::string> t;
		d.addEntry("SensorType",t);
		d.setUnusedWarning(false);
		d.setLoadPolicy(plist::Collection::FIXED);
		d.loadXML(node);
		if(t.size()==0)
			throw XMLLoadSave::bad_format(node,"KinematicJoint::SensorInfo entry missing SensorType, cannot instantiate");
		SensorInfo* si = FamilyFactory<SensorInfo>::getRegistry().create(t);
		if(si==NULL)
			throw XMLLoadSave::bad_format(node,"KinematicJoint::SensorInfo unknown SensorType '"+t+"', cannot instantiate");
		si->loadXML(node);
		return si;
	}
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
