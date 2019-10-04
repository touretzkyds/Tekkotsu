//-*-c++-*-
#ifndef INCLUDED_SensorInfo_h_
#define INCLUDED_SensorInfo_h_

#include "Shared/plist.h"
#include "Shared/plistSpecialty.h"
#include "Shared/DynamicRobotState.h"

//! Base class for sensor descriptions, actual subclass name stored in #sensorType
struct SensorInfo : virtual public plist::Dictionary {
	//! Subclass should add name/value pairs to the dictionary to allow variable number of values per Sensor, as well as fast filtering and serialization to subscribers.
	/*! A reference to a permanent plist::Primitive<float> instance should be added, this is how values are extracted from the sensor via this shared value. */
	virtual void declareValues(DynamicRobotState& values)=0;
	
	//! Subclass should remove values added via previous declareValues()
	virtual void reclaimValues(DynamicRobotState& values)=0;
	
	//! Name of sensor class, with "Sensor" prefix removed; see KinematicJoint::SensorInfo subclasses
	plist::Primitive<std::string> sensorType;
	
	PLIST_CLONE_ABS(SensorInfo);
	
protected:
	//! Constructor, pass the name of the subclass, minus the "Sensor" prefix — this is used to recreate the subtype via a factory (so you should also do the autoRegister thing and pass that here)
	explicit SensorInfo(const std::string& type) : plist::Dictionary(), sensorType(type) { init(); }
	//! Copy constructor for cloning
	SensorInfo(const SensorInfo& si) : plist::Dictionary(), sensorType(si.sensorType) { init(); }
	//! performs common initialization
	void init() {
		addEntry("SensorType",sensorType,"Name of sensor class, with \"Sensor\" prefix removed; see KinematicJoint::SensorInfo subclasses");
		setLoadSavePolicy(SYNC,SYNC); // should override to FIXED,SYNC from subclasses
	}
};

//! Configures a sensor to take range measurements
/*! Will test for a collision shape between min and max range.
 *  If a hit is found at a distance less than saturationRange, it will be clamped at that value.
 *  If no hit is found within the specified min/max range, the maximum value will be reported. */
struct SensorRangeFinder : public SensorInfo {
	//! constructor
	SensorRangeFinder() : SensorInfo(autoRegister), value(0), sensorName(), minRange(0), saturationRange(0),maxRange(0) { init(); }
	//! copy constructor for cloning
	SensorRangeFinder(const SensorRangeFinder& s)
		: SensorInfo(s), value(s.value), sensorName(s.sensorName), minRange(s.minRange),
		saturationRange(s.saturationRange), maxRange(s.maxRange) { init(); }
	
	virtual void declareValues(DynamicRobotState& values) {
		values.sensors.addEntry(sensorName,value);
	}
	virtual void reclaimValues(DynamicRobotState& values) {
		values.sensors.removeEntry(sensorName);
	}
	plist::Primitive<float> value; //!< current sensor value
	plist::Primitive<std::string> sensorName; //!< Name of the sensor instance, to match up with the a sensorNames[] in RobotInfo header.
	plist::Primitive<float> minRange; //!< Minimum range for hit testing, obstacles closer than this won't be detected (will report max range)
	plist::Primitive<float> saturationRange; //!< Minimum range of sensitivity, obstacles closer than this (but further than min range) will be reported at this value
	plist::Primitive<float> maxRange; //!< Maximum range for hit testing, obstacles further than this won't be detected (and thus will report this value)
	static const std::string autoRegister; //!< Provides registration with FamilyFactory<SensorInfo>
	PLIST_CLONE_DEF(SensorRangeFinder,new SensorRangeFinder(*this));
protected:
	//! performs common initialization
	void init() {
		addEntry("SensorName",sensorName,"Name of the sensor instance, to match up with the a sensorNames[] in RobotInfo header.");
		addEntry("MinRange",minRange,"Minimum range for hit testing, obstacles closer than this won't be detected (will report max range)");
		addEntry("SaturationRange",saturationRange,"Minimum range of sensitivity, obstacles closer than this (but further than min range) will be reported at this value");
		addEntry("MaxRange",maxRange,"Maximum range for hit testing, obstacles further than this won't be detected (and thus will report this value)");
		setLoadSavePolicy(FIXED,SYNC);
	}
};

//! Sets value to 1 if there are any contact constraints within the specified region of the associated body; 0 otherwise
/*! The min/max values can be used to limit which contact positions can toggle the sensor.
 *  Only axes where min≠max are considered for filtering.  Thus all 0's for min and max values indicate no filtering. */
struct SensorContact : public SensorInfo {
	//! constructor
	SensorContact() : SensorInfo(autoRegister), value(0), sensorName(),
	lx(2,0,false), ly(2,0,false), lz(2,0,false), ax(2,0,false), ay(2,0,false), az(2,0,false) { init(); }
	//! copy constructor for cloning
	SensorContact(const SensorContact& s) : SensorInfo(s), value(s.value), sensorName(s.sensorName),
	lx(s.lx), ly(s.ly), lz(s.lz), ax(s.ax), ay(s.ay), az(s.az) { init(); }
	
	virtual void declareValues(DynamicRobotState& values) {
		values.buttons.addEntry(sensorName,value);
	}
	virtual void reclaimValues(DynamicRobotState& values) {
		values.buttons.removeEntry(sensorName);
	}
	bool testPoint(float x, float y, float z);
	plist::Primitive<float> value; //!< current sensor value: 1 if there are any contact constraints within the specified region of the associated body; 0 otherwise
	plist::Primitive<std::string> sensorName; //!< Name of the sensor instance, to match up with the a sensorNames[] in RobotInfo header.
	plist::ArrayOf<plist::Primitive<float> > lx; //!< Minimum and maximum cartesian x
	plist::ArrayOf<plist::Primitive<float> > ly; //!< Minimum and maximum cartesian y
	plist::ArrayOf<plist::Primitive<float> > lz; //!< Minimum and maximum cartesian z
	plist::ArrayOf<plist::Angle> ax; //!< Minimum and maximum angle about joint origin x axis (y axis is 0°)
	plist::ArrayOf<plist::Angle> ay; //!< Minimum and maximum angle about joint origin y axis (z axis is 0°)
	plist::ArrayOf<plist::Angle> az; //!< Minimum and maximum angle about joint origin z axis (x axis is 0°)
	static const std::string autoRegister; //!< Provides registration with FamilyFactory<SensorInfo>
	PLIST_CLONE_DEF(SensorContact,new SensorContact(*this));
protected:
	//! performs common initialization
	void init() {
		addEntry("SensorName",sensorName,"Name of the sensor instance, to match up with the a sensorNames[] in RobotInfo header.");
		addEntry("LinX",lx,"Minimum and maximum cartesian x");
		addEntry("LinY",ly,"Minimum and maximum cartesian y");
		addEntry("LinZ",lz,"Minimum and maximum cartesian z");
		addEntry("AngX",ax,"Minimum and maximum angle about joint origin x axis");
		addEntry("AngY",ay,"Minimum and maximum angle about joint origin y axis");
		addEntry("AngZ",az,"Minimum and maximum angle about joint origin z axis");
		lx.setSaveInlineStyle(true); ly.setSaveInlineStyle(true); lz.setSaveInlineStyle(true);
		ax.setSaveInlineStyle(true); ay.setSaveInlineStyle(true); az.setSaveInlineStyle(true);
		setLoadSavePolicy(FIXED,SYNC);
	}
};

//! Sets value to the current joint position.  This is created automatically by Mirage for named joints where min≠max, so typically you don't need to explicitly request this sensor
struct SensorFeedback : public SensorInfo {
	//! constructor
	SensorFeedback() : SensorInfo(autoRegister), value(0), sensorName() { init(); }
	//! copy constructor for cloning
	SensorFeedback(const SensorFeedback& s) : SensorInfo(s), value(s.value), sensorName(s.sensorName) { init(); }
	
	virtual void declareValues(DynamicRobotState& values) {
		values.outputs.addEntry(sensorName,value);
	}
	virtual void reclaimValues(DynamicRobotState& values) {
		values.outputs.removeEntry(sensorName);
	}
	plist::Primitive<float> value; //!< current sensor value: 1 if there are any contact constraints within the specified region of the associated body; 0 otherwise
	std::string sensorName; //!< will be set by Client during loading to ensure synchronization with KinematicJoint name
	static const std::string autoRegister; //!< Provides registration with FamilyFactory<SensorInfo>
	PLIST_CLONE_DEF(SensorFeedback,new SensorFeedback(*this));
protected:
	//! performs common initialization
	void init() {
		setLoadSavePolicy(FIXED,SYNC);
	}
};

struct GPSSensor : public SensorInfo {
	//! constructor
	GPSSensor() : SensorInfo(autoRegister), sensorName(), curX(0), curY(0), curZ(0), curHeading(0) { init(); }
	//! copy constructor for cloning
	GPSSensor(const GPSSensor& s) : SensorInfo(s), sensorName(s.sensorName), curX(s.curX), curY(s.curY), curZ(s.curZ), curHeading(s.curHeading) { init(); }
	
	virtual void declareValues(DynamicRobotState& values) {
		values.sensors.addEntry(sensorName+"X",curX);
		values.sensors.addEntry(sensorName+"Y",curY);
		values.sensors.addEntry(sensorName+"Z",curZ);
		values.sensors.addEntry(sensorName+"Heading",curHeading);
	}
	virtual void reclaimValues(DynamicRobotState& values) {
		values.sensors.removeEntry(sensorName+"X");
		values.sensors.removeEntry(sensorName+"Y");
		values.sensors.removeEntry(sensorName+"Z");
		values.sensors.removeEntry(sensorName+"Heading");
	}
	plist::Primitive<std::string> sensorName; //!< Name of the sensor instance, to match up with the a sensorNames[] in RobotInfo header.
	plist::Primitive<float> curX; //!< current displacement along the X axis
	plist::Primitive<float> curY; //!< current displacement along the Y axis
	plist::Primitive<float> curZ; //!< current displacement along the Z axis
	plist::Primitive<float> curHeading; //!< current orientation
	static const std::string autoRegister; //!< Provides registration with FamilyFactory<SensorInfo>
	PLIST_CLONE_DEF(GPSSensor,new GPSSensor(*this));
protected:
	//! performs common initialization
	void init() {
		addEntry("SensorName",sensorName,"Name of the sensor instance, to match up with the a sensorNames[] in RobotInfo header.");
		setLoadSavePolicy(FIXED,SYNC);		
	}
};

//! Odometry is presently only used for the Create robot, which reports cumulative distances and angles. 
struct OdometrySensor : public SensorInfo {
	//! constructor
	OdometrySensor()
		: SensorInfo(autoRegister), forwardSensorName(), leftSensorName(), upSensorName(), headingSensorName(),
			leftEncoderSensorName(), rightEncoderSensorName(),
		  lastX(0), lastY(0), lastZ(0), lastHeading(0),
		  deltaX(0), deltaY(0), deltaZ(0), deltaHeading(0), 
		  cumX(0), cumY(0), cumZ(0), cumHeading(0),
			wheelCircumference(0), wheelBase(0), encoderTicksPerRev(0),
			leftEncoder(0), rightEncoder(0)
	{ init(); }

	//! copy constructor for cloning
	OdometrySensor(const OdometrySensor& s)
		: SensorInfo(s), forwardSensorName(s.forwardSensorName), leftSensorName(s.leftSensorName), upSensorName(s.upSensorName), headingSensorName(s.headingSensorName), 
			leftEncoderSensorName(s.leftEncoderSensorName), rightEncoderSensorName(s.rightEncoderSensorName),
		  lastX(s.lastX), lastY(s.lastY), lastZ(s.lastZ), lastHeading(s.lastHeading),
		  deltaX(s.deltaX), deltaY(s.deltaY), deltaZ(s.deltaZ), deltaHeading(s.deltaHeading),
		  cumX(s.cumX), cumY(s.cumY), cumZ(s.cumZ), cumHeading(s.cumHeading),
			wheelCircumference(s.wheelCircumference), wheelBase(s.wheelBase), 
			encoderTicksPerRev(s.encoderTicksPerRev), leftEncoder(s.leftEncoder), rightEncoder(s.rightEncoder)
	{ init(); }
	
	virtual void declareValues(DynamicRobotState& values) {
		if (forwardSensorName.size() > 0)
			values.sensors.addEntry(forwardSensorName,cumX);
		if (leftSensorName.size() > 0)
			values.sensors.addEntry(leftSensorName,cumY);
		if (upSensorName.size() > 0)
			values.sensors.addEntry(upSensorName,cumZ);
		if (headingSensorName.size() > 0)
			values.sensors.addEntry(headingSensorName,cumHeading);
		if (leftEncoderSensorName.size() > 0)
			values.sensors.addEntry(leftEncoderSensorName,leftEncoder);
		if (rightEncoderSensorName.size() > 0)
			values.sensors.addEntry(rightEncoderSensorName,rightEncoder);
	}

	virtual void reclaimValues(DynamicRobotState& values) {
		if (forwardSensorName.size() > 0)
			values.sensors.removeEntry(forwardSensorName);
		if (leftSensorName.size() > 0)
			values.sensors.removeEntry(leftSensorName);
		if (upSensorName.size() > 0)
			values.sensors.removeEntry(upSensorName);
		if (headingSensorName.size() > 0)
			values.sensors.removeEntry(headingSensorName);
		if (leftEncoderSensorName.size() > 0)
			values.sensors.removeEntry(leftEncoderSensorName);
		if (rightEncoderSensorName.size() > 0)
			values.sensors.removeEntry(rightEncoderSensorName);
	}

	plist::Primitive<std::string> forwardSensorName; //!< Name of the sensor instance, to match up with the sensorNames[] in RobotInfo header.
	plist::Primitive<std::string> leftSensorName; //!< Name of the sensor instance, to match up with the sensorNames[] in RobotInfo header.
	plist::Primitive<std::string> upSensorName; //!< Name of the sensor instance, to match up with the sensorNames[] in RobotInfo header.
	plist::Primitive<std::string> headingSensorName; //!< Name of the sensor instance, to match up with the sensorNames[] in RobotInfo header.
	plist::Primitive<std::string> leftEncoderSensorName; //!< Name of the left encoder instance, to match up with the SensorNames[] in RobotInfo header.
	plist::Primitive<std::string> rightEncoderSensorName; //!< Name of the right encoder instance, to match up with the SensorNames[] in RobotInfo header.
	float lastX; //!< previous displacement along the X axis
	float lastY; //!< previous displacement along the Y axis
	float lastZ; //!< previous displacement along the Z axis
	float lastHeading; //!< previous heading
	float deltaX; //!< current displacement along the X axis
	float deltaY; //!< current displacement along the Y axis
	float deltaZ; //!< current displacement along the Z axis
	float deltaHeading; //!< current heading change
	plist::Primitive<float> cumX; //!< cumulative displacement along the X axis
	plist::Primitive<float> cumY; //!< cumulative displacement along the Y axis
	plist::Primitive<float> cumZ; //!< cumulative displacement along the Z axis
	plist::Primitive<float> cumHeading; //!< cumulative heading change
	plist::Primitive<float> wheelCircumference; //!< wheel circumference in mm (72.0 for Create2)
	plist::Primitive<float> wheelBase; //!< wheel base in mm (235.0 for Create2)
	plist::Primitive<float> encoderTicksPerRev; //!< number of encoder ticks per revolution (508.8 for Create2)
	plist::Primitive<float> leftEncoder; //!< left encoder value for Create-like robots
	plist::Primitive<float> rightEncoder; //!< rightencoder value for Create-like robots
	static const std::string autoRegister; //!< Provides registration with FamilyFactory<SensorInfo>
	PLIST_CLONE_DEF(OdometrySensor,new OdometrySensor(*this));
protected:
	//! performs common initialization
	void init() {
		addEntry("ForwardSensorName",forwardSensorName,"Name of the sensor along X axis");
		addEntry("LeftSensorName",leftSensorName,"Name of the sensor along Y axis");
		addEntry("UpSensorName",upSensorName,"Name of the sensor along Z axis");
		addEntry("HeadingSensorName",headingSensorName,"Name of the sensor for orientation");		
		addEntry("WheelCircumference",wheelCircumference,"Wheel circumference in mm");
		addEntry("WheelBase",wheelBase,"Wheel base in mm");
		addEntry("EncoderTicksPerRev",encoderTicksPerRev,"Number of encoder ticks per revolution");
		addEntry("LeftEncoderSensorName",leftEncoderSensorName,"Name of the left wheel encoder");
		addEntry("RightEncoderSensorName",rightEncoderSensorName,"Name of the right wheel encoder");
		setLoadSavePolicy(FIXED,SYNC);		
	}
};

namespace plist {
	//! This specialization looks for the SensorInfo::sensorType, then has the factory construct the correct subtype before loading the @a node into and returning that.
	template<> SensorInfo* loadXML(xmlNode* node);
	//! SensorInfo::sensorType.set() could still lead to trouble, although with a little work we could probably find a way to support it...
	template<> inline SensorInfo* allocate() { throw std::runtime_error("cannot plist::allocate generic instances (SensorInfo)"); }
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
