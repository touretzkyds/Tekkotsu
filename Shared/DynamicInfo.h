//-*-c++-*-
#ifndef INCLUDED_DynamicInfo_h
#define INCLUDED_DynamicInfo_h

#include "Shared/CommonInfo.h"
#include <cmath>

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_DYNAMIC)
#	define TGT_IS_DYNAMIC
#	define TGT_HAS_WEBCAM 1
#endif

//! Declares symbols for the 'Dynamic' robot model, used for tools which are model agnostic
namespace DynamicInfo {

	extern const char* TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=32;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=1;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file, these are the primary grouping
	/*!Right now all binary joints are slow, but perhaps this won't always be the case... hence the IsFast/Slow bitmasks to select which type, in order to be more general */
	//!@name Output Types Information
	extern unsigned NumWheels; //!< no wheels, just legs
	
	extern unsigned JointsPerArm; //!< no arms, just legs
	extern unsigned NumArms; //!< no arms, just legs
	extern unsigned NumArmJoints;
	
	extern unsigned JointsPerLeg; //!< The number of joints per leg
	extern unsigned NumLegs; //!< The number of legs
	extern unsigned NumLegJoints; //!< the TOTAL number of joints on ALL legs
	extern unsigned NumHeadJoints; //!< The number of joints in the neck
	extern unsigned NumTailJoints; //!< The number of joints assigned to the tail
	extern unsigned NumMouthJoints; //!< the number of joints that control the mouth
	extern unsigned NumEarJoints; //!< The number of joints which control the ears (NOT per ear, is total)
	extern unsigned NumButtons; //!< the number of buttons that are available, 2 head, 4 paws, 3 back, 1 underbelly see ERS7Info::ButtonOffset_t
	extern unsigned NumSensors;  //!< 3 IR (distance), 3 accel (force), 5 from power, see ERS7Info::SensorOffset_t
	extern unsigned NumLEDs; //!< The number of LEDs which can be controlled
	extern unsigned NumFacePanelLEDs; //!< The number of face panel LEDs
	
	extern unsigned NumPIDJoints; //!< The number of joints which use PID motion - everything except ears
	extern unsigned NumOutputs; //!< the total number of outputs
	extern unsigned NumReferenceFrames; //!< for the base, paws (NumLegs), camera, and IR sensors (3) reference frames

	using namespace CameraGeneric60;
	
	extern unsigned PIDJointOffset; //!< The beginning of the PID Joints
	extern unsigned LEDOffset; //!< the offset of LEDs in WorldState::outputs and MotionCommand functions, see LedOffset_t for specific offsets
	extern unsigned BaseFrameOffset; //!< Use with kinematics to refer to base reference frame

	//! Name for each button
	extern std::vector<const char *> buttonNames;
		
	//! Provides a string name for each sensor
	extern std::vector<const char *> sensorNames;
	
	//! Names for each of the outputs
	extern std::vector<const char *> outputNames;
	
	
	//! provides polymorphic robot capability detection/mapping
	class DynamicCapabilities : public RobotInfo::Capabilities {
	public:
		//! constructor
		DynamicCapabilities()
		: Capabilities(TargetName,0,NULL,0,NULL,0,NULL,0,0,0,0,0),
		memoization(true)
		{}
		
		DynamicCapabilities& operator=(const Capabilities& cap) {
			Capabilities::operator=(cap);
			TargetName=name;
			PIDJointOffset=pidJointOffset;
			NumPIDJoints=numPIDJoints;
			LEDOffset=ledOffset;
			NumLEDs=numLEDs;
			NumOutputs=numOutputs;
			
			NumReferenceFrames=frames.size();
			NumButtons=buttons.size();
			NumSensors=sensors.size();
			return *this;
		}

		inline unsigned int getOutputOffset(const std::string& out) {
			if(memoization) {
				unsigned pre = NumReferenceFrames;
				unsigned int i = memoLookup(frames,frameToIndex,NumReferenceFrames,out);
				if(pre!=NumReferenceFrames)
					numOutputs=NumOutputs=NumReferenceFrames;
				return i;
			} else {
				return lookupT("output",frameToIndex,out);
			}
		}
		inline unsigned int getFrameOffset(const std::string& frame) {
			if(memoization) { // assume all frames are also outputs since we can't tell
				unsigned pre = NumReferenceFrames;
				unsigned int i = memoLookup(frames,frameToIndex,NumReferenceFrames,frame);
				if(pre!=NumReferenceFrames)
					numOutputs=NumOutputs=NumReferenceFrames;
				return i;
			} else {
				return lookupT("frame",frameToIndex,frame);
			}
		}
		inline unsigned int getButtonOffset(const std::string& but) { return memoization ? memoLookup(buttons,buttonToIndex,NumButtons,but) : lookupT("button",buttonToIndex,but); }
		inline unsigned int getSensorOffset(const std::string& sen) { return memoization ? memoLookup(sensors,sensorToIndex,NumSensors,sen) : lookupT("sensor",sensorToIndex,sen); }
		
		inline unsigned int findOutputOffset(const std::string& out) {
			if(memoization) {
				unsigned pre = NumReferenceFrames;
				unsigned int i = memoLookup(frames,frameToIndex,NumReferenceFrames,out);
				if(pre!=NumReferenceFrames)
					numOutputs=NumOutputs=NumReferenceFrames;
				return i;
			} else {
				return lookup(frameToIndex,out);
			}
		}
		inline unsigned int findFrameOffset(const std::string& frame) {
			if(memoization) { // assume all frames are also outputs since we can't tell
				unsigned pre = NumReferenceFrames;
				unsigned int i = memoLookup(frames,frameToIndex,NumReferenceFrames,frame);
				if(pre!=NumReferenceFrames)
					numOutputs=NumOutputs=NumReferenceFrames;
				return i;
			} else {
				return lookup(frameToIndex,frame);
			}
		}
		inline unsigned int findButtonOffset(const std::string& but) { return memoization ? memoLookup(buttons,buttonToIndex,NumButtons,but) : lookup(buttonToIndex,but); }
		inline unsigned int findSensorOffset(const std::string& sen) { return memoization ? memoLookup(sensors,sensorToIndex,NumSensors,sen) : lookup(sensorToIndex,sen); }
		
		void enableMemoization(bool b) { memoization=b; }
		bool isMemoizing() const { return memoization; }
		
	protected:
		inline unsigned int memoLookup(std::vector<std::string>& idxToName, std::map<std::string,unsigned int>& nameToIndex, unsigned& dim, const std::string& capname) {
			std::map<std::string,unsigned int>::const_iterator it=nameToIndex.find(capname);
			if(it!=nameToIndex.end())
				return it->second;
			idxToName.push_back(capname);
			dim=idxToName.size();
			nameToIndex.insert(std::pair<std::string,unsigned int>(capname,dim-1));
			return dim-1;
		}
		
		bool memoization;
	};
	extern DynamicCapabilities capabilities;
	
	//! provides 2D array of constant width
	template<typename T, size_t N>
	struct DynamicInfoRow {
		T pid[N];
		T operator[](int i) { return pid[i]; }
	};
	
	//! This table holds the default PID values for each joint.  see PIDMC
	extern std::vector<DynamicInfoRow<float,3> > DefaultPIDs;
		
	//!These values are Sony's recommended maximum joint velocities, in rad/sec
	/*! a value <= 0 means infinite speed (e.g. LEDs)
	 *  
	 *  These limits are <b>not</b> enforced by the framework.  They are simply available for you to use as you see fit.
	 *  HeadPointerMC and PostureMC are primary examples of included classes which do respect these values (although they can be overridden)
	 *  
	 *  These values were obtained from the administrators of the Sony OPEN-R BBS */
	extern std::vector<float> MaxOutputSpeed;

	//! This table holds the software limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	extern std::vector<DynamicInfoRow<float,2> > outputRanges;

	//! This table holds the mechanical limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	/*! Same as #outputRanges, don't know actual values because they were never specified by Sony */
	extern std::vector<DynamicInfoRow<float,2> > mechanicalLimits;
}
	
/*! @file
 * @brief Defines RobotInfo namespace for 'dynamic' models, gives some information about the robot's capabilities, such as joint counts, offsets, names and PID values
 * @author ejt (Creator)
 */

#endif
