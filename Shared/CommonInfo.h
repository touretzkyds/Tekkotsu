//-*-c++-*-
#ifndef INCLUDED_CommonInfo_h_
#define INCLUDED_CommonInfo_h_

#include <vector>
#include <map>
#include <set>
#include <string>
#include <stdexcept>
#include "Shared/fmat.h"

//! parameters for a 60° horizontal field of view, 4:3 aspect ratio camera
namespace CameraGeneric60 {
	const float CameraHorizFOV=60*(float)M_PI/180; //!< horizontal field of view (radians)
	const float CameraVertFOV=0.8172757101952f; //!< vertical field of view (radians): 2*atan( tan(60°/2) * 3/4 ) = 46.83°
	const float CameraFOV=CameraHorizFOV; //!< should be set to maximum of #CameraHorizFOV or #CameraVertFOV
	const unsigned int CameraResolutionX=640; //!< the number of pixels available in the 'full' layer
	const unsigned int CameraResolutionY=480; //!< the number of pixels available in the 'full' layer
	extern const char CameraModelName[]; //!< specifies a name of the camera to load calibration parameters into RobotInfo::CameraHomography
}

//! parameters for a 75° diagonal field of view, 4:3 aspect ratio camera, like Logitech QuickCam Pro 9000 or Communicate Deluxe
namespace Camera75DOF {
	// 2*atan( tan(75/2) * 4/5 ) = 63.09° (horizontal)
	// 2*atan( tan(75/2) * 3/5 ) = 49.44° (vertical)
	const float CameraHorizFOV = 1.1010990963f; //!< horizontal field of view (radians)
	const float CameraVertFOV = 0.8629313824f; //!< vertical field of view (radians)
	const float CameraFOV=CameraHorizFOV; //!< should be set to maximum of #CameraHorizFOV or #CameraVertFOV
  	const unsigned int CameraResolutionX=640; //!< the number of pixels available in the 'full' layer
  	const unsigned int CameraResolutionY=480; //!< the number of pixels available in the 'full' layer
	extern const char CameraModelName[]; //!< specifies a name of the camera to load calibration parameters into RobotInfo::CameraHomography
}

//! parameters for Microsoft Kinect
namespace CameraKinect {
	const float CameraHorizFOV = 1.01229097f; //!< horizontal field of view (radians)
	const float CameraVertFOV = 0.785398163f; //!< vertical field of view (radians)
	const float CameraFOV=CameraHorizFOV; //!< should be set to maximum of #CameraHorizFOV or #CameraVertFOV
	const unsigned int CameraResolutionX=640; //!< the number of pixels available in the 'full' layer
	const unsigned int CameraResolutionY=480; //!< the number of pixels available in the 'full' layer
	extern const char CameraModelName[]; //!< specifies a name of the camera to load calibration parameters into RobotInfo::CameraHomography
}

//! parameters for a 42° by 31.5° camera, like Logitech Communicate STX, note that these specifications indicate either a mistake or non-square pixels...
namespace CameraSTX {
	const float CameraHorizFOV = 0.7330382858f; //!< horizontal field of view (radians)
	const float CameraVertFOV = 0.5497787144f; //!< vertical field of view (radians)
	const float CameraFOV=CameraHorizFOV; //!< should be set to maximum of #CameraHorizFOV or #CameraVertFOV
	const unsigned int CameraResolutionX=640; //!< the number of pixels available in the 'full' layer
	const unsigned int CameraResolutionY=480; //!< the number of pixels available in the 'full' layer
	extern const char CameraModelName[]; //!< specifies a name of the camera to load calibration parameters into RobotInfo::CameraHomography
}

//! parameters for a 66° camera, like the Microsoft LifeCam, with a 16:9 aspect ratio
namespace CameraLifeCam {
	const float CameraHorizFOV = 1.03010024717f; //!< horizontal field of view (radians)
	const float CameraVertFOV = 0.61646432538f; //!< vertical field of view (radians)
	const float CameraFOV=CameraHorizFOV; //!< should be set to maximum of #CameraHorizFOV or #CameraVertFOV
	const unsigned int CameraResolutionX=1280; //!< the number of pixels available in the 'full' layer
	const unsigned int CameraResolutionY=720; //!< the number of pixels available in the 'full' layer
	extern const char CameraModelName[]; //!< specifies a name of the camera to load calibration parameters into RobotInfo::CameraHomography
}
namespace RobotInfo {
	
	//! Defines the indexes to use as indices to access the min and max entries of joint limit specifications (e.g. ERS7Info::outputRanges and ERS7Info::mechanicalLimits)
	enum MinMaxRange_t { MinRange,MaxRange };
	
	//! Some target models, such as ERS2xxInfo, may be dual-booting compatability modes.  This function returns the actual robot name (e.g. ERS210Info::TargetName or ERS220Info::TargetName)
	/*! This function should return the actual RobotInfo::TargetName and not a hard coded string.
	 *  This way, we can rely on testing equality with a direct pointer comparison instead of strcmp().
	 *  (Also eliminates chance of typo or missing refactorization if renamed!).
	 *
	 *  The result of this function is stored in #RobotName, so you don't need to call
	 *  this function -- the only reason it's declared in the header is so you can call
	 *  it during static initialization, when you can't rely on RobotName having been
	 *  initialized yet. */
	const char* const detectModel();
	
	//! Name of the robot which is actually running
	/*! This is usually set to the TargetName, but if the target model is a "compatability" target,
	 *  where the actual host hardware may be a different (more restrictive) configuration,
	 *  then RobotName will be set to the TargetName of that configuration.
	 *
	 *  Note that you should be able to rely on doing pointer comparisons
	 *  between RobotName and various TargetNames to test for different robot models,
	 *  instead of using strcmp() for each. 
	 *
	 *  However, a std::string is used on Aperios to transparently trigger the strcmp because
	 *  of the way the process model is handled there screws up the pointer comparison
	 *  (a different process does the static initialization, so we get a pointer relative to its
	 *  memory space instead of the one we're executing in.  Unix-based platforms don't
	 *  have this problem by using a "real" fork() operation.) */
#ifndef PLATFORM_APERIOS
	extern const char* const RobotName;
#else // have to use a string because aperios is annoying like that
	extern const std::string RobotName;
#endif
	
	//! Allows behaviors to lookup output/button/sensor names from other models to support basic cross-model portability
	/*! Use the getCapabilities() function to look up the Capabalities instance for a given model based on its string robot name */
	class Capabilities {
		friend const Capabilities* getCapabilities(const std::string& robName);
	public:
		//! constructor, pass the robot name this is regarding, and outputs, buttons, and sensor names
		Capabilities(const char* robName, size_t numFrame, const char * const frameNames[], size_t numBut, const char * const butNames[], size_t numSen, const char * const senNames[], size_t pidOff, size_t numPID, size_t ledOff, size_t numLED, size_t numTotalOut);
		//! shallow copy (explicit to satisfy warning)
		Capabilities(const Capabilities& cap)
			: name(cap.name),
			frames(cap.frames), buttons(cap.buttons), sensors(cap.sensors),
			frameToIndex(cap.frameToIndex), buttonToIndex(cap.buttonToIndex), sensorToIndex(cap.sensorToIndex),
			pidJointOffset(cap.pidJointOffset), numPIDJoints(cap.numPIDJoints), ledOffset(cap.ledOffset), numLEDs(cap.numLEDs), numOutputs(cap.numOutputs),
			fakeOutputs() {}
		//! shallow assign (explicit to satisfy warning)
		Capabilities& operator=(const Capabilities& cap) {
			name=cap.name;
			frames=cap.frames; buttons=cap.buttons; sensors=cap.sensors;
			frameToIndex=cap.frameToIndex; buttonToIndex=cap.buttonToIndex; sensorToIndex=cap.sensorToIndex;
			pidJointOffset=cap.pidJointOffset; numPIDJoints=cap.numPIDJoints; ledOffset=cap.ledOffset; numLEDs=cap.numLEDs; numOutputs=cap.numOutputs;
			fakeOutputs=cap.fakeOutputs;
			return *this;
		}
		//! destructor, explicit just to avoid warning when used as base class
		virtual ~Capabilities() {}
		
		//! returns the name of the robot this corresponds to
		inline const char * getRobotName() const { return name; }
		
		//! returns the number of unique outputs (i.e. not counting aliases and non-actuated reference frames)
		inline unsigned int getNumOutputs() const { return numOutputs; }
		//! returns the number of unique reference frames (includes all outputs, plus unactuated points of interest, like cameras or end effector tips
		inline unsigned int getNumFrames() const { return frames.size(); }
		//! returns the number of unique buttons (i.e. not counting aliases)
		inline unsigned int getNumButtons() const { return buttons.size(); }
		//! returns the number of unique sensors (i.e. not counting aliases)
		inline unsigned int getNumSensors() const { return sensors.size(); }
		
		inline unsigned int getPIDJointOffset() const { return pidJointOffset; } //!< returns the offset of the block of 'PID' joints in an output array
		inline unsigned int getNumPIDJoints() const { return numPIDJoints; } //!< returns the number of 'PID' joints
		inline unsigned int getLEDOffset() const { return ledOffset; } //!< returns the offset of the block of LEDs in an output array
		inline unsigned int getNumLEDs() const { return numLEDs; } //!< returns the number of LEDs
		
		//! look up the name corresponding to an offset, returns NULL if not found/available
		inline const char * getOutputName(unsigned int i) const { return i<numOutputs ? frames[i].c_str() : NULL; }
		//! look up the name corresponding to an offset, returns NULL if not found/available
		inline const char * getFrameName(unsigned int i) const { return i<frames.size() ? frames[i].c_str() : NULL; }
		//! look up the name corresponding to an offset, returns NULL if not found/available
		inline const char * getButtonName(unsigned int i) const { return i<buttons.size() ? buttons[i].c_str() : NULL; }
		//! look up the name corresponding to an offset, returns NULL if not found/available
		inline const char * getSensorName(unsigned int i) const { return i<sensors.size() ? sensors[i].c_str() : NULL; }
		
		//! Look up the offset corresponding to a output name, throws std::invalid_argument if not found
		/*! Identical to findOutputOffset(), except throws an exception instead of returning an invalid value.
		 *  Use this if you expect that the output is available, and want a noisy fail-fast if something's wrong (e.g. typo in name?) */
		inline unsigned int getOutputOffset(const std::string& out) const {
			unsigned int i = lookupT("output",frameToIndex,out);
			if(i>=numOutputs) {
				std::string str; str.append(name).append("::capabilities could not find output named ").append(out).append(" (but there is an immobile reference frame by that name)");
				throw std::invalid_argument(str);
			}
			return i;
		}
		//! Look up the offset corresponding to a reference frame name, throws std::invalid_argument if not found
		/*! Identical to findFrameOffset(), except throws an exception instead of returning an invalid value.
		 *  Use this if you expect that the frame is available, and want a noisy fail-fast if something's wrong (e.g. typo in name?) */
		inline unsigned int getFrameOffset(const std::string& frame) const { return lookupT("frame",frameToIndex,frame); }
		//! look up the offset corresponding to a button name, throws std::invalid_argument if not found
		/*! Identical to findButtonOffset(), except throws an exception instead of returning an invalid value.
		 *  Use this if you expect that the button is available, and want a noisy fail-fast if something's wrong (e.g. typo in name?) */
		inline unsigned int getButtonOffset(const std::string& but) const { return lookupT("button",buttonToIndex,but); }
		//! look up the offset corresponding to a sensor name, throws std::invalid_argument if not found
		/*! Identical to findSensorOffset(), except throws an exception instead of returning an invalid value.
		 *  Use this if you expect that the sensor is available, and want a noisy fail-fast if something's wrong (e.g. typo in name?) */
		inline unsigned int getSensorOffset(const std::string& sen) const { return lookupT("sensor",sensorToIndex,sen); }
		
		//! look up the offset corresponding to a output name, returns -1U if not found/available
		/*! Identical to getOutputOffset(), except returns an invalid value instead of throwing an exception.
		 *  Use this if you are testing to see if a capability exists, and don't want to incur exception handling if it isn't (say you're doing a lot of testing) */ 
		inline unsigned int findOutputOffset(const std::string& out) const {
			unsigned int i = lookup(frameToIndex,out);
			return (i>=numOutputs) ? -1U : i;
		}
		//! look up the offset corresponding to a reference frame name, returns -1U if not found/available
		/*! Identical to getFrameOffset(), except returns an invalid value instead of throwing an exception.
		 *  Use this if you are testing to see if a capability exists, and don't want to incur exception handling if it isn't (say you're doing a lot of testing) */ 
		inline unsigned int findFrameOffset(const std::string& frame) const { return lookup(frameToIndex,frame); }
		//! look up the offset corresponding to a button name, returns -1U if not found/available
		/*! Identical to getButtonOffset(), except returns an invalid value instead of throwing an exception.
		 *  Use this if you are testing to see if a capability exists, and don't want to incur exception handling if it isn't (say you're doing a lot of testing) */ 
		inline unsigned int findButtonOffset(const std::string& but) const { return lookup(buttonToIndex,but); }
		//! look up the offset corresponding to a sensor name, returns -1U if not found/available
		/*! Identical to getSensorOffset(), except returns an invalid value instead of throwing an exception.
		 *  Use this if you are testing to see if a capability exists, and don't want to incur exception handling if it isn't (say you're doing a lot of testing) */ 
		inline unsigned int findSensorOffset(const std::string& sen) const { return lookup(sensorToIndex,sen); }
		
		//! returns the offsets of "fake" outputs, see #fakeOutputs
		inline const std::set<unsigned int>& getFakeOutputs() const { return fakeOutputs; }
		
	protected:
		//! helper function, does the work of the get..Offset functions
		inline unsigned int lookupT(const char * errStr, const std::map<std::string,unsigned int>& nameToIndex, const std::string& capname) const {
			std::map<std::string,unsigned int>::const_iterator it=nameToIndex.find(capname);
			if(it==nameToIndex.end()) {
				std::string str; str.append(name).append("::capabilities could not find ").append(errStr).append(" named ").append(capname);
				throw std::invalid_argument(str);
			}
			return it->second;
		}
		//! helper function, does the work of the find..Offset functions
		inline unsigned int lookup(const std::map<std::string,unsigned int>& nameToIndex, const std::string& capname) const {
			std::map<std::string,unsigned int>::const_iterator it=nameToIndex.find(capname);
			return it==nameToIndex.end() ? -1U : it->second;
		}
		
		const char* name; //!< name of robot model
		std::vector<std::string> frames; //!< array of names for reference frames -- this is the "primary" name for each output/frame, #frameToIndex may contain additional aliases
		std::vector<std::string> buttons; //!< array of names for buttons -- this is the "primary" name for each button, #buttonToIndex may contain additional aliases
		std::vector<std::string> sensors; //!< array of names for sensors -- this is the "primary" name for each sensor, #sensorToIndex may contain additional aliases
		std::map<std::string,unsigned int> frameToIndex; //!< maps output names to offset values
		std::map<std::string,unsigned int> buttonToIndex; //!< maps button names to offset values
		std::map<std::string,unsigned int> sensorToIndex; //!< maps sensor names to offset values
		
		size_t pidJointOffset; //!< the offset of the PID joints
		size_t numPIDJoints; //!< the number of PID joints
		size_t ledOffset; //!< the offset of the LEDs
		size_t numLEDs; //!< the number of LEDs
		size_t numOutputs; //!< the total number of outputs (e.g. the reference frames which are actuated)
		
		//! Offsets of "fake" outputs, which don't correspond to any physical device on the robot
		/*! This is used in compatability modes, where some outputs may not be available on the
		 *  host hardware, or for meta-outputs, which control the interpretation of other outputs.
		 *  (such as the A/B LED mode setting for the ERS-7, where a "virtual" LED switches
		 *  the system's intepretation of the face panel LEDs).
		 *
		 *  Most robots can probably just leave this empty -- on Aperios the "fake" outputs are
		 *  skipped when interfacing with the system and their values receive feedback from
		 *  the motion process.  When using the tekkotsu executable under unix-based systems,
		 *  the HAL layer handles this functionality via its own configuration settings, and these
		 *  values are ignored. */
		std::set<unsigned int> fakeOutputs;
		
		//! returns a static map from robot names to capability instances, which are externally allocated
		/*! The Capabilties base class will automatically insert entries into this collection. */
		static std::map<std::string, const Capabilities*>& getCaps();
	};

}

/*! @file
 * @brief Defines items shared between robot models, like camera specifications
 * @author ejt (Creator)
 */

#endif
