//-*-c++-*-
#ifndef INCLUDED_ProjectInterface_h_
#define INCLUDED_ProjectInterface_h_

#include "Vision/colors.h"
#include <string>

template<class T> class MotionPtr;
class EmergencyStopMC;

class BehaviorBase;
class FilterBankGenerator;
class SegmentedColorGenerator;
class RLEGenerator;
class RegionGenerator;
class JPEGGenerator;
class PNGGenerator;
namespace std {
	class exception;
}

//! A collection of the global variables which should be set by a project to use the Tekkotsu framework
/*! This namespace hold a few variables needed for initialization of the
 *  framework, but mainly declares variables which are used by demo
 *  behaviors.  Although the variables are declared here, and
 *  some default values provided, it is up to your project to define
 *  the actual values used for your own project.  This provides a way
 *  to reassign conflicts between values provided by the framework vs.
 *  those you might wish to add to your project.
 *  
 *  Currently, all required members are references (so they can't be
 *  set to NULL and you'll get errors if you leave them out) and all
 *  optional settings are pointers so you can ignore them if you want.
 *  
 *  The "optional" variables are used by demo behaviors, and thus
 *  if you remove all of the demo behaviors, you won't need to define
 *  the corresponding interface values here.
 *
 *  If you want to add new ID values for your project, create a new
 *  'globals.h' or some such in your project -- you don't need to
 *  add them here since this file is shared by all projects which
 *  use the framework, you shouldn't need to modify it for each
 *  project.
 */
namespace ProjectInterface {
	
	//! REQUIRED: you must define a behavior which will be started when the boot is complete
	/*! This is similar in idea to the Linux init process - it should do
	 *  some basic initialization and then launch any other behavior you
	 *  would like to run at boot.
	 *  To avoid static initialization ordering issues, this is a function
	 *  which will be called after environment setup is complete, which
	 *  should then return a behavior to use as the startup behavior.
	 *  This behavior should not be reference counted, and probably makes
	 *  most sense to implement as a static local variable of the function.
	 *  (Each call should return the same behavior) */
	BehaviorBase& startupBehavior();
	
	//! The active emergency stop motion command, used by various behaviors and controls to test whether they are in control
	extern MotionPtr<EmergencyStopMC>& estop;
	
	//! sends a command to the project, allows GUI elements of the framework to send commands to the hardware abstraction layer
	/*! Generally commands are assumed to be for the Tekkostu HAL command line,
	 *  and otherwise this will be left NULL. */
	extern void (*sendCommand)(const std::string& cmd);
	
	//! The exception handler for exceptions which have fallen through to base Tekkotsu functions
	/*! You can override this to install your own handler by assigning a
	 *  new function.  This defaults to displayException(), which
	 *  <b>does not</b> call abort() (which would otherwise be the
	 *  default if the exception fell through).
	 *  @param file The file where the exception was caught (usually just pass __FILE__), or NULL
	 *  @param line The line number where the exception was caught (usually just pass __LINE__), if @a file is NULL, @a line is ignored
	 *  @param message An addition message, or NULL
	 *  @param ex The exception which was caught, or NULL if it is was not a std::exception subclass
	 *  @return true if the exception was handled, false if the exception should be rethrown */
	extern bool (*uncaughtException)(const char * file, int line, const char * message, const std::exception* ex);

	//! Displays information about an exception on #serr, provides a default value for #uncaughtException
	/*! You can call this directly from your own code any time you would like an exception error message.
	 *  @param file The file where the exception was caught (usually just pass __FILE__), or NULL
	 *  @param line The line number where the exception was caught (usually just pass __LINE__), if @a file is NULL, @a line is ignored
	 *  @param message An addition message, or NULL
	 *  @param ex The exception which was caught, or NULL if it is was not a std::exception subclass
	 *  @return true, indicating the exception was handled adequately */
	bool displayException(const char * file, int line, const char * message, const std::exception* ex);
	
	//! allows you to override how colors are defined -- by default, this will be set to a function which passes the call to defSegmentedColorGenerator
	/*! As per SegmentedColorGenerator::getColorIndex(), if @a name is not valid, return -1U */
	extern color_index (*lookupColorIndexByName)(const std::string& name);
	//! allows you to override how colors are defined -- by default, this will be set to a function which passes the call to defSegmentedColorGenerator
	extern color_index (*lookupColorIndexByRgb)(const rgb rgbval);
	//! allows you to override how colors are defined -- by default, this will be set to a function which passes the call to defSegmentedColorGenerator
	/*! As per SegmentedColorGenerator::getColorRGB(), if @a index is not valid, return black (rgb()) */
	extern rgb (*lookupColorRGB)(color_index cindex);
	//! allows you to override how colors are defined -- by default, this will be set to a function which passes the call to defSegmentedColorGenerator
	/*! As per SegmentedColorGenerator::getColorRGB(), if @a index is not valid, return color 0 */
	extern const char* (*lookupColorName)(color_index cindex);
	//! Returns the index corresponding to a color of specified name by calling lookupColorIndexByName()
	/*! As per SegmentedColorGenerator::getColorIndex(), if @a name is not valid, return -1U */
	inline color_index getColorIndex(const std::string& name) { if(lookupColorIndexByName==NULL) return -1U; return (*lookupColorIndexByName)(name); }
	//! Returns the index corresponding to an rgb value  by calling lookupColorIndexByRgb()
	inline color_index getColorIndex(const rgb rgbval) { if(lookupColorIndexByRgb==NULL) return -1U; return (*lookupColorIndexByRgb)(rgbval); }
	//! Returns rgb value corresponding to a color of specified name by calling lookupColorRGB(lookupColorIndexByName())
	/*! As per SegmentedColorGenerator::getColorRGB(), if @a name is not valid, return black (rgb()) */
	inline rgb getColorRGB(const std::string& name)  { if(lookupColorIndexByName==NULL || lookupColorRGB==NULL) return rgb(); return (*lookupColorRGB)((*lookupColorIndexByName)(name)); }
	//! Returns rgb value corresponding to a color of specified name by calling lookupColorRGB()
	/*! As per SegmentedColorGenerator::getColorRGB(), if @a index is not valid, return black (rgb()) */
	inline rgb getColorRGB(color_index cindex)  { if(lookupColorRGB==NULL) return rgb(); return (*lookupColorRGB)(cindex); }

	//! Returns color name corresponding to specified color index by calling lookupColorName()
	/*! As per SegmentedColorGenerator::getColorName(), if @a index is not valid, return NULL */
	inline const char* getColorName(color_index cindex) { if(lookupColorName==NULL) return NULL; return (*lookupColorName)(cindex); }
	
	//! Returns color name corresponding to specified rgb value by calling lookupColorName()
	/*! As per SegmentedColorGenerator::getColorName(), if @a index is not valid, return NULL */
	inline const char* getColorName(const rgb rgbval) { color_index c = getColorIndex(rgbval); return getColorName(c==-1U ? 0 : c); }

	extern unsigned int (*lookupNumColors)();
	//! Returns the number of colors, obtained from defSegmentedColorGenerator
	inline unsigned int getNumColors() { if (lookupNumColors == NULL) return -1U; return (*lookupNumColors)(); }

	//! A collection of the various stages of vision processing.  None of these are absolutely required, but are needed to run included demo behaviors and TekkotsuMon modules
	/*! @name Vision Setup */
	//! pointer to generator
	extern FilterBankGenerator * defRawCameraGenerator;
	extern FilterBankGenerator * defRawDepthGenerator;
	extern FilterBankGenerator * defInterleavedYUVGenerator;
	extern JPEGGenerator * defColorJPEGGenerator;
	extern JPEGGenerator * defGrayscaleJPEGGenerator;
	extern PNGGenerator * defColorPNGGenerator;
	extern PNGGenerator * defGrayscalePNGGenerator;
	extern SegmentedColorGenerator * defSegmentedColorGenerator;
	extern RLEGenerator * defRLEGenerator;
	extern RegionGenerator * defRegionGenerator;
	//@}

	//! Default source IDs for the various generators; These are given default values, but you can reassign them if you like.
	/*! @name Vision SIDs */
	//! source id for vision events from the corresponding pipeline stage or object detector
	extern unsigned int visRawCameraSID;
	extern unsigned int visRawDepthSID;
	extern unsigned int visInterleaveSID;
	extern unsigned int visColorJPEGSID;
	extern unsigned int visGrayscaleJPEGSID;
	extern unsigned int visColorPNGSID;
	extern unsigned int visGrayscalePNGSID;
	extern unsigned int visSegmentSID;
	extern unsigned int visRLESID;
	extern unsigned int visRegionSID;
	extern unsigned int visPinkBallSID;
	extern unsigned int visBlueBallSID;
	extern unsigned int visGreenBallSID;
	extern unsigned int visYellowBallSID;
	extern unsigned int visOrangeSID;
	extern unsigned int visHandSID; //!< synonym for #visOrangeSID
	//@}

	//! Allows you to request a particular layer abstractly - this isn't used by the framework, just a suggestion for clarity
	/*! @name Layer Resolutions */
	extern unsigned int doubleLayer;   //!< ERS-2xx: 352*288; ERS-7 416*320 (requires non-trivial computation)
	extern unsigned int fullLayer;     //!< ERS-2xx: 176*144; ERS-7 208*160
	extern unsigned int halfLayer;     //!< ERS-2xx: 88*72; ERS-7 104*80
	extern unsigned int quarterLayer;  //!< ERS-2xx: 44*36; ERS-7 52*40
	extern unsigned int eighthLayer;   //!< ERS-2xx: 22*18; ERS-7 26*20 (simply a bigger interleave referencing quarterLayer)
	extern unsigned int sixteenthLayer;//!< ERS-2xx: 11*9; ERS-7 13*10 (simply a bigger interleave referencing quarterLayer)
	//@}
}

/*! @file
 * @brief Defines ProjectInterface namespace - a collection of the global variables which should be set by a project to use the Tekkotsu framework
 * @author ejt (Creator)
 */

#endif
