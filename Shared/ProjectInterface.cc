#include "ProjectInterface.h"
#include "Motion/EmergencyStopMC.h"
#include "Motion/MotionPtr.h"
#include "Wireless/Socket.h"
#include "Vision/SegmentedColorGenerator.h"
#include "debuget.h"
#include <exception>


namespace ProjectInterface {
	
	// this bit of indirection is simply to avoid dependencies on MotionPtr and EmergencyStopMC for unrelated use of ProjectInterface
	MotionPtr<EmergencyStopMC> estopInstance;
	MotionPtr<EmergencyStopMC>& estop = estopInstance;

	//! default implementation used for #sendCommand (just displays a warning and ignores the call)
	void noSendCommandErr(const std::string& cmd) {
		serr->printf("command '%s' ignored because no ProjectInterface::sendCommand() is installed\n",cmd.c_str());
	}
	void (*sendCommand)(const std::string& cmd)=noSendCommandErr;
	
	bool displayException(const char * file, int line, const char * message, const std::exception* ex) {
		if(file!=NULL) {
			serr->printf("Exception caught at %s:%d => ",debuget::extractFilename(file),line);
		} else {
			serr->printf("Exception => ");
		}
		if(ex!=NULL) {
			serr->printf("'%s'",ex->what());
		} else {
			serr->printf("'%s'","Unknown type");
		}
		if(message!=NULL) {
			serr->printf(" (%s)\n",message);
		} else {
			serr->printf("\n");
		}
#ifndef PLATFORM_APERIOS
		serr->printf("\tWhen running in gdb, try 'catch throw' to break where exceptions are first thrown.\n");
#endif
		return true;
	}
	bool (*uncaughtException)(const char * file, int line, const char * message, const std::exception* ex)=&displayException;

	//! default implementation assigned to lookupColorIndexByName(); checks that #defSegmentedColorGenerator is non-NULL and returns getColorIndex on it
	color_index defLookupColorIndexByName(const std::string& name) {
	  if(defSegmentedColorGenerator==NULL)
	    return -1U;
	  return defSegmentedColorGenerator->getColorIndex(name);
	}
	color_index (*lookupColorIndexByName)(const std::string& name)=&defLookupColorIndexByName;
	
	//! default value initially assigned to lookupColorIndexByRgb(); checks that #defSegmentedColorGenerator is non-NULL and returns getColorIndex on it
	color_index defLookupColorIndexByRgb(const rgb rgbval) {
		if(defSegmentedColorGenerator==NULL)
			return -1U;
		return defSegmentedColorGenerator->getColorIndex(rgbval);
	}
	//! returns color index for color with specified "representitive" RGB color
	color_index (*lookupColorIndexByRgb)(const rgb rgbval)=&defLookupColorIndexByRgb;
	
	//! default implementation assigned to lookupColorRGB(); checks that #defSegmentedColorGenerator is non-NULL and returns getColorRGB on it
	rgb defLookupColorRGB(color_index cindex) {
		if(defSegmentedColorGenerator==NULL)
			return rgb();
		return defSegmentedColorGenerator->getColorRGB(cindex);
	}
	rgb (*lookupColorRGB)(color_index cindex)=&defLookupColorRGB;

        //! default implementation assigned to lookupColorName(); checks that #defSegmentedColorGenerator is non-NULL and returns getColorName on it
        const char* defLookupColorName(color_index cindex) {
	  if(defSegmentedColorGenerator==NULL)
	    return NULL;
	  return defSegmentedColorGenerator->getColorName(cindex);
	}
        const char* (*lookupColorName)(color_index cindex)=&defLookupColorName;

	//! default value initially assigned to lookupNumColors(); checks that #defSegmentedColorGenerator is non-NULL and returns getNumColors on it
	unsigned int defLookupNumColors() {
		if ( defSegmentedColorGenerator == NULL ) 
			return -1U; 
		return defSegmentedColorGenerator->getNumColors();
	}
	//! returns the number of indexed colors which are currently defined
	unsigned int (*lookupNumColors)() = &defLookupNumColors;


	/*** Vision Setup ***/
	FilterBankGenerator * defRawCameraGenerator=NULL;
	FilterBankGenerator * defRawDepthGenerator=NULL;
	FilterBankGenerator * defInterleavedYUVGenerator=NULL;
	JPEGGenerator * defColorJPEGGenerator=NULL;
	JPEGGenerator * defGrayscaleJPEGGenerator=NULL;
	PNGGenerator * defColorPNGGenerator=NULL;
	PNGGenerator * defGrayscalePNGGenerator=NULL;
	SegmentedColorGenerator * defSegmentedColorGenerator=NULL;
	RLEGenerator * defRLEGenerator=NULL;
	RegionGenerator * defRegionGenerator=NULL;
	
	
	/*** Vision SIDs ***/
	unsigned int visRawCameraSID=0;
	unsigned int visRawDepthSID=1;

	unsigned int visInterleaveSID=0;

	unsigned int visColorJPEGSID=0;
	unsigned int visGrayscaleJPEGSID=1;

	unsigned int visColorPNGSID=0;
	unsigned int visGrayscalePNGSID=1;

	unsigned int visSegmentSID=0;

	unsigned int visRLESID=0;

	unsigned int visRegionSID=0;

	unsigned int visPinkBallSID=0;
	unsigned int visBlueBallSID=1;
	unsigned int visGreenBallSID=2;
	unsigned int visYellowBallSID=3;
	unsigned int visOrangeSID=4;
	unsigned int visHandSID=visOrangeSID;
	
	
	/*** Layer Resolutions ***/
	unsigned int doubleLayer=5;
	unsigned int fullLayer=4;
	unsigned int halfLayer=3;
	unsigned int quarterLayer=2;
	unsigned int eighthLayer=1;
	unsigned int sixteenthLayer=0;

}

/*! @file
 * @brief Provides instantiation of the non-required members of ProjectInterface
 * @author ejt (Creator)
 */

