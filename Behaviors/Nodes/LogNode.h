//-*-c++-*-
#ifndef INCLUDED_LogNode_h_
#define INCLUDED_LogNode_h_

#include "Behaviors/StateNode.h"
#include "Shared/plist.h"
#include <fstream>

//! On activation, start logging data to disk as a series of image and/or sensor files
class LogNode : public virtual plist::Dictionary, public StateNode {

	// **************************** //
	// ******* CONSTRUCTORS ******* //
	// **************************** //
public:
	//! default constructor, use type name as instance name
	LogNode()
		: plist::Dictionary(), StateNode(),
		basepath(""), basename("log"), incrementNameIfExists(true), 
		logSensors(true), logImages(true), initial(true), compression("png"), 
		indexFile(), path(), startTime(), fileCount(), imageStartFrame(), sensorStartFrame()
	{}

	//! constructor, take an instance name
	LogNode(const std::string& nm)
		: plist::Dictionary(), StateNode(nm),
		basepath(""), basename("log"), incrementNameIfExists(true), 
		logSensors(true), logImages(true), initial(true), compression("png"), 
		indexFile(), path(), startTime(), fileCount(), imageStartFrame(), sensorStartFrame()
	{}
	
	
	// **************************** //
	// ********* MEMBERS ********** //
	// **************************** //
public:
	plist::Primitive<std::string> basepath;
	plist::Primitive<std::string> basename;
	plist::Primitive<bool> incrementNameIfExists;
	plist::Primitive<bool> logSensors;
	plist::Primitive<bool> logImages;
	plist::Primitive<bool> initial;
	plist::Primitive<std::string> compression;
	
protected:
	std::ofstream indexFile;
	std::string path;
	unsigned int startTime;
	unsigned int fileCount;
	unsigned int imageStartFrame;
	unsigned int sensorStartFrame;
	
	
	// **************************** //
	// ********* METHODS ********** //
	// **************************** //
public:
	virtual void postStart();
	virtual void doEvent();
	virtual void stop();
	
	virtual void writeImage(unsigned int time, class FilterBankGenerator& fbk, bool isPNG);
	virtual void writeSensor(unsigned int time);

	static std::string getClassDescription() { return "On activation, start logging data to disk as a series of image and/or sensor files"; }
	virtual std::string getDescription() const { return getClassDescription(); }


	// **************************** //
	// ********** OTHER *********** //
	// **************************** //
private:
	// Providing declarations for these functions will avoid a compiler warning if
	// you have any class members which are pointers.  However, as it is, an error
	// will result if you inadvertantly cause a call to either (which is probably
	// a good thing, unless you really intended to copy/assign a behavior, in
	// which case simply provide implementations for the functions)
	LogNode(const LogNode&); //!< don't call (copy constructor)
	LogNode& operator=(const LogNode&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines LogNode, which logs data to disk as a series of image and/or sensor files
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
