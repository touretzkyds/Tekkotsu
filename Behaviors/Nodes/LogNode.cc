#include "LogNode.h"
#include "Shared/get_time.h"
#include "Shared/string_util.h"
#include "Shared/ProjectInterface.h"
#include "Shared/WorldState.h"
#include "Shared/MarkScope.h"
#include "Events/EventRouter.h"
#include "Events/FilterBankEvent.h"
#include "Motion/PostureEngine.h"
#include "Vision/JPEGGenerator.h"
#include "Vision/PNGGenerator.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <sstream>
#include <iomanip>

#ifndef PLATFORM_APERIOS
REGISTER_CONTROL_INSTANCE(LogToDisk,new BehaviorSwitchControl<LogNode>("Log to Disk",false),"Status Reports");
#endif

using namespace std; 

void LogNode::postStart() {
	StateNode::postStart();
	if(!logSensors && !logImages)
		return;
	
	bool ispng=true;
	
	// add event listeners
	string stdcomp = string_util::makeUpper(compression);
	if(logImages && stdcomp.size()==0) {
		cerr << getName() << ": compression format not specified, defaulting to PNG" << endl;
		stdcomp = "PNG";
	}
	if(logImages) {
		if(stdcomp!="JPEG" || stdcomp!="JPG") {
			ispng=false;
			erouter->addListener(this,EventBase::visJPEGEGID,ProjectInterface::visColorJPEGSID,EventBase::activateETID);
		} else if(stdcomp!="PNG") {
			ispng=true;
			erouter->addListener(this,EventBase::visPNGEGID,ProjectInterface::visColorPNGSID,EventBase::activateETID);
		} else {
			cerr << getName() << ": Unknown compression format " << compression << " (png or jpeg please)" << endl;
			if(!logSensors)
				return;
		}
	}
	if(logSensors) {
		erouter->addListener(this,EventBase::sensorEGID);
	}
	
	// set up file system path
	path=string_util::tildeExpansion(basepath);
	if(!incrementNameIfExists) {
		if(path.size()!=0)
			path+='/';
		path+=basename;
	} else {
		if(path.size()!=0)
			path+='/';
		string name=basename+"000";
		struct stat sb;
		for(unsigned int i=0; stat((path+name).c_str(),&sb)==0; ++i) {
			stringstream nameass;
			nameass << basename << std::setw(3) << std::setfill('0') << i;
			name=nameass.str();
		}
		path+=name;
	}
	struct stat sb;
	if(stat(path.c_str(),&sb)==0) {
		if(!(sb.st_mode&S_IFDIR)) {
			cerr << getName() << ": could not create log directory " << path << " (a file by that name already exists)" << endl;
			erouter->removeListener(this);
			return;
		}
	} else {
		cout << "Creating " << path << " for log data" << endl;
		if(mkdir(path.c_str(),0755)!=0) {
			int err=errno;
			cerr << getName() << ": could not create log directory " << path << " (" << strerror(err) << ")" << endl;
			erouter->removeListener(this);
			return;
		}
	}
	
	// create index file
	indexFile.open((path+'/'+basename+".txt").c_str(), ios_base::out | ios_base::trunc);
	if(!indexFile) {
		cerr << getName() << ": could not create index file " << (path+'/'+basename+".txt") << endl;
		erouter->removeListener(this);
		return;
	}
	fileCount = 0;
	startTime = get_time();
	sensorStartFrame = imageStartFrame = -1U; // flag to cause value to be reassigned to next event
	indexFile << "First frame " << ProjectInterface::defRawCameraGenerator->getFrameNumber() << " timestamp: " << startTime << std::endl;
	if(initial) {
		if(logImages) {
			FilterBankGenerator* fbk = NULL;
			if(ispng)
				fbk = ProjectInterface::defColorPNGGenerator;
			else
				fbk = ProjectInterface::defColorJPEGGenerator;
			writeImage(startTime, *fbk, ispng);
		}
		if(logSensors) {
			writeSensor(startTime);
		}
	}
}

void LogNode::doEvent() {
	switch(event->getGeneratorID()) {
		
		case EventBase::visJPEGEGID:{
			const FilterBankEvent& fbk = dynamic_cast<const FilterBankEvent&>(*event);
			writeImage(event->getTimeStamp(),*fbk.getSource(),false);
		} break;
		case EventBase::visPNGEGID: {
			const FilterBankEvent& fbk = dynamic_cast<const FilterBankEvent&>(*event);
			writeImage(event->getTimeStamp(),*fbk.getSource(),true);
		} break;
			
		case EventBase::sensorEGID: {
			writeSensor(event->getTimeStamp());
		} break;
			
		default:
			std::cerr << getName() << ": unhandled event: " << event->getDescription() << std::endl;
	}
}

//! Just like a behavior, called when it's time to stop doing your thing
void LogNode::stop() {
	indexFile.close();
	StateNode::stop(); // do this last (required)
}

void LogNode::writeImage(unsigned int time, FilterBankGenerator& fbk, bool isPNG) {
	if(imageStartFrame==-1U)
		imageStartFrame = fbk.getFrameNumber();
	
	unsigned char * data = fbk.getImage(ProjectInterface::fullLayer,0);
	size_t datasize = fbk.getImageSize(ProjectInterface::fullLayer,0);
	std::stringstream name;
	name << basename << std::setw(6) << std::setfill('0') << (time - startTime) << (isPNG ? ".png" : ".jpg");
	std::ofstream of((path+'/'+name.str()).c_str());
	if(!of) {
		std::cerr << "*** WARNING could not open file for saving \"" << (path+'/'+name.str()) << "\"" << std::endl;
		return;
	}					
	of.write(reinterpret_cast<char*>(data),datasize);
	if(!of) {
		std::cerr << "*** WARNING saving of " << (path+'/'+name.str()) << " failed " << std::endl;
		return;
	}
	indexFile << name.str() << '\t' << fileCount++ << '\t' << (fbk.getFrameNumber() - imageStartFrame) << '\t' << (time-startTime) << std::endl;
}

void LogNode::writeSensor(unsigned int time) {
	if(sensorStartFrame==-1U)
		sensorStartFrame = ::state->frameNumber;
	PostureEngine pose;
	pose.setSaveFormat(true,::state);
	std::stringstream name;
	name << basename << std::setw(6) << std::setfill('0') << (time - startTime) << ".pos";
	
	FILE* f = fopen((path+'/'+name.str()).c_str(),"w");
	if(f==NULL) {
		std::cerr << "*** WARNING could not open file for saving \"" << (path+'/'+name.str()) << "\"" << std::endl;
		return;
	}
	unsigned int sz = pose.saveFileStream(f);
	if(sz==0) {
		std::cerr << "*** WARNING saving of " << (path+'/'+name.str()) << " failed " << std::endl;
		fclose(f);
		return;
	}
	int err=fclose(f);
	if(err!=0) {
		std::cerr << "*** WARNING error " << err << " while closing " << (path+'/'+name.str()) << std::endl;
		return;
	}
	
	indexFile << name.str() << '\t' << fileCount++ << '\t' << (state->frameNumber - sensorStartFrame) << '\t' << (time-startTime) << std::endl;
}



/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
