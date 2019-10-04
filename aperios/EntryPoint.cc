#include "EntryPoint.h"

void EntryPoint::useResource(Data& d) {
	data.push_back(&d);
	if(data.size()==1 || epFrame==NULL) {
		epFrame=ProcessID::getMapFrame();
		if(epFrame!=NULL) { // check that setMap has been called
#ifdef DEBUG_STACKTRACE
			epFrame->debug=0;
#endif
			getCurrentStackFrame(epFrame);
			while(unrollStackFrame(epFrame,epFrame)) {}
		}
	}
}

void EntryPoint::releaseResource(Data& d) {
	ASSERTRET(data.size()>0,"EntryPoint::releaseResource underflow");
	ASSERT(data.back()==&d,"Warning: data changed between resource usage and release");
	if(data.size()==1) {
#ifdef DEBUG_STACKTRACE
		//if(epFrame!=NULL)
		//memset(epFrame,0,sizeof(StackFrame));
#endif
		epFrame=NULL;
	}
	data.pop_back();
}

/*! @file
* @brief 
* @author Ethan Tira-Thompson (ejt) (Creator)
*/
