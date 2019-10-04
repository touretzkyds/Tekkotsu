#include "FreeMemReportControl.h"
#include "Shared/debuget.h"
#ifdef PLATFORM_APERIOS
#  include <AnalyzerAPI.h>
#  include <AnalyzerError.h>
#endif

REGISTER_CONTROL_OPT(FreeMemReportControl,"Status Reports",BEH_START);

void FreeMemReportControl::doEvent() {
	ASSERTRET(event->getGeneratorID()==EventBase::timerEGID,"Unexpected event");
	if(event->getSourceID()==0) {
		report();
	} else if(event->getSourceID()==1) {
		size_t freemem=freeMem();
		if(freemem<low_mem) {
			if(!isWarning) {
				isWarning=true;
				serr->printf("WARNING: Low memory: %lu\n",(unsigned long)freemem);
			}
		} else {
			if(isWarning) {
				isWarning=false;
				report();
			}
		}
	} else {
		ASSERT(false,"Unexpected timer event");
	}
}
	
void FreeMemReportControl::refresh() {
	char tmp[256];
	sprintf(tmp,"Free Mem: %lu",(unsigned long)freeMem());
	options[1]->setName(tmp);
	ControlBase::refresh();
	report();
}


//! reports size of free memory - if this is below low_mem, also generates a warning
void FreeMemReportControl::report() {
	size_t freemem=freeMem();
	sout->printf("%lu bytes free (%+ld)\n",(unsigned long)freemem,(long)(freemem-lastReport));
	lastReport=freemem;
	if(freemem<low_mem)
		if(isWarning)
			serr->printf("WARNING: Low memory: %lu\n",(unsigned long)freemem);
	resetTimerFreq();
}

//! returns the size of the free memory
size_t FreeMemReportControl::freeMem() {
	size_t freemem;
#ifdef PLATFORM_APERIOS
	if (AnalyzerGetSizeOfFreeMemory(&freemem)!=azrSUCCESS)
		sout->printf("Aperios error: getsizeoffreememory failed\n");
#else
	freemem=-1U;
#endif
	return freemem;
}

void FreeMemReportControl::resetTimerFreq() {
	if(report_freq<0)
		erouter->removeTimer(this,0);
	else
		erouter->addTimer(this,0,report_freq,true);
	erouter->addTimer(this,1,monitor_freq,true);
}

/*! @file
 * @brief Implements FreeMemReportControl, which gives reports on free memory size at various (configurable) rates
 * @author ejt (object), alokl (core function)
 */

