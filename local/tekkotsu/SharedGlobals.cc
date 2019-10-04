#include "SharedGlobals.h"
#include "Simulator.h"

const char * const SharedGlobals::runlevel_names[SharedGlobals::NUM_RUNLEVELS+1] = {
	"CREATED",
	"CONSTRUCTING",
	"STARTING",
	"RUNNING",
	"STOPPING",
	"DESTRUCTING",
	"DESTRUCTED",
	NULL
};

ipc_setup_t * ipc_setup=NULL;
SharedGlobals * globals=NULL;

unsigned int SharedGlobals::get_time() {
	decltype(lastTimeScale) lastLastTimeScale=lastTimeScale; // used to avoid repeated "pause" commands if we're past autoPauseTime
	if(timeScale<=0) {
		//just need to update lastTimeScale in case it had been realtime mode
		//adding an 'if' to test before the assignment is slower than just always doing the assignment
		lastTimeScale=timeScale;
	} else {
		if(lastTimeScale<=0) {
			//switching from non-realtime to realtime mode -- reset time offset
			timeOffset=bootTime.Age().Value()*timeScale*1000-simulatorTime;
			lastTimeScale=timeScale;
			//we reset timeOffset such that simulatorTime hasn't changed
		} else if(lastTimeScale!=timeScale) {
			//switching speeds -- reset time offset
			simulatorTime=get_real_time(lastTimeScale);
			timeOffset=bootTime.Age().Value()*timeScale*1000-simulatorTime;
			lastTimeScale=timeScale;
		} else {
			simulatorTime=get_real_time(timeScale);
		}
		//cout << timeOffset << ' ' << lastTimeScale << ' ' << timeScale << endl;
	}
	if(simulatorTime>autoPauseTime) {
		simulatorTime=autoPauseTime;
		if(ProcessID::getID()==ProcessID::SimulatorProcess) {
			if(timeScale!=0) // don't reset it if it's already set -- triggers infinite recursion if/when the hook calls get_time
				timeScale=0;
		} else {
			// can't set timeScale directly because Simulator process has hooks registered and we aren't in the Simulator process
			if(lastLastTimeScale!=0) {
				lastTimeScale=0;
				Simulator::sendCommand("pause quiet"); // so do this instead
			}
		}
	}
	return simulatorTime;
}

float SharedGlobals::getTimeScale() const { return (float)timeScale; }

/*! @file
 * @brief A class to hold various simulator parameters which need to be accessed from multiple processes
 * @author ejt (Creator)
 */
