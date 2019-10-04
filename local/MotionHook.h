//-*-c++-*-
#ifndef INCLUDED_MotionHook_h_
#define INCLUDED_MotionHook_h_

#include <cstring>
#include "Shared/plist.h"
#include "Shared/RobotInfo.h"
#include <vector>

//! Interface for connections to remote hosts and hardware devices which should be polled with output values
/*! 
 *  - Override motionCheck() if your hardware needs to have a value specified for every output
 *    on every update, regardless of whether it changes.
 *  - Override motionUpdated() if you only need to process the outputs which have changed.
 *
 *  You can expect to be called every FrameTime*NumFrame milliseconds in terms of simulator time.
 *  However, keep in mind this is relative to SharedGlobals::timeScale (Config.Speed) in terms of
 *  wall-clock time, and is also subject to the simulator being paused, set to full-speed mode, or hitting 
 *  a breakpoint in the debugger.  See enteringRealtime() and leavingRealtime() if you want updates 
 *  when the user switches simulation modes, although there's still no way to get notification if a
 *  debugger breakpoint is hit.
 */
class MotionHook {
public:
	//! used as input to updatePIDs()
	struct PIDUpdate {
		//! default constructor (unknown output, 0 pids)
		PIDUpdate() : idx(-1U) { pids[0]=0; pids[1]=0; pids[2]=0; }
		//! constructor, pass output index and pid array
		PIDUpdate(const std::pair<unsigned int, float[3]>& x) : idx(x.first) { memcpy(pids,x.second,sizeof(pids)); }
		unsigned int idx; //!< output index
		float pids[3]; //!< pid values
	};
	
	//! constructor
	MotionHook() : verbose(0), isFirstCheck(true) {}
	
	//! no-op destructor
	virtual ~MotionHook() {}
	
	//! Called when motion process is starting
	virtual void motionStarting() {}
	
	//! Should return true if the MotionHook is successfully connected to physical hardware.
	/*! If relevant, this will only be called after motionStarting() has been called in order to 
	 *  initialize a connection.
	 *
	 *  This is used mainly to cancel out of the WaitForSensors if all MotionHooks return false.
	 *  If you are still in the process of connecting or unsure of status, be optimistic and return true.
	 *  This function will be polled at a coarse rate while blocked on sensors in case of timeouts
	 *  on the part of the MotionHook render it moot. */
	virtual bool isConnected()=0;
	
	//! Called each time the motion process has polled active motion commands
	/*! When in realtime mode, this should be called every FrameTime*NumFrames (defined in the RobotInfo)
	 *  milliseconds if running at full speed.  See enteringRealtime() and leavingRealtime().
	 *
	 *  This default implementation checks to see which outputs have changed value since the last call and
	 *  passes the summary on to motionUpdated().  #lastOutputs will be updated with the new values @e after
	 *  the call to motionUpdated().
	 *
	 *  If you need to process all the outputs on every frame, you only need to override this function.
	 *  Your subclass doesn't need to call the MotionHook implementation unless you want to have
	 *  lastOutputs updated for you.
	 *
	 *  If you only need to process the @e changed outputs for each frame, override motionUpdated() instead.
	 *  motionUpdated() is always called for each update, even if there aren't any changes, so you can still
	 *  use that if there are some outputs which need to be updated every cycle.  */
	virtual void motionCheck(const float outputs[][NumOutputs]) {
		std::vector<size_t> changedIndices;
		changedIndices.reserve(NumOutputs);
		for(size_t i=0; i<NumOutputs; ++i) {
			if(isFirstCheck) {
				changedIndices.push_back(i);
			} else {
				for(size_t j=0; j<NumFrames; ++j) { // if *any* of the frames have changed, update the output
					if(outputs[j][i]!=lastOutputs[i]) { // (not just checking last frame for each output)
						changedIndices.push_back(i);
						break;
					}
				}
			}
		}
		motionUpdated(changedIndices,outputs);
		for(size_t i=0; i<NumOutputs; ++i)
			lastOutputs[i] = outputs[NumFrames-1][i];
		isFirstCheck=false;
	}
	
	//! Called by motionCheck(), after comparing the new output values to #lastOutputs, and before lastOutputs is updated
	/*! Override this if you only need to send commands to the hardware for values that have changed. 
	 *  This function is always called for each update, even though changedIndices might be empty. */
	virtual void motionUpdated(const std::vector<size_t>& /*changedIndices*/, const float /*outputs*/[][NumOutputs]) {}
	
	//! Called when PID values change
	virtual void updatePIDs(const std::vector<PIDUpdate>& pids) {}
	
	//! Called when motion process is stopping
	virtual void motionStopping() { isFirstCheck=true; }
	
	//! Called when the controller is going to be running in realtime mode, which is probably the normal mode you'd expect.
	/*! You might be in realtime mode, but a debugger breakpoint will still pause things, or thread scheduling could hiccup, so try to be robust.\n
	 *  The argument is a reference to SharedGlobals::timeScale, so the data source can subscribe to changes in
	 *  simulation speed if it can use that information.  (We avoid direct dependency on the tekkotsu simulator
	 *  so this code can be reused for other tools too.) */
	virtual void enteringRealtime(const plist::Primitive<double>& /*simTimeScale*/) {}
	
	//! Called when leaving realtime mode, which means you have no idea when motionCheck() is going to be called in terms of wall-clock time.
	/*! Argument set to true if entering full speed mode, which @e may mean motionCheck will be
	 *  called at a high(er) frequency, or slower the computation is overwhelming the host hardware.
	 *  However, if false, almost certainly indicates updates will be sparse.
	 *  May be called multiple times if changing between full-speed mode and paused
	 *
	 *  A non-realtime mode might be triggered if the user wants to pause the simulator/controller to step through something...
	 *  No guarantees though!  The debugger might catch a breakpoint and stop things, and this won't be called! */
	virtual void leavingRealtime(bool /*isFullSpeed*/) {}
	
	//! Called by simulator thread to indicate level of verbosity for diagnostics and reporting errors
	virtual void setMotionHookVerbose(int v) { verbose=v; }

protected:
	//! stores current verbosity
	int verbose;
	//! set to false following the first motionCheck, reset to true by motionStopping
	bool isFirstCheck;
	//! stores the last frame of the outputs, updated by motionCheck()
	float lastOutputs[NumOutputs];
};

/*! @file
 * @brief Describes MotionHook, an interface for connections to remote hosts and hardware devices which should be polled with output values
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
