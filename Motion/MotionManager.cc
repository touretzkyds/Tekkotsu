#include "MotionManager.h"
#include "MotionCommand.h"
#include "Events/EventRouter.h"
#include "Shared/StackTrace.h"
#include "Shared/ProjectInterface.h"
#include "Shared/debuget.h"
#include "Shared/WorldState.h"
#include "IPC/SharedObject.h"

#ifndef PLATFORM_APERIOS
#  include "IPC/MessageQueue.h"
#  include "IPC/MessageReceiver.h"
#  include "Shared/ProjectInterface.h"
#endif

#include "Shared/ERS210Info.h"
#include "Shared/ERS220Info.h"
#include "Shared/ERS7Info.h"
#include "Shared/Config.h"

#include <list>

MotionManager * motman=NULL;
int MotionManager::_MMaccID[ProcessID::NumProcesses];
EventTranslator* MotionManager::etrans=NULL;

const float MotionManager::kIgnoredPriority    =-1;
const float MotionManager::kBackgroundPriority = 0;
const float MotionManager::kLowPriority        = 5;
const float MotionManager::kStdPriority        = 10;
const float MotionManager::kHighPriority       = 50;
const float MotionManager::kEmergencyPriority  = 100;

using namespace std;

//! just for convenience
typedef unsigned int uint;

MotionManager::MotionManager()
: pidchanges(),cmdlist(),cur_cmd(invalid_MC_ID),MMlock(),numAcc(0)
{
	for(uint x=0; x<NumOutputs; x++)
		cmdSums[x]=0;
}

#ifdef PLATFORM_APERIOS

void
MotionManager::InitAccess(OSubject* subj) {
	if(numAcc==MAX_ACCESS) {
		printf("*** ERROR *** attempted to register more accessors with MotionManager than allowed by MAX_ACCESS\n");
		return;
	}
	_MMaccID[ProcessID::getID()]=numAcc++;
 	//	cout << "ID is now " << getAccID() << " of " << numAcc << endl;
	MMlock.lock(getAccID());
	//	accRegs[accID].init();
	subjs[getAccID()]=subj;
	if(cmdlist.size()>0) //Shouldn't happen - busy wait in addMotion
		cout << "*** WARNING *** MOTIONS ADDED BEFORE ALL INITACCESSED" << endl;
	MMlock.unlock();
}

#else //now PLATFORM_LOCAL

void
MotionManager::InitAccess(MessageQueueBase& mcbufq, Resource& behaviorLock) {
	if(numAcc==MAX_ACCESS) {
		printf("*** ERROR *** attempted to register more accessors with MotionManager than allowed by MAX_ACCESS\n");
		return;
	}
	_MMaccID[ProcessID::getID()]=numAcc++;
 	//	cout << "ID is now " << getAccID() << " of " << numAcc << endl;
	MMlock.lock(getAccID());
	subjs[getAccID()]=&mcbufq;
	mcrecvs[getAccID()]=new MessageReceiver(*subjs[getAccID()],receivedMsg);
	procLocks[getAccID()]=&behaviorLock;
	if(cmdlist.size()>0) //Shouldn't happen - busy wait in doAddMotion
		cout << "*** WARNING *** MOTIONS ADDED BEFORE ALL INITACCESSED" << endl;
	MMlock.unlock();
}

#endif //PLATFORM-specific initialization

void
MotionManager::RemoveAccess() {
#ifndef PLATFORM_APERIOS
	//kill the message receiver before we get the motion manager lock
	// that way if there's a message pending, it can be processed instead of deadlocking
	mcrecvs[getAccID()]->finish();
	delete mcrecvs[getAccID()];
	mcrecvs[getAccID()]=NULL;
#endif
	
	func_begin();
	for(MC_ID mc_id=cmdlist.begin(); mc_id!=cmdlist.end(); mc_id=cmdlist.next(mc_id)) {
		if(cmdlist[mc_id].rcr[getAccID()]!=NULL) {
			MotionCommand* mc=checkoutMotion(mc_id,true);
			int found=0;
			for(unsigned int i=0; i<numAcc; i++) {
				if(cmdlist[mc_id].rcr[i]!=NULL) {
					found++;
				}
			}
			cout << "Warning: " << ProcessID::getIDStr() << " dropping motion command " << mc_id << " of type " << typeid(*mc).name() << ", was active at shutdown " << (mc->getAutoPrune()?"(was set for autoprune)":"(leaked?)") << '\n';
			cout << "         Motion command " << mc_id << " was created from:" << endl;
			stacktrace::displayStackTrace(cmdlist[mc_id].trace);
#ifdef PLATFORM_APERIOS
			int refs=1;
#else
			int refs=cmdlist[mc_id].rcr[getAccID()]->NumberOfLocalReference();
#endif
			if(refs>1)
				cout << "Warning: " << ProcessID::getIDStr() << " still had " <<  refs-1 << " references to motion command " << mc_id << " as it was being dropped (these are now invalid)" << endl;
			for(int i=0; i<refs; ++i)
				cmdlist[mc_id].rcr[getAccID()]->RemoveReference();
			cmdlist[mc_id].rcr[getAccID()]=NULL;
			if(found==1) {
				MC_ID p=cmdlist.prev(mc_id);
				push_free(mc_id); //don't check in -- lock destructor will release the lock as the entry is destructed
				mc_id=p;
			} else {
				checkinMotion(mc_id);
			}
		}
	}
	func_end();
}


MotionManager::~MotionManager() {
	if(!cmdlist.empty()) {
		func_begin();
		cout << "WARNING: MotionManager destruction with MotionCommands still attached." << endl;
		while(!cmdlist.empty()) {
			MC_ID mc_id=cmdlist.begin();
			for(unsigned int i=0; i<numAcc; i++)
				if(cmdlist[mc_id].rcr[i]!=NULL)
					cout << "MC " << mc_id << " was still referenced by InitAccess caller #" << getAccID() << endl;
			push_free(mc_id);
		}
		func_end();
	}
}


void
MotionManager::motionReport() const {
	func_begin();
	std::cout << cmdlist.size() << " live motion commands\n";
	for(MC_ID mc_id=cmdlist.begin(); mc_id!=cmdlist.end(); mc_id=cmdlist.next(mc_id)) {
		MotionCommand* mc=checkoutMotion(mc_id,true);
		std::cout << "MC_ID " << mc_id << " type " << typeid(*mc).name() << " priority " << cmdlist[mc_id].priority << '\n';
		std::cout << "Created by:" << std::endl;
		stacktrace::displayStackTrace(cmdlist[mc_id].trace);
		checkinMotion(mc_id);
	}
	std::cout << "\nOutput Usage: (MCID @ value [* weight])\n";
	bool any=false;
	for(unsigned int i=0; i<NumOutputs; ++i) {
		bool first=true;
		for(unsigned int j=cmdstates[i].begin(); j!=cmdstates[i].end(); j=cmdstates[i].next(j)) {
			if(cmdstates[i][j].frames[NumFrames-1].weight > 0) {
				if(first) {
					std::cout << "   " << outputNames[i] << ':';
					first=false;
					any=true;
				}
				std::cout << " (" << cmdstates[i][j].mcid << " @ " << cmdstates[i][j].frames[NumFrames-1].value;
				if(cmdstates[i][j].frames[NumFrames-1].weight!=1)
					std::cout << " * " << cmdstates[i][j].frames[NumFrames-1].weight;
				std::cout << ")";
			}
		}
		if(!first)
			std::cout << '\n';
	}
	if(!any)
		std::cout << "   [No outputs in use]" << std::endl;
	func_end();
}


void
MotionManager::setOutput(const MotionCommand* caller, unsigned int output, const OutputCmd& cmd) {
	if(output >= NumOutputs)
		return;
	if(cmd.weight<=0)
		return;

	if(caller==NULL || caller->getID()!=cur_cmd)
		func_begin();
	if(cur_cmd==invalid_MC_ID) {
		cmdSums[output]=cmd.value;
	} else if(getPriority(cur_cmd)>=kBackgroundPriority) {
		cmdstatelist_t& curstatelist=cmdstates[output];
		cmdstatelist_t::index_t ent=curstatelist.begin();
		if(ent==curstatelist.end() || curstatelist[ent].mcid!=cur_cmd)
			curstatelist.push_front(OutputState(output,getPriority(cur_cmd),cur_cmd,cmd));
		else
			for(unsigned int i=0; i<NumFrames; i++)
				curstatelist[ent].frames[i]=cmd;
	}
	if(caller==NULL || caller->getID()!=cur_cmd)
		func_end();
}

void
MotionManager::setOutput(const MotionCommand* caller, unsigned int output, const OutputCmd& cmd, unsigned int frame) {
	if(output >= NumOutputs)
		return;
	if(cmd.weight<=0)
		return;

	if(caller==NULL || caller->getID()!=cur_cmd)
		func_begin();
	if(cur_cmd==invalid_MC_ID) {
		cmdSums[output]=cmd.value;
	} else if(getPriority(cur_cmd)>=kBackgroundPriority) {
		cmdstatelist_t& curstatelist=cmdstates[output];
		cmdstatelist_t::index_t ent=curstatelist.begin();
		if(ent==curstatelist.end() || curstatelist[ent].mcid!=cur_cmd)
			curstatelist.push_front(OutputState(output,getPriority(cur_cmd),cur_cmd,cmd,frame));
		else
			curstatelist[ent].frames[frame]=cmd;
	}
	if(caller==NULL || caller->getID()!=cur_cmd)
		func_end();
}

void 
MotionManager::setOutput(const MotionCommand* caller, unsigned int output, const OutputCmd ocmds[NumFrames]) {
	if(output >= NumOutputs)
		return;
	unsigned int hasWeight=NumFrames;
	for(unsigned int i=NumFrames-1; i!=-1U; i--)
		if(ocmds[i].weight>0) {
			hasWeight=i;
			break;
		}
	if(hasWeight==NumFrames)
		return;
	

	if(caller==NULL || caller->getID()!=cur_cmd)
		func_begin();
	if(cur_cmd==invalid_MC_ID) {
		cmdSums[output]=ocmds[hasWeight].value;
	} else if(getPriority(cur_cmd)>=kBackgroundPriority) {
		cmdstatelist_t& curstatelist=cmdstates[output];
		cmdstatelist_t::index_t ent=curstatelist.begin();
		if(ent==curstatelist.end() || curstatelist[ent].mcid!=cur_cmd)
			curstatelist.push_front(OutputState(output,getPriority(cur_cmd),cur_cmd,ocmds));
		else
			for(unsigned int i=0; i<NumFrames; i++)
				curstatelist[ent].frames[i]=ocmds[i];
	}
	if(caller==NULL || caller->getID()!=cur_cmd)
		func_end();
}

void
MotionManager::setOutput(const MotionCommand* caller, unsigned int output, const OutputPID& pid) {
	if(output >= NumOutputs)
		return;

	if(caller==NULL || caller->getID()!=cur_cmd)
		func_begin();
	if(cur_cmd==invalid_MC_ID) {
		setPID(output,pid.pid);
	} else if(getPriority(cur_cmd)>=kBackgroundPriority) {
		cmdstatelist_t& curstatelist=cmdstates[output];
		cmdstatelist_t::index_t ent=curstatelist.begin();
		if(ent==curstatelist.end() || curstatelist[ent].mcid!=cur_cmd)
			curstatelist.push_front(OutputState(output,getPriority(cur_cmd),cur_cmd,pid));
		else
			curstatelist[ent].pid=pid;
	}
	if(caller==NULL || caller->getID()!=cur_cmd)
		func_end();
}

void
MotionManager::setOutput(const MotionCommand* caller, unsigned int output, const OutputCmd& cmd, const OutputPID& pid) {
	if(output >= NumOutputs)
		return;

	if(caller==NULL || caller->getID()!=cur_cmd)
		func_begin();
	if(cur_cmd==invalid_MC_ID) {
		if(cmd.weight>0)
			cmdSums[output]=cmd.value;
		setPID(output,pid.pid);
	} else if(getPriority(cur_cmd)>=kBackgroundPriority) {
		cmdstatelist_t& curstatelist=cmdstates[output];
		cmdstatelist_t::index_t ent=curstatelist.begin();
		if(ent==curstatelist.end() || curstatelist[ent].mcid!=cur_cmd)
			curstatelist.push_front(OutputState(output,getPriority(cur_cmd),cur_cmd,cmd,pid));
		else {
			for(unsigned int i=0; i<NumFrames; i++)
				curstatelist[ent].frames[i]=cmd;
			curstatelist[ent].pid=pid;
		}
	}
	if(caller==NULL || caller->getID()!=cur_cmd)
		func_end();
}

void
MotionManager::setOutput(const MotionCommand* caller, unsigned int output, const OutputCmd ocmds[NumFrames], const OutputPID& pid) {
	if(output >= NumOutputs)
		return;

	if(caller==NULL || caller->getID()!=cur_cmd)
		func_begin();
	if(cur_cmd==invalid_MC_ID) {
		if(ocmds[NumFrames-1].weight>0)
			cmdSums[output]=ocmds[NumFrames-1].value;
		setPID(output,pid.pid);
	} else if(getPriority(cur_cmd)>=kBackgroundPriority) {
		cmdstatelist_t& curstatelist=cmdstates[output];
		cmdstatelist_t::index_t ent=curstatelist.begin();
		if(ent==curstatelist.end() || curstatelist[ent].mcid!=cur_cmd)
			curstatelist.push_front(OutputState(output,getPriority(cur_cmd),cur_cmd,ocmds,pid));
		else {
			for(unsigned int i=0; i<NumFrames; i++)
				curstatelist[ent].frames[i]=ocmds[i];
			curstatelist[ent].pid=pid;
		}
	}
	if(caller==NULL || caller->getID()!=cur_cmd)
		func_end();
}

/*! What's worse? A plethora of functions which are only called, and only useful at one place,
 *  or a big massive function which doesn't pollute the namespace?  This is the latter, for
 *  better or worse. */
void
MotionManager::getOutputs(float outputs[][NumOutputs]) {
	//	if(begin(id)!=end())
	//if(state && state->buttons[LFrPawOffset]) cout << "getAngles..." << flush;
	if(state==NULL) {
		// we haven't gotten the WorldState memory region from Main yet, just set LEDs to a wierd pattern and leave
		for(uint f=0;f<NumFrames;f++)
			for(uint i=0; i<NumOutputs; i++)
				outputs[f][i]=0;
		for(uint f=0;f<NumFrames;f++)
			for(uint l=0; l<NumLEDs; l++)
				outputs[f][l]=l/(NumLEDs-1.0f);
		//	if(begin(id)!=end())
		//if(state && state->buttons[LFrPawOffset])	cout << "getangles-nostate-done..." << flush;
		return;
	}
	func_begin();
	//	if(begin(id)!=end())
	//	cout << id << "..." << flush;
	//cout << "CHECKOUT..." << flush;
	for(uint output=0; output<NumOutputs; output++)
		cmdstates[output].clear();

	// for each PID joint which is set to 0 power, set the background
	// position value to current sensed value this prevents jerking back
	// to the previous position when joint(s) are moved during 0 power,
	// and then power is turned back on. (power here is in the 0 pid
	// sense, the joints are still receiving power from the system -
	// that's a separate system call)
	// Note that we wouldn't want to do this all the time, because
	// miscalibration between the sensed position and target position
	// will cause joints to drift to the extremities of motion, in some
	// cases, very quickly, and in worse cases, colliding with other
	// joints
	for(uint output=0; output<NumPIDJoints; output++)
		if(state->pids[output][0]==0 && state->pids[output][1]==0 && state->pids[output][2]==0)
			cmdSums[output]=state->outputs[output];

	//std::cout << "UPDATE..." << std::flush;
	std::list<MC_ID> unlocked;
	for(MC_ID mcNum=begin(); mcNum!=end(); mcNum=next(mcNum)) // check out all the MotionCommands (only one at a time tho)
		if(cmdlist[mcNum].lastAccessor!=(accID_t)-1)
			unlocked.push_back(mcNum);
	unsigned int lastProcessed=get_time();
	while(unlocked.size()>0) { // keep cycling through all the locks we didn't get
		for(std::list<MC_ID>::iterator it=unlocked.begin(); it!=unlocked.end(); ) {
			MotionCommand* mc=checkoutMotion(*it,false);
			if(mc==NULL)
				it++; //we didn't get a lock, skip it (we'll keep looping until we get it)
			else {
				// we got a lock
				cur_cmd=*it;
				bool prune=true;
				try {
					prune=mc->shouldPrune();
				} catch(const std::exception& ex) {
					ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionCommand prune test, will prune",&ex);
				} catch(...) {
					ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionCommand prune test, will prune",NULL);
				}
				if(prune) {
					// cout << "Removing expired " << *it << " (autoprune)" << endl;
					checkinMotion(*it); // release lock, done with motion (don't need to (and shouldn't) keep lock through the removeMotion())
					//only the last process to receive the remove notification actually does the remove, and
					//wouldn't be able to undo the thread portion of the lock made in this process
					//so we have to take off our own lock here first.
					removeMotion(*it);
				} else {
					try {
					  if ( cmdlist[mc->getID()].priority >= kBackgroundPriority )
							mc->updateOutputs(); // the MotionCommand should make calls to setOutput from within here
					} catch(const std::exception& ex) {
						ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionCommand updateOutputs",&ex);
					} catch(...) {
						ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionCommand updateOutputs",NULL);
					}
					checkinMotion(*it); // release lock, done with motion
				}
				cur_cmd=invalid_MC_ID;
				// remove id from list of unprocessed motioncommands
				std::list<MC_ID>::iterator rem=it++;
				unlocked.erase(rem);
				lastProcessed=get_time();
			}
		}
		if(get_time()-lastProcessed>FrameTime*NumFrames/2)
			break;
	}
	/* Kill annoying warning for now.  -- DST
	if(unlocked.size()>0) {
		cerr << "Warning: MotionManager was unable to obtain a lock on MCs: ";
		for(std::list<MC_ID>::iterator it=unlocked.begin(); it!=unlocked.end(); it++)
			cerr << *it << ' ';
		cerr << endl;
		cerr << (unlocked.size()>1?"They":"It") << " may have been left locked by a Behavior while it was busy." << endl;
		cerr << "Try reducing the scope of your MMAccessor or call checkinMotion sooner." << endl;
	}
	*/

	// sort the list of requested outputs based on priority
	// (insertion sort, data structure is linked list)
	for(uint output=0; output<NumOutputs; output++) {
		cmdstatelist_t& curstatelist=cmdstates[output];
		for(cmdstatelist_t::index_t bit=curstatelist.begin(); bit!=curstatelist.end(); bit=curstatelist.next(bit)) {
			MC_ID high_ent=bit;
			float high_p=cmdlist[curstatelist[high_ent].mcid].priority;
			for(cmdstatelist_t::index_t cit=curstatelist.next(bit); cit!=curstatelist.end(); cit=curstatelist.next(cit)) {
				float curp=cmdlist[curstatelist[cit].mcid].priority;
				if(curp>high_p) {
					high_p=curp;
					high_ent=cit;
				}
			}
			curstatelist.swap(bit,high_ent);
			/*if(curstatelist.countf()!=curstatelist.countb() || curstatelist.countf()!=curstatelist.size()) {
				cout << "LOST ONE! " << bit << ' ' << high_ent << endl;
				cout << curstatelist.countf() << ' ' << curstatelist.countb() << ' ' << curstatelist.size() << endl;
				}*/
			bit=high_ent;
		}
	}

	// now we've got, for each output, a list of requested values sorted by priority
	// summarize each output
	for(uint frame=0; frame<NumFrames; frame++) {
		for(uint output=0; output<NumOutputs; output++) {
			cmdstatelist_t& curstatelist=cmdstates[output];
			float alpha=1;
			OutputCmd sumcmd;
			/*if(curstatelist.size()>1) {
				cout << get_time() << "." << frame << " " << outputNames[output] << ": ";
				for(cmdstatelist_t::index_t ent=curstatelist.begin(); ent!=curstatelist.end(); ent=curstatelist.next(ent))
					cout << "  (" << curstatelist[ent].frames[frame].value <<',' << curstatelist[ent].frames[frame].weight << ',' << curstatelist[ent].priority << ')';
				cout << endl;
			}*/
			cmdstatelist_t::index_t ent=curstatelist.begin();
			while(ent!=curstatelist.end() && alpha>0) {
				OutputCmd curcmd;
				float curp=curstatelist[ent].priority;
				float curalpha=1; // curalpha is cumulative product of leftovers (weights between 0 and 1)
				for(;ent!=curstatelist.end() && curstatelist[ent].priority==curp; ent=curstatelist.next(ent)) {
					//weighted average within priority level
					float curweight=curstatelist[ent].frames[frame].weight;
					ASSERT(curweight>=0,"negative output weights are illegal, MC_ID="<<curstatelist[ent].mcid<<" joint="<<outputNames[output]<<" frame="<<frame<<" weight="<<curweight);
					if(curweight<0) //negative weights are illegal
						curweight=0;
					curcmd.value+=curstatelist[ent].frames[frame].value*curweight;
					curcmd.weight+=curweight;
					if(curweight<1)
						curalpha*=(1-curweight);
					else
						curalpha=0;
				}
				if(curcmd.weight>0) {
					//weighted average of priority levels
					sumcmd.value+=curcmd.value/curcmd.weight*alpha*(1-curalpha);
					sumcmd.weight+=alpha*(1-curalpha);
					alpha*=curalpha;
				}
			}

			//if(curstatelist.size()>1)
			//	cout << "   -> " << sumcmd.value/sumcmd.weight << ',' << sumcmd.weight << " @ " << alpha << endl;
			if(sumcmd.weight>0) {
				sumcmd.value/=sumcmd.weight;
				outputs[frame][output]=sumcmd.value;
			} else //if zero weight, hold last value
				sumcmd.value=outputs[frame][output]=cmdSums[output];
			if(frame==NumFrames-1)
				cmds[output]=sumcmd;
		}
	}
	
	for(uint output=0; output<NumOutputs; output++)
		cmdSums[output]=outputs[NumFrames-1][output];

	for (uint frame_idx = 0; frame_idx < NumFrames; frame_idx++) {
		for(uint output=PIDJointOffset; output<PIDJointOffset+NumPIDJoints; output++)
			outputs[frame_idx][output] = (outputs[frame_idx][output] + config->motion.calibration_offset[output-PIDJointOffset])
				* config->motion.calibration_scale[output-PIDJointOffset];
	}
			
				
	// now summarize each output's PID values (for those which use PID control)
	for(uint output=PIDJointOffset; output<PIDJointOffset+NumPIDJoints; output++) {
		cmdstatelist_t& curstatelist=cmdstates[output];
		float alpha=1;
		float sumpid[3];
		for(uint i=0; i<3; i++)
			sumpid[i]=0;
		float sumweight=0;
		cmdstatelist_t::index_t ent=curstatelist.begin();
		while(ent!=curstatelist.end() && alpha>0) {
			float tmppid[3];
			for(uint i=0; i<3; i++)
				tmppid[i]=0;
			float tmpweight=0;
			float curp=curstatelist[ent].priority;
			float curalpha=1; // curalpha is multiplicative sum of leftovers (weights between 0 and 1)
			for(;ent!=curstatelist.end() && curstatelist[ent].priority==curp; ent=curstatelist.next(ent)) {
				//weighted average within priority level
				float curweight=curstatelist[ent].pid.weight;
				ASSERT(curweight>=0,"negative PID weights are illegal")
				if(curweight<0) //negative weights are illegal
					curweight=0;
				for(uint i=0; i<3; i++)
					tmppid[i]+=curstatelist[ent].pid.pid[i]*curweight;
				tmpweight+=curweight;
				if(curweight<1)
					curalpha*=(1-curweight);
				else
					curalpha=0;
			}
			if(tmpweight>0) {
				//weighted average of priority levels
				for(uint i=0; i<3; i++)
					sumpid[i]+=tmppid[i]/tmpweight*(1-curalpha);
				sumweight+=(1-curalpha);
				alpha*=curalpha;
			}
		}
		if(sumweight>0) {
			for(uint i=0; i<3; i++)
				sumpid[i]/=sumweight;
			setPID(output,sumpid);
		}
	}

	func_end();
	//	if(begin(id)!=end())
	//if(state && state->buttons[LFrPawOffset]) cout << "getAngles-done." << flush;
}

/*! This function handles the conversion from the Tekkotsu format (one
 *  regular IEEE float per parameter, to the OPEN-R format (which
 *  takes a specialized, reduced precision floating point number) This
 *  is all documented in PIDMC as well.
 *
 *  In order to send Tekkotsu's PIDs to the system, they are converted
 *  to the gain/shift format.  On the ERS-2xx, we could dynamically
 *  choose shift values to allow more precision in setting values.
 *
 *  With the ERS-7, all shifts are shared, so they must be set to a
 *  common set of values, defined by WorldState::DefaultPIDShifts.
 *  This limits the range of gains which can then be set.
 *
 *  Due to the mysterious warning which would occur with the 2xx,
 *  (AGRMSDriver::SetGain() : 0x0A IS USED FOR GAIN SHIFT VALUE.)  and
 *  since this seems to be the way things are going, all models now,
 *  by default, use global shift values (which can vary from model to
 *  model, just global for each model)
 *
 *  You can revert to the dynamic shift selection by commenting-in
 *  the noted code section below.
 *
 *  A final note: the OPENR::SetJointGain function seems to be
 *  a rather costly function call.  You should probably try to avoid
 *  setting PIDs at too high a frequency.
 */
#ifdef PLATFORM_APERIOS
bool
MotionManager::updatePIDs(OPrimitiveID primIDs[NumOutputs]) {
	//cout << "updatePIDs " << endl;
	bool dirty=!pidchanges.empty();
	while(!pidchanges.empty()) {
		float gain[3];
		word shift[3];
		
		//if you want to enforce the default shifts:
		for(uint i=0; i<3; i++) {
			shift[i]=DefaultPIDShifts[i];
		  gain[i]=pidchanges.front().pids[i]*(1<<(0x10-shift[i]));
		}
		
		//if you want to allow shifts to move for better precision:
		// this is OK on 2xx, although it occasionally produces warnings like:
		// AGRMSDriver::SetGain() : 0x0A IS USED FOR GAIN SHIFT VALUE.
		// It still seems to work fine though.
		// HOWEVER, the ERS-7 is a different story.  Apparently ( https://openr.aibo.com/cgi-bin/openr/e_regi/im_trbbs.cgi?uid=general&df=bbs.dat&prm=TAN&pg=1&no=0893#0893 )
		// all joints share the same shift, and so there must be one
		// global setting enforced.  If this is the way things are heading,
		// might as well just have everyone use the first method instead.
		// but here's the more dynamic way just for posterity:
		/*
		for(uint i=0; i<3; i++) {
			gain[i]=pidchanges.front().pids[i]*2;
			shift[i]=0xF;
			while(shift[i]!=2 && (gain[i]!=(word)gain[i] || gain[i]<=1) && gain[i]<0x20) {
				gain[i]*=2;
				shift[i]--;
			}
		}
		 */
		
		//some debugging output (pick your favorite joint - i was having trouble with the ERS-7 nod joint in particular)
		//if(pidchanges.front().joint==HeadOffset+NodOffset)
		//cout << (word)gain[0] << ' ' << shift[0] << "   " << (word)gain[1] << ' ' << shift[1] << "   " << (word)gain[2] << ' ' << shift[2] << endl;
		//cout << gain[0] << ' ' << shift[0] << "   " << gain[1] << ' ' << shift[1] << "   " << gain[2] << ' ' << shift[2] << endl;
		
		OPENR::SetJointGain(primIDs[pidchanges.front().joint],(word)gain[0],(word)gain[1],(word)gain[2],shift[0],shift[1],shift[2]);
		for(uint i=0; i<3; i++)
			state->pids[pidchanges.front().joint][i]=pidchanges.front().pids[i];
		pidchanges.pop_front();
	}
	return dirty;
}
#else
bool
MotionManager::updatePIDs(std::vector<std::pair<unsigned int, float[3]> >& pids) {
	//cout << "updatePIDs " << endl;
	bool dirty=!pidchanges.empty();
	std::pair<unsigned int, float[3]> entry;
	while(!pidchanges.empty()) {
		entry.first=pidchanges.front().joint;
		for(uint i=0; i<3; i++)
			state->pids[entry.first][i] = entry.second[i] = pidchanges.front().pids[i];
		pids.push_back(entry);
		pidchanges.pop_front();
	}
	return dirty;
}
#endif //PLATFORM_APERIOS or PLATFORM_LOCAL

#ifdef PLATFORM_APERIOS
void
MotionManager::receivedMsg(const ONotifyEvent& event) {
	//	cout << "receivedMsg..." << flush;
	func_begin();
	//	cout << id << "..." << flush;
	//cout << "Received at " << get_time() << endl;
	for(int x=0; x<event.NumOfData(); x++)
		processMsg(event.RCData(x));
	//	cout << "receivedMsg-done" << endl;
	func_end();
}
#else //PLATFORM_LOCAL
bool
MotionManager::receivedMsg(RCRegion* msg) {
	try {
		Thread::NoCancelScope nc;
		MarkScope l(*motman->procLocks[getAccID()]);
		/*MotionManagerMsg * mminfo = reinterpret_cast<MotionManagerMsg*>(msg->Base());
		if(mminfo->creatorPID==ProcessID::getID())
			return true; //don't process echos*/
		motman->processMsg(msg);
	} catch(const std::exception& ex) {
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionManagerMsg processing (MotionManager::receivedMsg)",&ex))
			throw;
	} catch(...) {
		if(!ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionManagerMsg processing (MotionManager::receivedMsg)",NULL))
			throw;
	}
	return true;
}
#endif //PLATFORM_APERIOS or PLATFORM_LOCAL

void
MotionManager::processMsg(RCRegion * rcr) {
	//cout << "processMsg..." << flush;
	if(rcr==NULL) {
		cout << "WARNING: MotionManager::processMsg was given a NULL region" << endl;
		return;
	}
	MotionManagerMsg * mminfo = reinterpret_cast<MotionManagerMsg*>(rcr->Base());
#ifndef PLATFORM_APERIOS
	if(mminfo->creatorPID==ProcessID::getID())
		return; // don't process echos
#endif

	func_begin();
	const accID_t MYACCID = getAccID();
	//cout << ProcessID::getIDStr() << "..." << flush;
	MC_ID mc_id=mminfo->mc_id;
	switch(mminfo->type) {
		case MotionManagerMsg::addMotion: {
			if(mc_id==invalid_MC_ID) { // || cmdlist[mc_id].lastAccessor==(accID_t)-1
				cout << "WARNING: MotionManager::processMsg addMotion for invalid_MC_ID motion" << endl;
				func_end();
				return;
			}
			if(cmdlist[mc_id].rcr[MYACCID]!=NULL) {
				cerr << "WARNING: MotionManager::processMsg addMotion for motion which was already added!" << endl;
			} else {
				//cout << "receiveMotion()NOW: rcr->NumberOfReference()==" << rcr->NumberOfReference() << endl;
				cmdlist[mc_id].rcr[MYACCID]=rcr;
				//cout << "receiveMotion(): rcr->NumberOfReference()==" << rcr->NumberOfReference() << endl;
				rcr->AddReference();
				//should be able to do a nice dynamic cast instead of a static one
				// but it gives NULL for some reason - i blame having to do the fork trick
				//cout << "Adding mc_id=="<< mc_id << " (and dynamic_cast is still " << dynamic_cast<MotionCommand*>(mminfo) << ")" << endl;
				cmdlist[mc_id].baseaddrs[MYACCID]=static_cast<MotionCommand*>(mminfo);
			}
			if(ProcessID::getID()==ProcessID::MotionProcess && cmdlist[mc_id].lastAccessor!=(accID_t)-1) {
				checkoutMotion(mc_id,true);
				try {
					cmdlist[mc_id].baseaddrs[MYACCID]->start();
				} catch(const std::exception& ex) {
					ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionCommand::start()",&ex);
				} catch(...) {
					ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionCommand::start()",NULL);
				}
				checkinMotion(mc_id);
			}
		} break;
		case MotionManagerMsg::deleteMotion: {
			//cout << "deleteMotion(): cmdlist[mc_id].rcr[MYACCID]->NumberOfReference()==" << cmdlist[mc_id].rcr[MYACCID]->NumberOfReference() << endl;
			//cout << "deleting mc_id=="<<mc_id << "..." << flush;
			ASSERT(cmdlist[mc_id].lastAccessor==(accID_t)-1,"delete motion message for motion command not marked for deletion");
			if(cmdlist[mc_id].rcr[MYACCID]==NULL) {
				cout << "WARNING: MotionManager attempted to delete a NULL motion! mc_id="<<mc_id<<" process=" << ProcessID::getIDStr() << endl;
				stacktrace::displayCurrentStackTrace();
			} else {
				cmdlist[mc_id].lock.lock(MYACCID);
				unsigned int found=0;
				for(unsigned int i=0; i<numAcc; i++) {
					if(cmdlist[mc_id].rcr[i]!=NULL) {
						found++;
					}
				}
				ASSERT(found>0,"WARNING: MotionManager::processMsg underflow");
				if(found==1 && mc_id==cmdlist[mc_id].baseaddrs[MYACCID]->mc_id) // now removing last reference, and no in-flight re-add under new ID
					cmdlist[mc_id].baseaddrs[MYACCID]->clearID();
				cmdlist[mc_id].rcr[MYACCID]->RemoveReference();
				cmdlist[mc_id].rcr[MYACCID]=NULL;
				cmdlist[mc_id].baseaddrs[MYACCID]=NULL;
				if(found==1) {
					push_free(mc_id); //don't unlock -- lock destructor will release the lock as the entry is destructed
				} else {
					cmdlist[mc_id].lock.unlock();
				}
				if(ProcessID::getID()==ProcessID::MainProcess) {
					try {
						erouter->postEvent(EventBase::motmanEGID,mc_id,EventBase::deactivateETID,0);
					} catch(const std::exception& ex) {
						ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during event posting",&ex);
					} catch(...) {
						ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during event posting",NULL);
					}
				}
			}
			//cout << "deleteMotion()NOW: cmdlist[mc_id].rcr[MYACCID]->NumberOfReference()==" << cmdlist[mc_id].rcr[MYACCID]->NumberOfReference() << endl;
		} break;
		default:
			printf("*** WARNING *** unknown MotionManager msg type received\n");
	}
	//cout << "processMsg-done" << endl;
	func_end();
}


/*! We have made this function protected because it's more readable if you
*  call addPrunableMotion() or addPersistentMotion() instead... we decided
*  requiring people to pass a true/false arguement wouldn't make it clear
*  what that true/false was controlling. */
MotionManager::MC_ID 
MotionManager::doAddMotion(const SharedObjectBase& sm, bool autoprune, float priority) {
	MotionCommand * mc = dynamic_cast<MotionCommand*>(static_cast<MotionManagerMsg*>(sm.data()));
	if(mc==NULL) {
		cout << "MotionManager::addMotion() - SharedObject does not seem to hold a MotionCommand" << endl;
		return invalid_MC_ID;
	}
	mc->setAutoPrune(autoprune);
	//cout << "addMotion...";
	while(numAcc<MAX_ACCESS-1) { std::cout << "WAIT" << std::flush; } //Wait for everyone to register
	func_begin();
	const accID_t MYACCID = getAccID();
	//cout << cmdlist.size() << " exist..." << endl;
	//	cout << id << "..." << flush;
	for(MC_ID it=cmdlist.begin(); it!=cmdlist.end(); it=cmdlist.next(it)) {
		if(cmdlist[it].baseaddrs[MYACCID]==mc) {
			cerr << "Warning: adding motion command at " << mc << ", is already running in motion manager as MC_ID " << it << endl;
			return func_end(it);
		}
	}
	MC_ID mc_id = pop_free();
	//cout << sm.getRegion()->ID().key << " holds " << mc_id << endl;
	if(mc_id==cmdlist.end()) {
		cout << "MotionManager::addMotion() - Out of room, could not add" << endl;
		return func_end(cmdlist.end());
	}
	//cout << "setAdd(" << mc_id << ")" << endl;
	mc->setAdd(mc_id);
	cmdlist[mc_id].baseaddrs[MYACCID]=mc;
	cmdlist[mc_id].rcr[MYACCID]=sm.getRegion();
	//cout << "addMotion(): sm.getRegion()->NumberOfReference()==" << sm.getRegion()->NumberOfReference() << endl;
	cmdlist[mc_id].rcr[MYACCID]->AddReference();
	//cout << "addMotion()NOW: sm.getRegion()->NumberOfReference()==" << sm.getRegion()->NumberOfReference() << endl;
	cmdlist[mc_id].lastAccessor=MYACCID;
	cmdlist[mc_id].priority=priority;
	try {
		erouter->postEvent(EventBase::motmanEGID,mc_id,EventBase::activateETID,0);
	} catch(const std::exception& ex) {
		ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during event posting",&ex);
	} catch(...) {
		ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during event posting",NULL);
	}
	
	// only send remove message if there is still a reference
	// otherwise, the previous add message is probably still in transit, and we cleared the mc_id, so it will be ignored
#ifdef PLATFORM_APERIOS
	OStatus err;
	/*{
	 unsigned int i=0;
	 for(ObserverConstIterator it=subjs[MYACCID]->begin();it!=subjs[MYACCID]->end();it++) {
	 cout << "RemainBuffer("<<i++<<")==" << subjs[MYACCID]->RemainBuffer(*it) << endl;
	 }
	 ASSERT((int)i==subjs[MYACCID]->NumberOfObservers(),"did I miss an observer?");
	 }*/
	//cout << "Sent at " << get_time() << flush;
	err=subjs[MYACCID]->SetData(sm.getRegion());
	ASSERT(err==oSUCCESS,"*** ERROR MotionManager: SetData returned " << err);
	//cout << "addMotion()afterSetData: sm.getRegion()->NumberOfReference()==" << sm.getRegion()->NumberOfReference() << endl;
	err=subjs[MYACCID]->NotifyObservers();
	ASSERT(err==oSUCCESS,"*** ERROR MotionManager: NotifyObservers returned " << err);
#else //PLATFORM_LOCAL
	try {
		subjs[MYACCID]->sendMessage(sm.getRegion());
	} catch(const std::exception& ex) {
		ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during message sending",&ex);
	} catch(...) {
		ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during message sending",NULL);
	}
#endif //PLATFORM check for IPC stuff
	//	cout << "addMotion-done" << endl;
	//cout << " - " << get_time() << endl;
	return func_end(mc_id);
}

MotionCommand *
MotionManager::checkoutMotion(MC_ID mcid,bool block) const {
	//cout << "checkout..." << flush;
	if(mcid>=MAX_MOTIONS || mcid==invalid_MC_ID) {
		cout << "*** ERROR *** " << ProcessID::getIDStr() << " tried to access invalid mcid " << mcid << endl;
		stacktrace::displayCurrentStackTrace();
		return NULL;
	}
	if(cmdlist[mcid].lastAccessor==(accID_t)-1) { // test both before and after the block
		cout << "*** WARNING *** " << ProcessID::getIDStr() << " tried to access dead mcid " << mcid << endl;
		stacktrace::displayCurrentStackTrace();
		return NULL;
	}
	// block would lock up if the motion is dead in the simulator (left in a locked state)
	if(block)
		cmdlist[mcid].lock.lock(getAccID());
	else
		if(!cmdlist[mcid].lock.try_lock(getAccID()))
			return NULL;
	if(cmdlist[mcid].lastAccessor==(accID_t)-1) { // test both before and after the block
		cout << "*** WARNING *** " << ProcessID::getIDStr() << " tried to access mcid as it was removed " << mcid << endl;
		stacktrace::displayCurrentStackTrace();
		cmdlist[mcid].lock.unlock();
		return NULL;
	}
	//cout << "locked..." << endl;
	//cout << "checkout-done..." << flush;
	return convertMotion(mcid);
}

MotionCommand *
MotionManager::convertMotion(MC_ID mc) const {
	const accID_t MYACCID = getAccID();
	MotionCommand * base = cmdlist[mc].baseaddrs[MYACCID];
	//	cout << "base=" << base << "..." << flush;
	if(cmdlist[mc].lastAccessor!=MYACCID) {
		//cout << "converting from " << MCRegistrar::getRaw(base) << "..." << flush;
		//cout << "prev=" << accRegs[cmdlist[mcid].lastAccessor].getReg(base) << "..." << flush;
		//		accRegs[id].convert(base);
		//cout << "to=" << MCRegistrar::getRaw(base) << ", " << accRegs[cmdlist[mcid].lastAccessor].getReg(base) << endl;
		cmdlist[mc].lastAccessor=MYACCID;
	}
	base->setTranslator(etrans);
#ifdef PLATFORM_APERIOS
	base->setWorldState(state);
#endif
	return base;
}

void
MotionManager::checkinMotion(MC_ID mcid) const {
	if(mcid>=MAX_MOTIONS || mcid==invalid_MC_ID) {
		cout << "*** ERROR *** " << ProcessID::getIDStr() << " tried to checkin invalid mcid " << mcid << endl;
		stacktrace::displayCurrentStackTrace();
		return;
	}
	const accID_t MYACCID = getAccID();
	if(cmdlist[mcid].lock.get_lock_level()==1 && cmdlist[mcid].rcr[MYACCID]!=NULL) { //about to release last lock (and region hasn't been removed)
		MotionCommand * base = cmdlist[mcid].baseaddrs[MYACCID];
		base->setTranslator(NULL);
#ifdef PLATFORM_APERIOS
		base->setWorldState(NULL);
#endif
	}
	cmdlist[mcid].lock.unlock();
}

void
MotionManager::removeMotion(MC_ID mcid) {
	if(mcid>=MAX_MOTIONS || mcid==invalid_MC_ID)
		return;
	func_begin();
	const accID_t MYACCID = getAccID();
	if(cmdlist[mcid].lastAccessor==(accID_t)-1) {
		cout << "WARNING: removeMotion called for a motion which has already been removed mc_id="<<mcid<<" process=" << ProcessID::getIDStr() << endl;
		stacktrace::displayCurrentStackTrace();
		func_end();
		return;
	}
	if(cmdlist[mcid].rcr[MYACCID]==NULL) { 
		cout << "WARNING: removeMotion called for a NULL motion! mc_id="<<mcid<<" process=" << ProcessID::getIDStr() << endl;
		stacktrace::displayCurrentStackTrace();
		func_end();
		return;
	}
	cmdlist[mcid].lock.lock(MYACCID);
	MotionCommand * mc=checkoutMotion(mcid,true);
	try {
		mc->stop();
	} catch(const std::exception& ex) {
		ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionCommand::stop()",&ex);
	} catch(...) {
		ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during MotionCommand::stop()",NULL);
	}
	checkinMotion(mcid);
	cmdlist[mcid].lastAccessor=(accID_t)-1;
	cmdlist[mcid].rcr[MYACCID]->RemoveReference();
	cmdlist[mcid].rcr[MYACCID]=NULL;
	cmdlist[mcid].baseaddrs[MYACCID]=NULL;
	cmdlist[mcid].lock.unlock();
	if(ProcessID::getID()==ProcessID::MainProcess) {
		try {
			erouter->postEvent(EventBase::motmanEGID,mcid,EventBase::deactivateETID,0);
		} catch(const std::exception& ex) {
			ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during event posting",&ex);
		} catch(...) {
			ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during event posting",NULL);
		}
	}
#ifdef PLATFORM_APERIOS
	MotionManagerMsg dmsg;
	dmsg.setDelete(mcid);
	//cout << "Remove at " << get_time() << flush;
	subjs[MYACCID]->SetData(&dmsg,sizeof(dmsg));
	subjs[MYACCID]->NotifyObservers();
#else //PLATFORM_LOCAL
	// local will "echo" the message, so we'll do the actual remove when we get the echo
	SharedObject<MotionManagerMsg> dmsg;
	dmsg->setDelete(mcid);
	//cout << "Remove at " << get_time() << flush;
	try {
		subjs[MYACCID]->sendMessage(dmsg.getRegion());
	} catch(const std::exception& ex) {
		ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during message sending",&ex);
	} catch(...) {
		ProjectInterface::uncaughtException(__FILE__,__LINE__,"Occurred during message sending",NULL);
	}
#endif //PLATFORM check for IPC stuff
	func_end();
}


/*! Note that we don't actually set the PIDs in the system here, we just queue them up.
 *  PID changes seem to be an expensive operation, so may only want to clear the queue
 *  at some reduced rate (although that's not actually currently being done, it just
 *  could be) */
void
MotionManager::setPID(unsigned int joint, const float pids[3]) {
	func_begin();

	//see if there's already an update for this joint
	for(uint u = pidchanges.begin(); u!=pidchanges.end(); u=pidchanges.next(u)) {
		if(pidchanges[u].joint==joint) { //found it
			for(uint i=0; i<3; i++) {
				pidchanges[u].pids[i]=pids[i];
				if(pids[i]!=state->pids[joint][i]) { //see if we're setting back to current PID
					for(i++; i<3; i++) //we aren't, copy over the rest
						pidchanges[u].pids[i]=pids[i];
					func_end();
					return;
				}
			}
			//if it didn't return within the loop, no difference was found from current state
			//so just delete the update
			pidchanges.erase(u);
			func_end();
			return;
		}
	}

	//if we completed the for loop, we didn't find an update for the joint
	for(uint i=0; i<3; i++) //check to see if it's different from the current
		if(pids[i]!=state->pids[joint][i]) {
			PIDUpdate update(joint,pids); //it is different, insert a new update
			pidchanges.push_back(update);
			// or for debugging:
			//ListMemBuf<PIDUpdate,NumPIDJoints>::index_t it=pidchanges.push_back(update);
			//ASSERT(it!=pidchanges.end(),"MotionManager ran out of pidchanges entries!");
			break;
		}
	func_end();
}


MotionManager::MC_ID
MotionManager::skip_ahead(MC_ID mcid) const {
	// this is in case a new motion has been added, but the current
	// process hasn't received its own copy yet, so should skip over them
	while(mcid!=cmdlist.end() && cmdlist[mcid].rcr[getAccID()]==NULL)
		mcid=cmdlist.next(mcid);
	return mcid;
}

MotionManager::OutputState::OutputState()
	: priority(0),mcid(MotionManager::invalid_MC_ID), pid()
{}
MotionManager::OutputState::OutputState(unsigned int out, float pri, MC_ID mc, const OutputCmd cmds[NumFrames])
	: priority(pri),mcid(mc), pid(DefaultPIDs[out])
{
	for(unsigned int i=0; i<NumFrames; i++)
		frames[i]=cmds[i];
}
MotionManager::OutputState::OutputState(unsigned int out, float pri, MC_ID mc, const OutputCmd& cmd)
	: priority(pri),mcid(mc), pid(DefaultPIDs[out])
{
	for(unsigned int i=0; i<NumFrames; i++)
		frames[i]=cmd;
}
MotionManager::OutputState::OutputState(unsigned int out, float pri, MC_ID mc, const OutputCmd& cmd, unsigned int frame)
	: priority(pri),mcid(mc), pid(DefaultPIDs[out])
{
	frames[frame]=cmd;
}
MotionManager::OutputState::OutputState(unsigned int /*out*/, float pri, MC_ID mc, const OutputPID& p)
	: priority(pri),mcid(mc), pid(p)
{}
MotionManager::OutputState::OutputState(unsigned int /*out*/, float pri, MC_ID mc, const OutputCmd cmds[NumFrames], const OutputPID& p)
	: priority(pri),mcid(mc), pid(p)
{
	for(unsigned int i=0; i<NumFrames; i++)
		frames[i]=cmds[i];
}
MotionManager::OutputState::OutputState(unsigned int /*out*/, float pri, MC_ID mc, const OutputCmd& cmd, const OutputPID& p)
	: priority(pri),mcid(mc), pid(p)
{
	for(unsigned int i=0; i<NumFrames; i++)
		frames[i]=cmd;
}


/*! @file
 * @brief Implements MotionManager, simplifies sharing of MotionCommand's and provides mutual exclusion to their access
 * @author ejt (Creator)
 */


/*
		for(uint f=0;f<NumFrames;f++)
			for(uint i=0; i<NumOutputs; i++)
				outputs[f][i]=0;
		const uint cyctime=128;
		uint ot=get_time()+3*cyctime;
		for(uint f=0;f<NumFrames;f++) {
			uint t=ot+f*FrameTime;
			outputs[f][TopBrLEDOffset]=(t/(double)cyctime-t/cyctime)*.75+.25;
			t-=cyctime;
			outputs[f][TopLLEDOffset]=(t/(double)cyctime-t/cyctime)*.75+.25;
			outputs[f][TopRLEDOffset]=(t/(double)cyctime-t/cyctime)*.75+.25;
			t-=cyctime;
			outputs[f][MidLLEDOffset]=(t/(double)cyctime-t/cyctime)*.75+.25;
			outputs[f][MidRLEDOffset]=(t/(double)cyctime-t/cyctime)*.75+.25;
			t-=cyctime;
			outputs[f][BotLLEDOffset]=(t/(double)cyctime-t/cyctime)*.75+.25;
			outputs[f][BotRLEDOffset]=(t/(double)cyctime-t/cyctime)*.75+.25;
		}
*/

	/*	for(uint output=TlRedLEDOffset-1; output<LEDOffset+NumLEDs-1; output++) {
		cmdstatelist_t& curstatelist=cmdstates[output];
		cout << "Out " << output << ": ";
		for(cmdstatelist_t::index_t bit=curstatelist.begin(); bit!=curstatelist.end(); bit=curstatelist.next(bit))
			cout << '('<<curstatelist[bit].mcid<<','<<cmdlist[curstatelist[bit].mcid].priority<<','<<curstatelist[bit].frames[0].value<<','<<curstatelist[bit].frames[0].weight<<") ";
		cout << endl;
		}*/


	/*	cout << get_time() << ' ' << size() << endl;
	for(uint output=TlRedLEDOffset; output<LEDOffset+NumLEDs-1; output++) {
		cmdstatelist_t& curstatelist=cmdstates[output];
		cout << "Out " << output << ": ";
		for(cmdstatelist_t::index_t bit=curstatelist.begin(); bit!=curstatelist.end(); bit=curstatelist.next(bit))
			cout << '('<<curstatelist[bit].mcid<<','<<cmdlist[curstatelist[bit].mcid].priority<<','<<curstatelist[bit].frames[0].value<<','<<curstatelist[bit].frames[0].weight<<") ";
		cout << endl;
	}
	*/
