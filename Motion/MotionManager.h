//-*-c++-*-
#ifndef INCLUDED_MotionManager_h
#define INCLUDED_MotionManager_h

#include "OutputCmd.h"
#include "OutputPID.h"
#include "Shared/RobotInfo.h"
#include "Shared/StackTrace.h"
#include "IPC/ListMemBuf.h"
#include "IPC/MutexLock.h"
#include "MotionManagerMsg.h"

#ifdef PLATFORM_APERIOS
#  include <OPENR/OPENR.h>
#  include <OPENR/OPENRAPI.h>
#  include <OPENR/OSubject.h>
#  include <OPENR/ObjcommEvent.h>
#  include <OPENR/OObject.h>
#else //PLATFORM_LOCAL
class MessageQueueBase;
class MessageReceiver;
#endif

class EventTranslator;
class MotionCommand;
class RCRegion;
class SharedObjectBase;

//! The purpose of this class is to repeatedly compute the final set of joint angles for the robot, managing a set of (possibly conflicting) MotionCommands
/*! Since Main and Motion run as separate processes, they
 *  could potentially try to access the same motion command at the
 *  same time, leading to unpredictable behavior.  The MotionManager
 *  enforces a set of locks to serialize access to the MotionCommands.
 *  Although you could call checkoutMotion() and checkinMotion() directly,
 *  it is instead recommended to use MMAccessor to automatically handle
 *  casting and checkin for you.
 *
 *  The other problem is that we are sharing the memory holding MotionCommands between processes.
 *  MotionManager will do the necessary magic behind the scenes to distribute new
 *  MotionCommands to all the involved processes (currently just Main and
 *  Motion)\n You can create and add a new motion in one line:
 *
 *  @code
 *  // type A: prunable motions are automatically removed when completed:
 *  motman->addPrunableMotion( SharedObject<YourMC>([arg1,...]) [, priority ] );
 *
 *  // type B: persistent motions are removed only when you explicitly request it:
 *  MC_ID id = motman->addPersistentMotion( SharedObject<YourMC>([arg1,...]) [, priority ] );
 *  // then later: motman->removeMotion(id);
 *  @endcode
 *
 *  The priority level can be changed later via setPriority(), and there are
 *  symbolic values defined in #kIgnoredPriority through #kEmergencyPriority
 *  to give some common guidelines on the magnitudes to use.  The default
 *  priority level if unspecified is #kStdPriority.
 *  
 *  If you want to do some more initializations not handled by the MotionCommand's
 *  constructor (the @p arg1, @p arg2, ...  params) then you would
 *  want to do something like the following:
 *  
 *  @code
 *  SharedObject<YourMC> yourmc([arg1,[arg2,...]]);
 *  yourmc->cmd1();
 *  yourmc->cmd2();
 *  //...
 *  motman->addPrunableMotion(yourmc [, ...]); //or addPersistentMotion(...)
 *  @endcode
 *
 *  Notice that @c yourmc is actually of type SharedObject, but you're calling @c
 *  YourMC's functions on it through the '->' operator...  SharedObject is a "smart pointer" which
 *  will pass your function calls on to the underlying templated type.
 *  Isn't C++ great? :)
 *
 *  @warning Once the MotionCommand has been added, you must check it
 *  out to make any future modifications it or risk concurrent access problems.
 *  In other words, you should @e not keep the SharedObject and continue
 *  to access the motion through that unless you know the motion is not active
 *  in the MotionManager.  Instead, always use a MMAccessor.
 *
 *  @see MMAccessor for information on accessing motions after you've
 *  added them to MotionManager, or if it @e may be active in the MotionManager.
 *
 *  @see MotionCommand for information on creating new motion primitives.
 */
class MotionManager {
public:
	//! This is the number of processes which will be accessing the MotionManager
	/*! Probably just MainObject and MotionObject... This isn't really a
	 *  hard maximum, but should be actual expected, need to know when
	 *  they're all connected */
	static const unsigned int MAX_ACCESS=2;

	static const unsigned int MAX_MOTIONS=64;   //!< This is the maximum number of Motions which can be managed, can probably be increased reasonably without trouble

	typedef MotionManagerMsg::MC_ID MC_ID;      //!< use this type when referring to the ID numbers that MotionManager hands out
	static const MC_ID invalid_MC_ID=MotionManagerMsg::invalid_MC_ID; //!< for errors and undefined stuff

	//!Just to give you some guidelines for what values to use for different priority levels, but you can pick any value you like (that's why they are floats)
	//!@name Priority Level Constants
	static const float kIgnoredPriority;    //!< won't be expressed, handy if you want to temporarily pause something
	static const float kBackgroundPriority; //!< will only be expressed if *nothing* else is using that joint
	static const float kLowPriority;        //!< for stuff that's not background but lower than standard
	static const float kStdPriority;        //!< for every-day commands
	static const float kHighPriority;       //!< for stuff that should override standard stuff
	static const float kEmergencyPriority;  //!< for really important stuff, such as the emergency stop
	//@}

	MotionManager();                            //!< Constructor, sets all the outputs to 0
#ifdef PLATFORM_APERIOS
	void InitAccess(OSubject* subj);            //!< @b LOCKS @b MotionManager Everyone who is planning to use the MotionManager needs to call this before they access it or suffer a horrible fate
	void receivedMsg(const ONotifyEvent& event); //!< @b LOCKS @b MotionManager This gets called by an OObject when it receives a message from one of the other OObject's MotionManagerComm Subject
#else
	void InitAccess(MessageQueueBase& mcbufq, Resource& behaviorLock); //!< @b LOCKS @b MotionManager Everyone who is planning to use the MotionManager needs to call this before they access it or suffer a horrible fate
	static bool receivedMsg(RCRegion* msg); //!< called with incoming messages, will pass non-echos to processMsg()
#endif
	void RemoveAccess(); //!< needed in order to dereference shared memory regions before shutting down
	static void setTranslator(EventTranslator* et) {etrans=et;} //!< sets #etrans, should be called before any events can be sent
	void processMsg(RCRegion* region); //!< @b LOCKS @b MotionManager This gets called by receivedMsg when under Aperios, or directly if you already have an RCRegion
	~MotionManager(); //!<destructor
	void motionReport() const; //!< displays a report of active motion commands on stderr

	//!@name MotionCommand Safe
	void setOutput(const MotionCommand* caller, unsigned int output, const OutputCmd& cmd); //!< @b LOCKS @b MotionManager Requests a value be set for the specified output, copies cmd across frames
	void setOutput(const MotionCommand* caller, unsigned int output, const OutputCmd& cmd, unsigned int frame); //!< @b LOCKS @b MotionManager Requests a value be set for the specified output in the specified frame
	void setOutput(const MotionCommand* caller, unsigned int output, const OutputCmd cmd[NumFrames]); //!< @b LOCKS @b MotionManager Requests a value be set for the specified output across frames
	void setOutput(const MotionCommand* caller, unsigned int output, const OutputPID& pid); //!< @b LOCKS @b MotionManager Requests a PID be set for the specified output, notice that this might be overruled by a higher priority motion
	void setOutput(const MotionCommand* caller, unsigned int output, const OutputCmd& cmd, const OutputPID& pid); //!< @b LOCKS @b MotionManager Requests a value and PID be set for the specified output
	void setOutput(const MotionCommand* caller, unsigned int output, const OutputCmd cmd[NumFrames], const OutputPID& pid); //!< @b LOCKS @b MotionManager Requests a value and PID be set for the specified output
#ifndef TGT_DYNAMIC
	const OutputCmd& getOutputCmd(unsigned int output) const { return cmds[output]; } //!< Returns the value of the output last sent to the OS.  Note that this will differ from the sensed value in state, even when staying still.  There is no corresponding getOutputPID because this value *will* duplicate the value in state.
#endif
	void setPriority(MC_ID mcid, float p) { if ( mcid != invalid_MC_ID ) cmdlist[mcid].priority=p; }//!< sets the priority level of a MotionCommand, symbolic values are available to give some guidelines -- see #kIgnoredPriority through #kEmergencyPriority
	float getPriority(MC_ID mcid) const { return cmdlist[mcid].priority; } //!< returns priority level of a MotionCommand, symbolic values are available to give some guidelines -- see #kIgnoredPriority through #kEmergencyPriority
	//@}

	//@{
	inline MC_ID begin() const         { return skip_ahead(cmdlist.begin()); }   //!< returns the MC_ID of the first MotionCommand
	inline MC_ID next(MC_ID cur) const { return skip_ahead(cmdlist.next(cur)); } //!< returns the MC_ID of MotionCommand following the one that is passed
	inline MC_ID end() const           { return cmdlist.end();      } //!< returns the MC_ID of the one-past-the-end MotionCommand (like the STL)
	inline unsigned int size() const   { return cmdlist.size();     } //!< returns the number of MotionCommands being managed
	//@}

	//!You can have one MC check out and modify another, but make sure the other MC doesn't call setOutput()
	//!@name MotionCommand "Risky"
	MotionCommand * checkoutMotion(MC_ID mcid,bool block=true) const; //!< locks the command and possibly performs RTTI conversion; supports recursive calls
	void checkinMotion(MC_ID mcid) const; //!< marks a MotionCommand as unused
	MotionCommand * peekMotion(MC_ID mcid) const { return mcid==invalid_MC_ID?NULL:cmdlist[mcid].baseaddrs[getAccID()]; } //!< allows access to a MotionCommand without checking it out; warning @b never call a function based on this, only access member fields through it
	unsigned int checkoutLevel(MC_ID mcid) const { return mcid==invalid_MC_ID?0:cmdlist[mcid].lock.get_lock_level(); } //!< returns the number of times @a mcid has been checked out minus the times it's been checked in
	bool isOwner(MC_ID mcid) const { return mcid==invalid_MC_ID?false:(cmdlist[mcid].lock.owner()==getAccID()); }
	//@}

	//!@name MotionCommand Unsafe
	//@{
	//! @b LOCKS @b MotionManager adds a new motion (wrapped in a SharedObject) and marks that it should be automatically deleted when the MotionCommand::isAlive() returns false.
	MC_ID addPrunableMotion(const SharedObjectBase& sm, float priority=kStdPriority) { return doAddMotion(sm,true,priority); }
	//! @b LOCKS @b MotionManager adds a new motion (wrapped in a SharedObject) and marks that it should @e not be deleted, until removeMotion(MC_ID mcid) is called.
	MC_ID addPersistentMotion(const SharedObjectBase& sm, float priority=kStdPriority) { return doAddMotion(sm,false,priority); }
	void removeMotion(MC_ID mcid); //!< @b LOCKS @b MotionManager removes the specified MotionCommand
	//@}

	//@{
	void lock()    { MMlock.lock(getAccID()); } //!< gets an exclusive lock on MotionManager - functions marked @b LOCKS @b MotionManager will cause (and require) this to happen automatically
	bool trylock() { return MMlock.try_lock(getAccID()); } //!< tries to get a lock without blocking
	void unlock() { MMlock.unlock(); } //!< releases a lock on the motion manager
	//@}

	//@{
#ifndef TGT_DYNAMIC
	void getOutputs(float outputs[][NumOutputs]);  //!< @b LOCKS @b MotionManager called by MotionObject to fill in the output values for the next ::NumFrames frames (only MotoObj should call this...)
#endif
#ifdef PLATFORM_APERIOS
	bool updatePIDs(OPrimitiveID primIDs[NumOutputs]);      //!< call this when you want MotionManager to update modified PID values, returns true if changes made (only MotoObj should be calling this...), see PIDMC for general PID documentation
#else
	bool updatePIDs(std::vector<std::pair<unsigned int, float[3]> >& pids);      //!< call this when you want MotionManager to update modified PID values, returns true if changes made (only MotoObj should be calling this...), see PIDMC for general PID documentation
#endif
	//@}

	//! holds the full requested value of an output
	class OutputState {
	public:
		//!@name Constructors
		//!Constructor
		OutputState();
		OutputState(unsigned int out, float pri, MC_ID mc, const OutputCmd cmds[NumFrames]);
		OutputState(unsigned int out, float pri, MC_ID mc, const OutputCmd& cmd);
		OutputState(unsigned int out, float pri, MC_ID mc, const OutputCmd& cmd, unsigned int frame);
		OutputState(unsigned int out, float pri, MC_ID mc, const OutputPID& p);
		OutputState(unsigned int out, float pri, MC_ID mc, const OutputCmd cmds[NumFrames], const OutputPID& p);
		OutputState(unsigned int out, float pri, MC_ID mc, const OutputCmd& cmd, const OutputPID& p);
		//@}
		float priority;             //!< priority level
		MC_ID mcid;                 //!< MC_ID of requester
		OutputCmd frames[NumFrames]; //!< values of output planned ahead
		OutputPID pid;               //!< pid of output
	};
	
	bool hasReference(ProcessID::ProcessID_t proc, MC_ID mcid) const { return cmdlist[mcid].rcr[_MMaccID[proc]]!=NULL; }

protected:
	//!does the actual work of adding a motion
	MC_ID doAddMotion(const SharedObjectBase& sm, bool autoprune, float priority);
	//! sets up a motion command to be accessed by the current process
	MotionCommand* convertMotion(MC_ID mc) const;
	
	//! used to request pids for a given joint
	struct PIDUpdate {
		//!constructor
	 	PIDUpdate() : joint((unsigned int)-1) {}
		//!constructor
		PIDUpdate(unsigned int j, const float p[3]) : joint(j) {
			for(unsigned int i=0; i<3; i++)
				pids[i]=p[i];
		}
		unsigned int joint; //!< the joint ID (see RobotInfo.h for offset values)
		float pids[3]; //!< the PID values to use (see ::Pid )
	};
#ifndef TGT_DYNAMIC
	ListMemBuf<PIDUpdate,NumPIDJoints> pidchanges;  //!< stores PID updates, up to one per joint (if same is set more than once, it's just overwrites previous update)
#endif
	void setPID(unsigned int j, const float p[3]); //!< @b LOCKS @b MotionManager, called internally to do the work of setting the PID... you probably want to call setOutput with an OutputPID argument, not this...

	typedef unsigned short accID_t; //!< type to use to refer to accessors of MotionManager (or its locks)

	void func_begin() const { MMlock.lock(getAccID()); } //!< called at the begining of many functions to lock MotionManager
	void func_end() const { MMlock.unlock(); } //!< called at the end of a function which called func_begin() to release it
	template<class T> T func_end(T val) const { func_end(); return val; } //!< same as func_end(), except passes return value through

	MC_ID skip_ahead(MC_ID mcid) const; //!< during iteration, skips over motioncommands which are still in transit from on OObject to another
		
	//!All the information we need to maintain about a MotionCommand
	struct CommandEntry {
		//! Constructor, sets everything to basics
		CommandEntry() : lastAccessor((unsigned short)-1),lock(),priority(MotionManager::kStdPriority), trace(stacktrace::recordStackTrace(10,5)) {
			for(unsigned int i=0; i<MAX_ACCESS; i++) {
				baseaddrs[i]=NULL;
				rcr[i]=NULL;
			}
		}
		~CommandEntry() { stacktrace::freeStackTrace(trace); trace=NULL; }
		MotionCommand * baseaddrs[MAX_ACCESS]; //!< for each accessor, the base address of the motion command
		RCRegion * rcr[MAX_ACCESS];            //!< for each accessor the shared memory region that holds the motion command
		mutable accID_t lastAccessor;                  //!< the ID of the last accessor to touch the command (which implies if it wants to touch this again, we don't have to convert again)
		mutable MutexLock<MAX_ACCESS> lock;            //!< a lock to maintain mutual exclusion
		float priority;                        //!< MotionCommand's priority level
		stacktrace::StackFrame* trace; //records the stack trace where the motion was added to the motion manager for leak tracking
	private:
		CommandEntry(const CommandEntry&); //!< this shouldn't be called...
		CommandEntry& operator=(const CommandEntry&); //!< this shouldn't be called...
	};
	ListMemBuf<CommandEntry,MAX_MOTIONS,MC_ID> cmdlist;     //!< the list where MotionCommands are stored, remember, we're in a shared memory region with different base addresses - no pointers!
	MC_ID cur_cmd; //!< MC_ID of the MotionCommand currently being updated by getOutputs(), or NULL if not in getOutputs.  This is used by the setOutput()'s to tell which MotionCommand is calling


	inline MC_ID pop_free() { return cmdlist.new_front(); } //!<pulls an entry from cmdlist's free section and returns its index
	inline void push_free(MC_ID a) { cmdlist.erase(a); }    //!<puts an entry back into cmdlist's free section

	mutable MutexLock<MAX_ACCESS> MMlock;          //!< The main lock for the class

	typedef ListMemBuf<OutputState,MAX_MOTIONS> cmdstatelist_t; //!< shorthand for a list of OutputState's
#ifndef TGT_DYNAMIC
	cmdstatelist_t cmdstates[NumOutputs];  //!< requested positions by each of the MC's for each of the outputs
	float cmdSums[NumOutputs];             //!<Holds the final values for the outputs of the last frame generated
	OutputCmd cmds[NumOutputs];            //!<Holds the weighted values and total weight for the outputs of the last frame
#endif

	accID_t numAcc;                        //!<The number of accessors who have registered with InitAccess()
#ifdef PLATFORM_APERIOS
	OSubject* subjs[MAX_ACCESS];           //!<The OSubject for each process (accessor) on which it should be broadcast when a command is added
#else //PLATFORM_LOCAL
	//!Storage of each process's attachment of the message queue, used to internally transmit sound buffers to SoundPlay
	MessageQueueBase * subjs[MAX_ACCESS];
	MessageReceiver * mcrecvs[MAX_ACCESS]; //!< message receivers which watch for incoming motion command regions, or requests to free them
	Resource* procLocks[MAX_ACCESS]; //!< pointers to per-process thread locks, acquired during message processing from one of #mcrecvs
#endif

	static int getAccID() { return _MMaccID[ProcessID::getID()]; }
	static int _MMaccID[ProcessID::NumProcesses]; //!<Stores the accessor id assigned in InitAccess() for each process
	static EventTranslator* etrans; //!< EventTranslator for sending events to Main -- each process will set the correct value for calls within that process.

private:
	MotionManager(const MotionManager&); //!< this shouldn't be called...
	MotionManager& operator=(const MotionManager&); //!< this shouldn't be called...
};

//!anyone who includes MotionManager.h will be wanting to use the global motman... don't want multiple of these! created by MotoObj
extern MotionManager * motman;

/*! @file
 * @brief Describes MotionManager, simplifies sharing of MotionCommand's and provides mutual exclusion to their access
 * @author ejt (Creator)
 */

#endif
