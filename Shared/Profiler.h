//-*-c++-*-
#ifndef INCLUDED_Profiler_h_
#define INCLUDED_Profiler_h_

#include "TimeET.h"
#include <string>

//! put this at the beginning of any function for which you wish to collect profiling information
/*! Uses a variable named _PROFSECTION_id to store a static ID number - don't redefine or modify that...
 *  @param NAME the name of this section for reporting
 *  @param PROF the actual profiler to use
 */
#ifndef PROFSECTION
#define PROFSECTION(NAME,PROF) \
  static unsigned int _PROFSECTION_id=(PROF).getNewID(NAME);\
  Profiler::Timer _PROFSECTION_timer(_PROFSECTION_id,&(PROF).prof);
#endif

//! Manages a hierarchy of timers for profiling time spent in code, gives microsecond resolution
/*! Doesn't use any pointers so it's safe to put this in shared memory regions.\n
 *  That's handy so one process can collate all the profiling information across processes
 *  to give a summary report to the user.\n
 *  
 *  Example usage:
 *  - Use a static variable to hold an id number for the code section (doesn't necessarily have to be static, but is faster that way)
 *  - Create a Profiler::Timer object - its construction marks the 'start' time, and its destructor marks the 'stop' time.
 *
 *  @code
 *  ProfilerOfSize<2> prof; //A global manager for all the sections
 *  
 *  void f() {
 *    static unsigned int id=prof.getNewID("f"); // <== Get the ID number 
 *    Profiler::Timer timer(id,&prof.prof);      // <== start the timer
 *    //...
 *    if(rand()>RAND_MAX/2)
 *      return; // destruction of timer occurs automatically!
 *    //...
 *  } // if we didn't hit the return, timer will otherwise destruct here!
 *  @endcode
 *
 *  However, there's a macro that makes this a one liner:
 *  
 *  @code
 *  void g() {
 *    PROFSECTION("g",prof);   // <== Most of the time, this is all you need
 *    //...                    // (unless you're doing something fancy like conditional timers)
 *    f(); // will note f's time as g's child time, as well as g's own time
 *    //...
 *  }
 *  @endcode
 *
 *  The idea is similar to that used by MMAccessor.  If you want to profile a section at smaller
 *  resolution than a function, you can use tricks shown in MMAccessor's documentation to limit
 *  the timer's scope.
 *
 *  For convenience, there are three global profilers predefined: #mainProfiler, #motionProfiler,
 *  and #soundProfiler.  These are what are polled by the ProfilerCheckControl -- if you instantiate
 *  a new profiler, you will have to call its report() function yourself to get the results.  (If it's simply
 *  a matter of running out of sections in mainProfiler, increase the template parameter at the end
 *  of Profiler.h)  Keep in mind however, that these global profilers are pointers, and need to be
 *  dereferenced to use with the macro, e.g. <code>PROFSECTION("g2",*mainProfiler)</code>
 *
 *  Here were the constraints I set for myself:
 *  - Processes can read each other's Profilers - so must be able to live in shared memory\n
 *    This is so one process can generate a report on performance of the entire system at once
 *  - Flexible memory footprint\n
 *    MainObject will probably have a lot of subsections.  MotionObject won't.  Since SectionInfo
 *    takes some significant memory space, we don't want to force MotionObject to needlessly make
 *    a lot of them.
 *  - Flexible usage - can have a single generic global, as well as creating multiple
 *  - Fast - don't want to kill performance of profiled sections, or throw off reported results
 *
 *  Consessions made:
 *  - Sections are not dynamically allocated
 *  - Sections within a Profiler are mutually exclusive (otherwise curSection won't be reliable)
 *  - Results don't try to take affects of pre-emptive multitasking into account.
 *
 *  Global readability is first priority since generating reports is the primary usage, thus
 *  we have to be able to handle being in shared memory space.  This means no virtual functions and
 *  no pointer storage.  Unfortunately, this makes the second constraint rather difficult.\n
 *  Run-time dynamic allocation is right out.  But the number of sections is set at compile time
 *  anyway, so it should be OK to set this at compile time, using a template parameter.\n
 *  That gets us 99% of the way there, but it can be burdensome to use this since the template
 *  means there's no consistant type for all profilers - you can't have a generic Profiler type
 *  if it's templated - you would have to know the size of the profiler you're referring to.\n
 *  That kind of brings in the third constraint... Instead of accepting a single global, I
 *  decided to make a general base (Profiler) and then a templated subclass to hold the bulk data
 *  section.  This has the nice side affect of not having to specify the bulk of the code in the
 *  header, but has the downside that accessing the info stored in the subclass from the super class
 *  is very much a hack.  If you think you can get around this, good luck!
 *
 *  @note This could be made much prettier if we didn't care about the virtual function-shared
 *  memory problems... sigh
 */
class Profiler {
public:
	//! constructor, but you don't want to construct one of these!  Use ProfilerOfSize<x> instead!
	Profiler(unsigned int mx);
	
	//! maximum length of names of timers
	static const unsigned int MaxSectionNameLen=75;
	//! number of slots in the histograms
	static const unsigned int HistSize=32;
	//! the upper bound (exclusive) of the histograms, in milliseconds.
	static const unsigned int HistTime=10*1000;
	//! affects how linearly the buckets are distributed - 1 means linear, >1 causes higher resolution for short times
	static const float HistCurve;
	
		
	//! holds all the information needed for book keeping for each timer
	struct SectionInfo {
		SectionInfo();                     //!< constructor
		void reset();                      //!< resets profiling information
		char name[MaxSectionNameLen];      //!< the name of this timer
		TimeET totalTime;                  //!< the total time spent in this section
		TimeET lastTime;                   //!< time of last call, used to calculate #totalInterval, which gives idea of rate of calls
		TimeET totalInterval;              //!< the total time spent between calls (not time between end of one and start of next, is time between start of one and start of next)
		TimeET childTime;                  //!< the total time spent in child sections
		float execExpAvg;                 //!< exponential average of execution time
		float interExpAvg;                //!< exponential average of inter-call time
		unsigned int execHist[HistSize];   //!< histogram of execution times, uses logarithmic size bins (so high res for quick functions, low res for longer functions)
		unsigned int interHist[HistSize];  //!< histogram of inter-call time, uses logarithmic size bins (so high res for quick functions, low res for longer functions)
		unsigned int calls;                //!< number of calls to this section
	};		
	
	//! Measures the time that this class exists, reports result to a profiler
	/*! Don't bother trying to use this as a quick timer - just use TimeET directly.
	 *  But there are functions to get the elapsed time and such if you insist.  */
	class Timer {
		//! Profiler will need to read out some data that no one else should be depending on
		friend class Profiler;
	public:
		Timer() : _prof(NULL), _id(-1U), _parent(-1U), _t() {} //!< constructor - starts timer, but you can restart it...
		Timer(unsigned int id, Profiler* prof); //!< constructor - starts the timer, sets current timer in @a prof
		Timer(const Timer& t) : _prof(t._prof), _id(t._id), _parent(t._parent),_t(t._t) { } //!< copy constructor, not that you should need it, does same as default
		Timer& operator=(const Timer& t) { _prof=t._prof; _id=t._id; _parent=t._parent; _t=t._t; return *this; } //!< not that you should need it, does same as default
		~Timer(); //!< destructor - stops the timer, reports results
		void setID(unsigned int id, Profiler* prof); //!< sets the ID and profiler, also starts timer
		void start() { _t.Set(); } //!< starts timer (or resets it)
		const TimeET& startTime() { return _t; } //!< returns time of start
		TimeET elapsed() { return _t.Age(); } //!< returns time since start
	protected:
		Profiler* _prof;      //!< the profiler this should report to
		unsigned int _id;     //!< the id number for this code section (See example in beginning of class documentation for how these are assigned)
		unsigned int _parent; //!< the id number of the timer this timer is under
		TimeET _t;            //!< the time this timer was created
	};
	
	//! call this to get a new ID number
	unsigned int getNewID(const char* name);

	//! called during process init (before any profiled sections)
	static void initBuckets();

	//! returns the bucket boundaries
	float* getBuckets() { return buckets; }

	//! outputs profiling information
	std::string report();

	//! resets profiling information
	void reset();
		
	unsigned int curSection;         //!< the current timer
	TimeET startTime;                //!< time of beginning profiling
	float gamma;                    //!< gamma to use with exponential averages (1 to freeze, 0 to set to last)
	const unsigned int maxSections;  //!< so we can read the size of the infos array back again at runtime
	unsigned int sectionsUsed;       //!< the number of timer IDs which have been assigned

	//! gets the actual storage area of the SectionInfo's
	inline SectionInfo* getInfos() { return (SectionInfo*)((char*)this+infosOffset); }
		
protected:
	//! Only the Timer's should be calling setCurrent() and finished() upon the Timer's construction and destruction
	friend class Timer;
	//! called automatically by Timer() - sets the current timer
	void setCurrent(Timer& tr);
	//! called automatically by ~Timer() - notes the specified timer as finished (doesn't check if the timer is actually the current one - don't screw up!)
	void finished(Timer& tr);
	
	//! returns which bucket a time should go in, does a binary search over buckets (unless someone things a log() call would be faster...)
	unsigned int getBucket(float t) {
		unsigned int l=0;          //inclusive
		unsigned int h=HistSize-1; //inclusive
		unsigned int c=(h+l)/2;    //current bucket
		while(l!=h) {
			//			std::cout << this << ' ' << t << '\t' << l << ' ' << c << ' ' << h <<std::endl;
			if(t>buckets[c])
				l=c+1;
			else
				h=c;
			c=(h+l)/2;
		}
		return h;
	}
		
	static float buckets[HistSize];  //!< holds boundaries for each bucket

	static unsigned int infosOffset; //!< NASTY HACK - this is how we get around using virtual functions
	
	//! Automatically causes initialization of the histogram buckets when the first Profiler is instantiated
	class AutoInit {
	public:
		AutoInit(); //!< constructor, adds to #refcount and #totalcount, causes initalization if was 0
		~AutoInit(); //!< destructor, decrements #refcount, does teardown if it hits 0
	protected:
		static unsigned int refcount; //!< the number of profilers in existance
		static unsigned int totalcount; //!< the number of profilers which have been created
	} autoInit;
};

//! templated subclass allows compile-time flexibility of how much memory to use.
template<unsigned int MaxSections>
class ProfilerOfSize {
public:
	ProfilerOfSize() : prof(MaxSections) {} //!< constructor
	
	//! call this to get a new ID number
	unsigned int getNewID(const char* name) { return prof.getNewID(name); }
	
	//! outputs profiling information
	std::string report() { return prof.report(); }
	
	Profiler prof; //!< the profiler that does the work, must immediately preceed #infos!
	Profiler::SectionInfo infos[MaxSections]; //!< the actual profiling information storage
};

// feel free to ignore these or add your own as well -- these are the ones that framework-included code will use
typedef ProfilerOfSize<20> mainProfiler_t; //!< defines type for code profiling information for MainObject; use this instead of the templated type so you don't rely on a particular size
typedef ProfilerOfSize<6> motionProfiler_t; //!< defines type for code profiling information for MotionObject; use this instead of the templated type so you don't rely on a particular size
typedef ProfilerOfSize<6> soundProfiler_t; //!< defines type for code profiling information for SoundPlay; use this instead of the templated type so you don't rely on a particular size
extern mainProfiler_t * mainProfiler; //!< holds code profiling information for MainObject
extern motionProfiler_t * motionProfiler; //!< holds code profiling information for MotoObject
extern soundProfiler_t * soundProfiler; //!< holds code profiling information for SoundPlay

/*! @file
 * @brief Describes Profiler, which managers a hierarchy of timers for profiling time spent in code
 * @author ejt (Creator)
 */

#endif
