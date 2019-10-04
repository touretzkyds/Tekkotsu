#include "get_time.h"

#ifdef PLATFORM_APERIOS

#include <MCOOP.h>
unsigned int get_time() {
  static struct SystemTime time;
  GetSystemTime(&time);
  return time.seconds*1000+time.useconds/1000;
}

#else

#include "TimeET.h"
namespace project_get_time {
	
	unsigned int simulation_time=-1U;
	
	//! provides default implementation of #get_time_callback -- starts a TimeET instance on the first call and then returns its age thereafter
	unsigned int default_get_time_callback() {
		static TimeET first;
		return static_cast<unsigned int>(first.Age().Value()*1000);
	}
	
	unsigned int (*get_time_callback)()=&default_get_time_callback;
	
	float (*get_timeScale_callback)()=NULL;
	
}

#endif

/*! @file
 * @brief Implementation of get_time(), a simple way to get the current time since boot in milliseconds
 * @author ejt (Creator)
 */
