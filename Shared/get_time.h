#ifndef INCLUDED_get_time_h
#define INCLUDED_get_time_h

#ifdef PLATFORM_APERIOS

//! a simple way to get the current time since boot in milliseconds (see TimeET for better accuracy)
unsigned int get_time();

//! just to provide compatability with simulator code, this function always returns '1' since we're running in realtime on real hardware
float getTimeScale();

#else //PLATFORM_LOCAL

//! If we're running locally, these will let users in "project" space control time for simulation
namespace project_get_time {
	//! This will default to -1, which signals get_time() to use get_time_callback.  Any other value will cause that value to be used instead.
	extern unsigned int simulation_time;
	//! This by default will return the time in milliseconds since the first call was made.  Note this is a function pointer, so you can reassign it to your own implementation!
	/*! For instance, the simulator can assign a function which forwards
	 *  the call to SharedGlobals::get_time(), so that all processes get
	 *  consistent time values under control of the simulator
	 *  Note that this is a slightly different usage paradigm than get_timeScale_callback(), which is probably cleaner, but trying to avoid spreading dependency on TimeET.h */
	extern unsigned int (*get_time_callback)();

	/*! @brief You can reassign this to your own implementation if you might play games with time control.
	 *  For instance, the simulator can assign a function which 
	 *  simply returns SharedGlobals::timeScale.
	 *  By default this is NULL, which indicates to getTimeScale that it should use the default implementation. 
	 *  Note that this is a slightly different usage paradigm than get_time_callback(), which is assumed to always be non-NULL (at least, unless you assign a value to #simulation_time...)  */
	extern float (*get_timeScale_callback)();
}

//! This will call and return project_get_time::get_time_callback if project_get_time::simulation_time is -1.  Otherwise simulation_time is returned.
/*! Default values are set such that the system clock will be used,
 *  and values will range from 0 (first call) onward.   However, by
 *  reassigning project_get_time::get_time_callback to your own
 *  function, you can control the flow of time however you wish. */
inline unsigned int get_time() {
	if(project_get_time::simulation_time==-1U)
		return (*project_get_time::get_time_callback)();
	else
		return project_get_time::simulation_time;
}

//! If project_get_time::get_timeScale_callback is NULL, this will return 1 if project_get_time::simulation_time is -1U (the default value), and -1 otherwise.  If the callback is available, returns that.
inline float getTimeScale() {
	if(project_get_time::get_timeScale_callback)
		return (*project_get_time::get_timeScale_callback)();
	else if(project_get_time::simulation_time==-1U)
		return 1;
	else
		return -1;
}

#endif


/*! @file
 * @brief prototype for get_time(), a simple way to get the current time since boot in milliseconds
 * @author ejt (Creator)
 */

#endif
