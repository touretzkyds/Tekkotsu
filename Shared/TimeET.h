#ifndef __TIME_ET_CLASS__
#define __TIME_ET_CLASS__

#ifdef PLATFORM_APERIOS
#include <MCOOP.h>
#endif
#include <iostream>
#include <sys/time.h>

//!a nice class for handling time values with high precision
/*@test negative times might not be handled properly */
class TimeET {
	//! lets the class be displayed easily
	friend std::ostream& operator<<(std::ostream& o, const TimeET& t);
	
public:
	//@{
	//!constructor
	TimeET() : tv() {
		Set();
	}
	TimeET(long ms) : tv() {
		Set(ms);
	}
	TimeET(time_t sec, long usec) : tv() {
		Set(sec,usec);
	}
	TimeET(const timeval& tval) : tv(tval) {}
	TimeET(const timespec& tspec) : tv() {
		Set(tspec.tv_sec,tspec.tv_nsec/ns_per_us);
	}
	//!constructor, sepecify @a t seconds
	TimeET(double t) :tv() {
		Set(t);
	}
	//@}
	
	//!returns the difference between the current time and the time stored
	inline TimeET Age() const { return TimeET()-(*this); }
	//@{
	//!returns the time stored as seconds in a double
	inline double Value() const { return (double)tv.tv_sec+(double)tv.tv_usec/(double)us_per_sec; }
	//! returns the time as a timeval system construct
	inline operator timeval &() { return tv; }
	//! returns the time as a const timeval system construct
	inline operator const timeval &() const { return tv; }
	//! returns the time as a timespec system construct (though the nanosecond resolution isn't actually retained)
	inline operator timespec() {
		timespec tspec={tv.tv_sec,tv.tv_usec*ns_per_us};
		return tspec;
	}
	//! returns the seconds portion (not rounded)
	inline time_t getSeconds() const { return tv.tv_sec; }
	//! returns the millisecond representation (includes both seconds and microseconds contribution); pass 0 to round down, 1000 to round up, 500 to round nearest
	inline long getMilliseconds(long round=us_per_ms/2) const { return tv.tv_sec*ms_per_sec + (tv.tv_usec+round)/us_per_ms; }
	//! returns the microseconds portion (doesn't include seconds)
	inline long getMicroPortion() const { return tv.tv_usec; }
	//@}
	
	//@{
	//!sets the time stored in the class in terms of milliseconds
	inline void Set(long ms) {
		Set(0,ms*us_per_ms);
	}
	//!sets the time in terms of seconds and microseconds (aka timeval)
	inline void Set(time_t sec, long usec) {
		tv.tv_sec=sec+usec/us_per_sec;
		tv.tv_usec=usec%us_per_sec;;
	}
	//!sets the time in terms of floating-point seconds
	inline void Set(double t) {
		tv.tv_sec=(long)t;
		tv.tv_usec=(long)((t-tv.tv_sec)*us_per_sec);
	}
	/*!@brief sets the time to the current time
	 * @todo not getting timeofday on OPEN-R, is time since boot instead...*/
	inline void Set() {
#ifdef PLATFORM_APERIOS
		static struct SystemTime t;
		GetSystemTime(&t);
		Set(t.seconds,t.useconds);
#else
		gettimeofday(&tv,&tz);
#endif
	}
	//@}
	
	//@{
	//!for comparing times
	inline bool operator<(long ms) const { //what if ms is negative?
		time_t sec = ms/ms_per_sec;
		return tv.tv_sec<sec || (tv.tv_sec==sec && tv.tv_usec<static_cast<long>((ms-sec*ms_per_sec)*us_per_ms));
	}
	inline bool operator<(double t) const {
		return Value()<t;
	}
	inline bool operator<(const TimeET& t) const {
	  return tv.tv_sec<t.tv.tv_sec || (tv.tv_sec==t.tv.tv_sec && tv.tv_usec<t.tv.tv_usec);
	}
	//@}
	
	//@{
	//!for doing doing math with time
	inline TimeET operator+(const TimeET& t) const {
		long usec = tv.tv_usec+t.tv.tv_usec;
		long sec = tv.tv_sec+t.tv.tv_sec+usec/us_per_sec;
		usec%=us_per_sec;
		return TimeET(sec,usec);
	}
	inline TimeET& operator+=(const TimeET& t) {
		tv.tv_usec+=t.tv.tv_usec;
		tv.tv_sec+=t.tv.tv_sec+tv.tv_usec/us_per_sec;
		tv.tv_usec%=us_per_sec;
		return *this;
	}
	inline TimeET operator-(const TimeET& t) const {
		long usec = tv.tv_usec-t.tv.tv_usec;
		long sec = tv.tv_sec-t.tv.tv_sec+usec/us_per_sec;
		usec%=us_per_sec;
		if(usec<0) {
			usec+=us_per_sec;
			sec--;
		}
		return TimeET(sec,usec);
	}
	inline TimeET& operator-=(const TimeET& t) {
		tv.tv_usec-=t.tv.tv_usec;
		tv.tv_sec=tv.tv_sec-t.tv.tv_sec+tv.tv_usec/us_per_sec;
		tv.tv_usec%=us_per_sec;
		if(tv.tv_usec<0) {
			tv.tv_usec+=us_per_sec;
			tv.tv_sec--;
		}
		return *this;
	}
	inline TimeET operator*(double x) const {
		if(x>1 || x<-1) {
			double usec = tv.tv_usec*x;
			long carry=static_cast<long>(usec/us_per_sec);
			long sec = static_cast<long>(tv.tv_sec*x)+carry;
			usec-=carry*us_per_sec;
			return TimeET(sec,static_cast<long>(usec));
		} else {
			double secv=tv.tv_sec*x;
			long sec=static_cast<long>(secv);
			double carry=secv-sec;
			long usec=static_cast<long>(tv.tv_usec*x+carry*us_per_sec);
			return TimeET(sec,usec);
		}
	}
	inline TimeET& operator*=(double x) {
		if(x>1 || x<-1) {
			double usec = tv.tv_usec*x;
			long carry=static_cast<long>(usec/us_per_sec);
			tv.tv_sec = static_cast<long>(tv.tv_sec*x)+carry;
			tv.tv_usec=static_cast<long>(usec-carry*us_per_sec);
			return *this;
		} else {
			double secv=tv.tv_sec*x;
			tv.tv_sec=static_cast<long>(secv);
			double carry=secv-tv.tv_sec;
			tv.tv_usec=static_cast<long>(tv.tv_usec*x+carry*us_per_sec);
			return *this;
		}
	}
	inline TimeET operator/(double x) const {
		if(x>1 || x<-1) {
			double secv=tv.tv_sec/x;
			long sec=static_cast<long>(secv);
			double carry=secv-sec;
			long usec=static_cast<long>(tv.tv_usec/x+carry*us_per_sec);
			return TimeET(sec,usec);
		} else {
			double usec = tv.tv_usec/x;
			long carry=static_cast<long>(usec/us_per_sec);
			long sec = static_cast<long>(tv.tv_sec/x)+carry;
			usec-=carry*us_per_sec;
			return TimeET(sec,static_cast<long>(usec));
		}
	}
	inline TimeET& operator/=(double x) {
		if(x>1 || x<-1) {
			double secv=tv.tv_sec/x;
			tv.tv_sec=static_cast<long>(secv);
			double carry=secv-tv.tv_sec;
			tv.tv_usec=static_cast<long>(tv.tv_usec/x+carry*us_per_sec);
			return *this;
		} else {
			double usec = tv.tv_usec/x;
			long carry=static_cast<long>(usec/us_per_sec);
			tv.tv_sec = static_cast<long>(tv.tv_sec/x)+carry;
			tv.tv_usec=static_cast<long>(usec-carry*us_per_sec);
			return *this;
		}
	}
	//@}
	
	static const long us_per_sec=1000000; //!< conversion factor for microseconds to seconds
	static const long ms_per_sec=1000;    //!< conversion factor for milliseconds to seconds
	static const long us_per_ms=1000;     //!< conversion factor for microseconds to milliseconds
	static const long ns_per_us=1000;     //!< conversion factor for nanoseconds to microseconds
	
protected:
	timeval tv; //!< stores the time
	static struct timezone tz; //!< stores the timezone (not really used)
};

//@{
//!for doing doing math with time
inline TimeET operator+(long t1, const TimeET& t2) { return TimeET(t1)+t2; }
inline TimeET operator-(long t1, const TimeET& t2) { return TimeET(t1)-t2; }
inline TimeET operator+(double t1, const TimeET& t2) { return TimeET(t1)+t2; }
inline TimeET operator-(double t1, const TimeET& t2) { return TimeET(t1)-t2; }
inline TimeET operator*(double x, const TimeET& t) { return t*x; }
//@}

//! displays the value as text: secs~usecs
inline std::ostream& operator<<(std::ostream& o, const TimeET& t) {
	o << t.tv.tv_sec << '.';
	o.width(6);
	o.fill('0');
	o << t.tv.tv_usec;
	o.fill(' ');
	return o;
}

/*! @file
* @brief Describes TimeET, a nice class for handling time values with high precision
* @author ejt (Creator)
*/

#endif
