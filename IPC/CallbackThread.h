//-*-c++-*-
#ifndef INCLUDED_CallbackThread_h_
#define INCLUDED_CallbackThread_h_

#include "PollThread.h"

//! A simple adapter to cause a callback under a new thread
/*! The constructor is designed to take just about anything, with zero or one arguments.
 *  If the functor returns a pointer type, this will be passed as the result of the run()
 *  call, accessible via join() or getReturnValue(). */
class CallbackThread : public Thread {
public:
	//! constructor, pass zero-argument callback function
	template<typename F>
	CallbackThread(const F& cb, bool autoStart=false) : Thread(), fun(new FunctorInstance<F,void>(cb)) { if(autoStart) start(); }
	
	//! constructor, pass the user data and single argument callback function
	template<typename F, typename C>
	CallbackThread(const F& cb, C& userdata, bool autoStart=false) : Thread(), fun(new FunctorInstance<F,C>(cb,userdata)) { if(autoStart) start(); }
	
	//! constructor, pass the user data and single argument callback function
	template<typename F, typename C>
	CallbackThread(const F& cb, const C& userdata, bool autoStart=false) : Thread(), fun(new FunctorInstance<F,C>(cb,userdata)) { if(autoStart) start(); }
	
	//! destructor
	~CallbackThread() { delete fun; fun=NULL; }
	
protected:
	/*! @cond INTERNAL */

	//! generic base case, supplies a functor interface with no arguments
	/*! (subclasses must store the arguments passed to CallbackThread constructor) */
	struct FunctorAdapter {
		virtual ~FunctorAdapter() {}
		virtual void* operator()()=0;
	};
	
	//! generic template instance, handles std::mem_fun_t and such class based functors (ignores return value)
	template<typename F, typename C> struct FunctorInstance : public FunctorAdapter {
		FunctorInstance(const F& cb, const C& x) : fun(cb), data(x) {}
		virtual void* operator()() { fun(data); return NULL; }
		F fun;
		C data;
	private:
		FunctorInstance(const FunctorInstance&); //!< don't call
		FunctorInstance& operator=(const FunctorInstance&); //!< don't call
	};
	
	//! partial specialization, handles non-const member functions (ignores return value)
	template<typename R, class C, class SUB> struct FunctorInstance<R (C::*)(),SUB> : public FunctorAdapter {
		FunctorInstance(R (C::*cb)(), SUB& x) : fun(cb), cl(x) {}
		virtual void* operator()() { (cl.*fun)(); return NULL; }
		R (C::*fun)();
		SUB& cl;
	};
	//! partial specialization, handles const member functions (ignores return value)
	template<typename R, class C, class SUB> struct FunctorInstance<R (C::*)() const,SUB> : public FunctorAdapter {
		FunctorInstance(R (C::*cb)() const, const SUB& x) : fun(cb), cl(x) {}
		virtual void* operator()() { (cl.*fun)(); return NULL; }
		R (C::*fun)() const;
		const SUB& cl;
	};
	//! partial specialization, handles zero-argument functors (ignores return value)
	template<typename R> struct FunctorInstance<R (*)(),void> : public FunctorAdapter {
		FunctorInstance(R (*cb)()) : fun(cb) {}
		virtual void* operator()() { fun(); return NULL; }
		R (*fun)();
	};

	//! partial specialization, handles non-const member functions (pass through return value)
	template<typename R, class C, class SUB> struct FunctorInstance<R* (C::*)(),SUB> : public FunctorAdapter {
		FunctorInstance(R* (C::*cb)(), SUB& x) : fun(cb), cl(x) {}
		virtual void* operator()() { return (cl.*fun)(); }
		R* (C::*fun)();
		SUB& cl;
	};
	//! partial specialization, handles const member functions (pass through return value)
	template<typename R, class C, class SUB> struct FunctorInstance<R* (C::*)() const,SUB> : public FunctorAdapter {
		FunctorInstance(R* (C::*cb)() const, const SUB& x) : fun(cb), cl(x) {}
		virtual void* operator()() { return (cl.*fun)(); }
		R* (C::*fun)() const;
		const SUB& cl;
	};
	//! partial specialization, handles zero-argument member functions (pass through return value)
	template<typename R> struct FunctorInstance<R* (*)(),void> : public FunctorAdapter {
		FunctorInstance(R* (*cb)()) : fun(cb) {}
		virtual void* operator()() { return fun(); }
		R* (*fun)();
	};
	
	/*! @endcond */

	virtual void* run() {
		testCancel();
		return (*fun)();
	}
	FunctorAdapter * fun;
	
private:
	CallbackThread(const CallbackThread&); //!< don't call
	CallbackThread& operator=(const CallbackThread&); //!< don't call
};


// A simple adapter to cause a periodic callback under a new thread
/*! Templates specify the type of user data (C) and the type of the callback function (F).
 *  F defaults to take a std::mem_fun call on C, so that it is used with a class method, but
 *  you could just as well use a standard c-style static function. */
class CallbackPollThread : public PollThread {
public:
	enum ReturnHandle_t {
		IGNORE_RETURN,
		STOP_FALSE,
		STOP_TRUE
	};
	
	// constructor, pass the user data and callback function
	template<typename F>
	CallbackPollThread(const F& cb, const TimeET& initial, const TimeET& freq, bool countPollTime, ReturnHandle_t stopCond, bool autoStart=false) 
	: PollThread(initial,freq,countPollTime), fun(NULL) {
		switch(stopCond) {
			case IGNORE_RETURN: fun=new FunctorInstance<F,void,IGNORE_RETURN>(cb); break;
			case STOP_FALSE: fun=new FunctorInstance<F,void,STOP_FALSE>(cb); break;
			case STOP_TRUE: fun=new FunctorInstance<F,void,STOP_TRUE>(cb); break;
		}
		if(autoStart)
			start();
	}
	
	// constructor, pass the user data and callback function
	template<typename F, typename C>
	CallbackPollThread(const F& cb, C& userdata, const TimeET& initial, const TimeET& freq, bool countPollTime, ReturnHandle_t stopCond, bool autoStart=false) 
	: PollThread(initial,freq,countPollTime), fun(NULL) {
		switch(stopCond) {
			case IGNORE_RETURN: fun=new FunctorInstance<F,C,IGNORE_RETURN>(cb,userdata); break;
			case STOP_FALSE: fun=new FunctorInstance<F,C,STOP_FALSE>(cb,userdata); break;
			case STOP_TRUE: fun=new FunctorInstance<F,C,STOP_TRUE>(cb,userdata); break;
		}
		if(autoStart)
			start();
	}
	
	// constructor, pass the user data and callback function
	template<typename F, typename C>
	CallbackPollThread(const F& cb, const C& userdata, const TimeET& initial, const TimeET& freq, bool countPollTime, ReturnHandle_t stopCond, bool autoStart=false) 
	: PollThread(initial,freq,countPollTime), fun(NULL) {
		switch(stopCond) {
			case IGNORE_RETURN: fun=new FunctorInstance<F,C,IGNORE_RETURN>(cb,userdata); break;
			case STOP_FALSE: fun=new FunctorInstance<F,C,STOP_FALSE>(cb,userdata); break;
			case STOP_TRUE: fun=new FunctorInstance<F,C,STOP_TRUE>(cb,userdata); break;
		}
		if(autoStart)
			start();
	}
	
	~CallbackPollThread() { delete fun; fun=NULL; }
	
	//! sets the polling frequency to @a p, if @a immediate is set then it will call interrupt() so the period takes effect on the current cycle
	void resetPeriod(const TimeET& p, bool immediate=true) { period=p; if(immediate && isRunning()) { delay=p; interrupt(); } }

protected:
	/*! @cond INTERNAL */
	
	//! generic base case, supplies a functor interface with no arguments
	/*! (subclasses must store the arguments passed to CallbackThread constructor) */
	struct FunctorAdapter {
		virtual ~FunctorAdapter() {}
		virtual bool operator()()=0;
	};
	
	//! generic template instance, handles std::mem_fun_t and such class based functors (pass through return value)
	template<typename F, typename C, ReturnHandle_t USE_RETURN> struct FunctorInstance : public FunctorAdapter {
		FunctorInstance(const F& cb, const C& x) : fun(cb), data(x) {}
		virtual bool operator()() {
#ifdef DEBUG
			if(USE_RETURN==IGNORE_RETURN) std::cout << "CallbackPollThread should be ignoring return value, but is not" << std::endl;
#endif
			return (fun(data)) ? (USE_RETURN==STOP_FALSE) : (USE_RETURN==STOP_TRUE);
		}
		F fun;
		C data;
	private:
		FunctorInstance(const FunctorInstance&); //!< don't call
		FunctorInstance& operator=(const FunctorInstance&); //!< don't call
	};
	//! partial specialization, handles non-const member functions (pass through return value)
	template<typename R, class C, class SUB, ReturnHandle_t USE_RETURN> struct FunctorInstance<R (C::*)(),SUB,USE_RETURN> : public FunctorAdapter {
		FunctorInstance(R (C::*cb)(), SUB& x) : fun(cb), cl(x) {}
		virtual bool operator()() {
#ifdef DEBUG
			if(USE_RETURN==IGNORE_RETURN) std::cout << "CallbackPollThread should be ignoring return value, but is not" << std::endl;
#endif
			return ((cl.*fun)()) ? (USE_RETURN==STOP_FALSE) : (USE_RETURN==STOP_TRUE);
		}
		R (C::*fun)();
		SUB& cl;
	};
	//! partial specialization, handles const member functions (pass through return value)
	template<typename R, class C, class SUB, ReturnHandle_t USE_RETURN> struct FunctorInstance<R (C::*)() const,SUB,USE_RETURN> : public FunctorAdapter {
		FunctorInstance(R (C::*cb)() const, const SUB& x) : fun(cb), cl(x) {}
		virtual bool operator()() {
#ifdef DEBUG
			if(USE_RETURN==IGNORE_RETURN) std::cout << "CallbackPollThread should be ignoring return value, but is not" << std::endl;
#endif
			return ((cl.*fun)()) ? (USE_RETURN==STOP_FALSE) : (USE_RETURN==STOP_TRUE);
		}
		R (C::*fun)() const;
		const SUB& cl;
	};
	//! partial specialization, handles zero-argument functors (pass through return value)
	template<typename R, ReturnHandle_t USE_RETURN> struct FunctorInstance<R (*)(),void,USE_RETURN> : public FunctorAdapter {
		FunctorInstance(R (*cb)()) : fun(cb) {}
		virtual bool operator()() {
#ifdef DEBUG
			if(USE_RETURN==IGNORE_RETURN) std::cout << "CallbackPollThread should be ignoring return value, but is not" << std::endl;
#endif
			return (fun()) ? (USE_RETURN==STOP_FALSE) : (USE_RETURN==STOP_TRUE);
		}
		R (*fun)();
	};
	
	//! partial specialization, handles std::mem_fun_t and such class based functors (ignores return value)
	template<typename F, typename C> struct FunctorInstance<F,C,IGNORE_RETURN> : public FunctorAdapter {
		FunctorInstance(const F& cb, const C& x) : fun(cb), data(x) {}
		virtual bool operator()() { fun(data); return true; }
		F fun;
		C data;
	private:
		FunctorInstance(const FunctorInstance&); //!< don't call
		FunctorInstance& operator=(const FunctorInstance&); //!< don't call
	};
	//! partial specialization, handles non-const member functions (ignores return value)
	template<typename R, class C, class SUB> struct FunctorInstance<R (C::*)(),SUB,IGNORE_RETURN> : public FunctorAdapter {
		FunctorInstance(R (C::*cb)(), SUB& x) : fun(cb), cl(x) {}
		virtual bool operator()() { (cl.*fun)(); return true; }
		R (C::*fun)();
		SUB& cl;
	};
	//! partial specialization, handles const member functions (ignores return value)
	template<typename R, class C, class SUB> struct FunctorInstance<R (C::*)() const,SUB,IGNORE_RETURN> : public FunctorAdapter {
		FunctorInstance(R (C::*cb)() const, const SUB& x) : fun(cb), cl(x) {}
		virtual bool operator()() { (cl.*fun)(); return true; }
		R (C::*fun)() const;
		const SUB& cl;
	};
	//! partial specialization, handles zero-argument member functions (ignores return value)
	template<typename R> struct FunctorInstance<R (*)(),void,IGNORE_RETURN> : public FunctorAdapter {
		FunctorInstance(R (*cb)()) : fun(cb) {}
		virtual bool operator()() { fun(); return true; }
		R (*fun)();
	};
	
	/*! @endcond */
	
	virtual bool poll() { return (*fun)(); }

	FunctorAdapter* fun; //!< function to be called from within new thread

private:
	CallbackPollThread(const CallbackPollThread&); //!< don't call
	CallbackPollThread& operator=(const CallbackPollThread&); //!< don't call
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
