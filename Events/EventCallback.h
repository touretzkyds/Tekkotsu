//-*-c++-*-
#ifndef INCLUDED_EventCallback_h_
#define INCLUDED_EventCallback_h_

#include "Events/EventListener.h"
#include "Shared/string_util.h"
#include "Events/EventRouter.h"
#include <typeinfo>

//! Listens for a specified event and then forwards to the specified callback
/*! This allows you to avoid the switch-block style of processing for events */
template<class EV>
class EventCallbackAs : public EventListener {
public:
	//! constructor, pass zero-argument callback function
	template<typename F>
	EventCallbackAs(const F& cb) : EventListener(), fun(new FunctorInstance<F,void>(cb)) { }
	
	//! constructor, pass the user data and single argument callback function
	template<typename F, typename C>
	EventCallbackAs(const F& cb, C& userdata) : EventListener(), fun(new FunctorInstance<F,C>(cb,userdata)) { }
	
	//! constructor, pass the user data and single argument callback function
	template<typename F, typename C>
	EventCallbackAs(const F& cb, const C& userdata) : EventListener(), fun(new FunctorInstance<F,C>(cb,userdata)) { }
	
	//! constructor, pass zero-argument callback function
	template<typename F>
	void redirect(const F& cb) { delete fun; fun = new FunctorInstance<F,void>(cb); }
	
	//! constructor, pass the user data and single argument callback function
	template<typename F, typename C>
	void redirect(const F& cb, C& userdata) { delete fun; fun=new FunctorInstance<F,C>(cb,userdata); }
	
	//! constructor, pass the user data and single argument callback function
	template<typename F, typename C>
	void redirect(const F& cb, const C& userdata) { delete fun; fun=new FunctorInstance<F,C>(cb,userdata); }
	
	//! destructor
	~EventCallbackAs() { erouter->remove(this); delete fun; fun=NULL; }
	
protected:
	/*! @cond INTERNAL */
	
	//! generic base case, supplies a functor interface with no arguments
	/*! (subclasses must store the arguments passed to EventCallback constructor) */
	struct FunctorAdapter {
		virtual ~FunctorAdapter() {}
		virtual void operator()(const EV& event)=0;
	};
	
	//! generic template instance, handles std::mem_fun_t and such class based functors
	template<typename F, typename C> struct FunctorInstance : public FunctorAdapter {
		FunctorInstance(const F& cb, const C& x) : fun(cb), data(x) {}
		virtual void operator()(const EV& event) { fun(data,event); }
		F fun;
		C data;
	private:
		FunctorInstance(const FunctorInstance&); //!< don't call
		FunctorInstance& operator=(const FunctorInstance&); //!< don't call
	};
	
	//! partial specialization, handles non-const member functions
	template<typename R, class C, class SUB, class ET> struct FunctorInstance<R (C::*)(const ET&),SUB> : public FunctorAdapter {
		FunctorInstance(R (C::*cb)(const EV&), SUB& x) : fun(cb), cl(x) {}
		virtual void operator()(const EV& event) { (cl.*fun)(event); }
		R (C::*fun)(const EV& event);
		SUB& cl;
	};
	//! partial specialization, handles const member functions
	template<typename R, class C, class SUB, class ET> struct FunctorInstance<R (C::*)(const ET&) const,SUB> : public FunctorAdapter {
		FunctorInstance(R (C::*cb)(const EV&) const, const SUB& x) : fun(cb), cl(x) {}
		virtual void operator()(const EV& event) { (cl.*fun)(event); }
		R (C::*fun)(const EV& event) const;
		const SUB& cl;
	};
	//! partial specialization, handles zero-argument functors
	template<typename R, class ET> struct FunctorInstance<R (*)(const ET&),void> : public FunctorAdapter {
		FunctorInstance(R (*cb)(const EV&)) : fun(cb) {}
		virtual void operator()(const EV& event) { fun(event); }
		R (*fun)(const EV& event);
	};
	
	//! partial specialization, handles non-const member functions
	template<typename R, class C, class SUB, class ET> struct FunctorInstance<R* (C::*)(const ET&),SUB> : public FunctorAdapter {
		FunctorInstance(R* (C::*cb)(const EV&), SUB& x) : fun(cb), cl(x) {}
		virtual void operator()(const EV& event) { (cl.*fun)(event); }
		R* (C::*fun)(const EV& event);
		SUB& cl;
	};
	//! partial specialization, handles const member functions
	template<typename R, class C, class SUB, class ET> struct FunctorInstance<R* (C::*)(const ET&) const,SUB> : public FunctorAdapter {
		FunctorInstance(R* (C::*cb)(const EV&) const, const SUB& x) : fun(cb), cl(x) {}
		virtual void operator()(const EV& event) { (cl.*fun)(event); }
		R* (C::*fun)(const EV& event) const;
		const SUB& cl;
	};
	//! partial specialization, handles zero-argument member functions
	template<typename R, class ET> struct FunctorInstance<R* (*)(const ET&),void> : public FunctorAdapter {
		FunctorInstance(R* (*cb)(const EV&)) : fun(cb) {}
		virtual void operator()(const EV& event) { fun(event); }
		R* (*fun)(const EV& event);
	};
	
	/*! @endcond */
	
	virtual void processEvent(const EventBase& event) {
		const EV* ev = dynamic_cast<const EV*>(&event);
		if(ev==NULL) {
			std::cerr << "EventCallback dropping " << event.getDescription() << ", was expected type " << string_util::demangle(typeid(EV).name()) << " not " << string_util::demangle(typeid(event).name()) << std::endl;
		} else {
			(*fun)(*ev);
		}
	}
	FunctorAdapter * fun;
	
private:
	EventCallbackAs(const EventCallbackAs&); //!< don't call
	EventCallbackAs& operator=(const EventCallbackAs&); //!< don't call
};

typedef EventCallbackAs<EventBase> EventCallback;

/*! @file
 * @brief EventCallback listens for a specified event and then forwards to the specified callback
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
