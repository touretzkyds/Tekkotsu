//-*-c++-*-
#ifndef INCLUDED_Factories_h_
#define INCLUDED_Factories_h_

//! A factory interface which doesn't actually allow anything to be produced (for completeness, like 'NULL')
template<typename=void>
struct FactoryInvalidT {
	//! concrete class for not doing anything
	template<class U> struct Factory : public FactoryInvalidT<U> {};
};
//! specialization: all invalid factories are the same...
typedef FactoryInvalidT<> FactoryInvalid;

// Alternate implementation of FactoryInvalid just for reference:
// (I prefer the one used above though, because it provides a (unused) template for the base class like the other factory types)
#if 0
//! A factory interface which doesn't actually allow anything to be produced (for completeness, like 'NULL')
struct FactoryInvalid {
	//! concrete class for not doing anything (prototype here, "implementation" follows)
	template<class U> struct Factory;
};
//! implementation of concrete class for not doing anything
template<class U> struct FactoryInvalid::Factory : public FactoryInvalid {};
#endif


//! Untyped base class for factories which don't require any arguments to construct their product, see Factories.h file notes (see Factory0Arg)
/*! Note that this does not mean the product has to be constructed without arguments... see Factory0Arg1Static for example. */
struct Factory0ArgBase {
	//! functor interface for constructing the product
	virtual void* construct()=0; 
	//! explicit destructor to avoid warnings regarding virtual functions without virtual destructor
	virtual ~Factory0ArgBase() {}
};
//! Typed base class for factories which don't require any arguments to construct their product, see Factories.h file notes
/*! Note that this does not mean the product has to be constructed without arguments... see Factory0Arg1Static for example. */
template<class Base>
struct Factory0Arg : public Factory0ArgBase {
	//! functor interface for constructing the product
	virtual Base* operator()()=0; 
	virtual void* construct() { return operator()(); }
	//! concrete class for constructing products of a specified subtype of Base
	template<class T> struct Factory : public Factory0Arg<Base> {
		virtual Base* operator()() { return new T; } //!< functor for constructing the product
	};
};
//! This class produces objects based on a constant specified as a template parameter
/*! You could also implement this by adding a member field to the factory and passing that to
 *  the target constuctor instead of the non-type template parameter (which has some restrictions).
 *  See Factory0Arg1Member */
template<class Base, class S1, S1 s1>
struct Factory0Arg1Static : public Factory0Arg<Base> {
	//! concrete class for constructing products of a specified subtype of Base
	template<class T> struct Factory : public Factory0Arg1Static<Base,S1,s1> {
		virtual Base* operator()() { return new T(s1); } //!< functor for constructing the product
	};
};
//! This class produces objects based on a constant specified as a factory member
/*! You could also implement this by using a non-type template parameter argument, see Factory0Arg1Static. */
template<class Base, class M1>
struct Factory0Arg1Member : public Factory0Arg<Base> {
	//! concrete class for constructing products of a specified subtype of Base
	template<class T> struct Factory : public Factory0Arg1Member<Base,M1> {
		Factory() : m1() {} //!< use default constructor for #m1
		explicit Factory(const M1& im1) : m1(im1) {} //!< use specified value to initalize #m1
		M1 m1; //!< storage for the product's constructor value
		virtual Base* operator()() { return new T(m1); } //!< functor for constructing the product
	};
};
//! This class produces objects based on constant values specified as a template parameters
/*! You could also implement this by adding member fields to the factory and passing that to
 *  the target constuctor instead of the non-type template parameter (which has some restrictions).
 *  See Factory0Arg1Member for an example how this is done. */
template<class Base, class S1, S1 s1, class S2, S2 s2>
struct Factory0Arg2Static : public Factory0Arg<Base> {
	//! concrete class for constructing products of a specified subtype of Base
	template<class T> struct Factory : public Factory0Arg2Static<Base,S1,s1,S2,s2> {
		virtual Base* operator()() { return new T(s1,s2); } //!< functor for constructing the product
	};
};
//! A factory for singleton classes which return the instance via getInstance()
template<class T>
class SingletonFactory : public Factory0Arg<T> {
public:
	//! returns the singleton instance, factory-style
	virtual T* operator()() { return &T::getInstance(); }
};



//! Untyped base class for factories which require a single argument to construct their product, which is passed to the concrete Factory's functor (see Factory1Arg)
/*! Note that this does not mean the product can only be constructed with only one argument... see Factory1Arg1Static for example. */
template<class A1>
struct Factory1ArgBase {
	//! functor interface for constructing the product
	virtual void* construct(const A1& a1)=0; 
	//! explicit destructor to avoid warnings regarding virtual functions without virtual destructor
	virtual ~Factory1ArgBase() {}
};
//! Typed base class for factories which require a single argument to construct their product, which is passed to the concrete Factory's functor
/*! Note that this does not mean the product can only be constructed with only one argument... see Factory1Arg1Static for example. */
template<class Base, class A1>
struct Factory1Arg : public Factory1ArgBase<A1> {
	//! functor interface for constructing the product
	virtual Base* operator()(const A1& a1)=0;
	virtual void* construct(const A1& a1) { return operator()(a1); }
	//! concrete class for constructing products of a specified subtype of Base
	template<class T> struct Factory : public Factory1Arg<Base,A1> {
		virtual Base* operator()(const A1& a1) { return new T(a1); } //!< functor for constructing the product
	};
};
//! This class produces objects based on a functor argument and a constant specified as a template parameter
/*! You could also implement this by adding a member field to the factory and passing that to
 *  the target constuctor instead of the non-type template parameter (which has some restrictions).
 *  See Factory0Arg1Member for an example how this is done. */
template<class Base, class A1, class S1, S1 s1>
struct Factory1Arg1Static : public Factory1Arg<Base,A1> {
	//! concrete class for constructing products of a specified subtype of Base
	template<class T> struct Factory : public Factory1Arg1Static<Base,A1,S1,s1> {
		virtual Base* operator()(const A1& a1) { return new T(a1,s1); } //!< functor for constructing the product
	};
};
//! This class produces objects based on a constant specified as a template parameter and a constructor argument
/*! You could also implement this by adding a member field to the factory and passing that to
 *  the target constuctor instead of the non-type template parameter (which has some restrictions).
 *  See Factory0Arg1Member for an example how this is done. */
template<class Base, class S1, S1 s1, class A1>
struct Factory1Static1Arg : public Factory1Arg<Base,A1> {
	//! concrete class for constructing products of a specified subtype of Base
	template<class T> struct Factory : public Factory1Static1Arg<Base,S1,s1,A1> {
		virtual Base* operator()(const A1& a1) { return new T(s1,a1); } //!< functor for constructing the product
	};
};



//! Base class for factories which can create products more than one way, in this case via default constructor (0 arguments) or 1-argument constructor
/*! We use multiple inheritance to combine the Factory0Arg and Factory1Arg base classes.  Keep in mind however,
 *  that all product subtypes must support all constr*/
template<class Base, class A1>
struct Factory0_1Arg : public virtual Factory0Arg<Base>, public virtual Factory1Arg<Base,A1> {
	using Factory0Arg<Base>::operator();
	using Factory1Arg<Base,A1>::operator();
	//! concrete class for constructing products of a specified subtype of Base
	template<class T> struct Factory : public Factory0_1Arg<Base,A1> {
		virtual Base* operator()() { return new T; } //!< 0 argument functor for constructing the product
		virtual Base* operator()(const A1& a1) { return new T(a1); } //!< 1 argument functor for constructing the product
	};
};
//! Variant of Factory0_1Arg, this uses a constant non-type template parameter to specify a default value to use when the 0-argument functor is called...
/*! Thus, we only ever call the product's 1 argument constructor, but provide more options in the factory interface... */
template<class Base, class S1, S1 s1, class A1>
struct Factory1Static_1Arg : public Factory0_1Arg<Base,A1> {
	//! concrete class for constructing products of a specified subtype of Base
	template<class T> struct Factory : public Factory1Static_1Arg<Base,S1,s1,A1> {
		virtual Base* operator()() { return new T(s1); } //!< 0 argument functor for constructing the product using the static value
		virtual Base* operator()(const A1& a1) { return new T(a1); } //!< 1 argument functor for constructing the product
	};
};

/*! @file
 * @brief Defines a variety of factory templates which can be used with FamilyFactory.
 * This is not (and couldn't be) an exhaustive list of Factory configurations.  It provides some commonly used
 * configurations, and should give you some examples and ideas how to create your own for more complex cases.
 *
 * Factory0Arg and Factory1Arg allow you to instantiate classes with 0 or 1 arguments respectively.  Subclasses of
 * these interfaces allow you to specify additional parameters to the product's constructor call.  Factory0_1Arg
 * shows how to combine Factory configurations so you can have multiple constructor options at runtime.
 *
 * @author ejt (Creator)
 */

#endif
