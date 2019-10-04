//-*-c++-*-
#ifndef INCLUDED_Cloneable_h_
#define INCLUDED_Cloneable_h_

//! An interface for cloning objects -- needed for making copies with polymorphism (operator= doesn't work as virtual)
class Cloneable {
public:
	//!destructor
	virtual ~Cloneable() {}

	//! returns a copy of @c this
	/*! When implementing, your class @e should use its own type as the
	 *  return type, e.g.:
	 *
	 *  @code
	 *  class Foo : public Cloneable {
	 *  public:
	 *    //note: return Foo* instead of Cloneable* !
	 *    //Allows users to avoid needing to cast the results
	 *    virtual Foo* clone() { return new Foo(*this); }
	 *  };
	 *  @endcode
	 *
	 *  <i>HOWEVER, this is currently unsupported in gcc 3.3</i>, which
	 *  the latest version for which Sony has supplied the patches to
	 *  work on the Aibo.
	 *
	 *  So instead, you must currently provide the interface exactly as
	 *  shown, and then the caller will probably need to cast the result
	 *  to the known type.  Hopefully Sony will eventually update the
	 *  gcc version to at least 3.4 and we can switch over to use the
	 *  "covariant return type".
	 */

#if defined(__GNUC__) && (__GNUC__ > 3 || (__GNUC__ == 3 && (__GNUC_MINOR__ > 3)))
	virtual Cloneable* clone() const __attribute__ ((warn_unused_result)) =0;
#else
	virtual Cloneable* clone() const =0;
#endif
};

/*! @file
 * @brief Defines Cloneable, and interfacing for cloning objects -- needed for making copies with polymorphism (operator= doesn't work as virtual)
 * @author ejt (Creator)
 */

#endif
