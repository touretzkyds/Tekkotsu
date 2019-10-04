//-*-c++-*-
#ifndef INCLUDED_PListSupport_h_
#define INCLUDED_PListSupport_h_

#include "plistBase.h"
#include "plistPrimitives.h"
#include "plistCollections.h"
//#include "plistSpecialty.h"

/*! The plist namespace provides convenient serialization for persistent storage, 
 *  dynamic introspection, and polymorphic primitive values.  The classes happen
 *  to use Apple's XML-based "property list" format for serialization, using libxml2
 *  for the low level parsing.
 *
 *  The choice of storage format was based on a desire for human readability,
 *  including per-value comments and inline documentation, as well as the desire
 *  to make use of syntax hilighting or high-level editing tools.
 *  
 *  Individual values are based on templated instances of the Primitive<T> class,
 *  while groups of values are based on the Collection interface.  Currently, the
 *  concrete collections available are Array (one dimensional ordered list of mixed
 *  primitive types), ArrayOf<T> (ordered list of a single type), or Dictionary (unordered
 *  mapping from string names to mixed primitives).
 *
 *  Generally, there are two ways you might use the functionality contained within
 *  this namespace.  One is as an external storage interface, such as the STL
 *  vector and map classes are used.  This might be only a intermediary instance
 *  for conversion to and from existing data structures, as a higher-level interface
 *  than directly accessing libxml.
 *
 *  However, to fully benefit from this namespace's functionality, you will generally
 *  want your classes to inherit from plist::Dictionary, define your members
 *  as public plist Primitives and Collections, and register listeners for notification
 *  when these values are modified (either by loading from file or programmatic
 *  modification.)
 *
 *  Example usage is shown below, you can find this code and more in Tekkotsu/tools/test/plist/.
 *  @include plist-example.cc
 */
namespace plist {
	
	//! From the name of @a node, will instantiate a new ObjectBase subclass to load it
	/*! The mapping from node names to actual instantiated types is: 
	 *    - 'array' -> Vector
	 *    - 'dict' -> Map
	 *    - 'real' -> Primitive<double>
	 *    - 'integer' -> Primitive<long>
	 *    - 'string' -> Primitive<std::string>
	 *    - 'true', 'false' -> Primitive<bool>
	 *  
	 *  If successful, returns a pointer to a newly allocated region, which the caller is
	 *  responsible for freeing.  If an error occurs, NULL is returned.
	 */
	ObjectBase* loadXML(xmlNode* node);
	
	//! Attempts to parse the file found at @a file, using plist::loadXML()
	ObjectBase* loadFile(const std::string& file);
	
	//! Attempts to parse the buffer found at @a buf, using plist::loadXML()
	ObjectBase* loadBuffer(const char* buf, unsigned int len);
	
} //namespace plist

/*! @file
 * @brief Provides the plist namespace, a collection of classes to implement the Propery List data storage format, a XML standard used by Apple and others
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
