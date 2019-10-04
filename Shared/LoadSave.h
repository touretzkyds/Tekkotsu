//-*-c++-*-
#ifndef INCLUDED_LoadSave_h
#define INCLUDED_LoadSave_h

#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/param.h>
#include <stdexcept>
#include <stdarg.h>
#include <cstdio>
#include <iosfwd>
#include "attributes.h"

#ifdef PLATFORM_APERIOS
//! prototype declared only on PLATFORM_APERIOS; system provides an implementation, but apparently no API declaration
int vasprintf(char** ret, const char* format, va_list al);
#endif

/*! @def LOADSAVE_SWAPBYTES
 *  @brief Set to 0 on platforms which don't actually need to swap bytes, 1 otherwise.  If not set externally, will attempt to auto-detect.
 *  Swapping is performed to standardize on little-endian byte order.  Mainly because this is what we used on the Aibo, which
 *  was the most processor-starved platform.  If running embedded on other platforms, you may want to reverse the logic
 *  for determining whether byte swapping will be performed! */
#ifndef LOADSAVE_SWAPBYTES

#ifdef BYTE_ORDER
#  if BYTE_ORDER == BIG_ENDIAN
#    define LOADSAVE_SWAPBYTES 1
#  elif BYTE_ORDER == LITTLE_ENDIAN
#    define LOADSAVE_SWAPBYTES 0
#  else
#    warning unknown byte ordering for current platform, assuming no swap
#    define LOADSAVE_SWAPBYTES 0
#  endif
#else
//aperios distribution doesn't actually define BYTE_ORDER :(
//just as well, since we're using that byte order anyway
#  ifdef PLATFORM_APERIOS
//!On PLATFORM_APERIOS, we avoid swapping bytes, since it's likely to be the most CPU constrained platform
#    define LOADSAVE_SWAPBYTES 0
#  else
#    warning unknown byte ordering for current platform, assuming no swap
#    define LOADSAVE_SWAPBYTES 0
#  endif
#endif

#endif

//! Intended as an interface to allow easy and portable serialization operations
/*! Generally, for triggering serialization of a LoadSave subclass, all you need to know is to call saveFile() /
    saveBuffer() in order to have the class serialize itself, and loadFile() /
    loadBuffer() in order to reload the data.
    
    When saveFile() is called, it checks that it can open the specified file, and then calls
    saveFileStream() with the open file.  This will then check getBinSize(), create a buffer of that
    size, and call saveBuffer() to do the actual work of serialization into that buffer.  If
    saveBuffer is successful, saveFileStream() copies the buffer out to the file, and then finally,
    saveFile() will close the file.
    
    This means when writing a class which requires serialization, you need only define 3 functions:
    loadBuffer(), saveBuffer(), and getBinSize().  If you are saving directly into a file and need
    the highest possible performance, overriding loadFileStream and saveFileStream and
    reimplementing the serialization operations into the file stream can save a buffer copy.
    Usually this is not a performance issue, but the interface is there if you need it.
    
    The recommended style for using LoadSave in classes with multiple levels of inheritance is to
    have each subclass first call the superclass's implementation (e.g. of loadBuffer/saveBuffer),
    and then save their own data afterward.  This compartmentalizes the data access and makes it
    easy to maintain - the code that serializes is right in with the code that defines the
    structure.  If you change one, it's easy to see where to change the other.  And protection
    between levels of inheritance is retained.  (This is why I say it's highly
    flexible/maintainable, but poor readability since the serialization is all broken up.)
    
    I also recommend putting a little string header at the beginning of each class's info via saveCreator() and checkCreator().  This
    will allow polymorphism when loading files (you can look at the string and create the
    appropriate type) but also is handy for checking field alignment... it's a lot easier to tell
    how much you're offset within a string than to do the same with a stream of binary values.
    Further, you can use the string as version information if you want to be backward compatible in
    future versions of your code.
	
    LoadSave provides a series of encode() and decode() functions for all the primitive types.  This
    will handle copying the value into the buffer or file, and can provide platform independence
    through byte swapping if needed (there's a compiler flag you can set for platforms that have the
    opposite byte order, although this should be autodetected from the system headers).
    Most of these are pretty straightfoward - an int is just 4 bytes and so on.
	
    However, there's one caveat that I want to make sure to point out if you have to write parsing
    code in say, Java.  Strings are encoded by first storing an int to hold the string's length,
    then the string itself, and then a null character.  This adds 5 bytes to the length of any
    string, but makes loading the files much easier/faster - you can call string library functions
    directly on the buffer if it's already in memory since the string is null terminated, or can
    allocate memory to hold the string with one pass from a file because you'll know the
    size of the string before you get to it.
	
    Of course, the string serialization format is transparent if you just stick to using LoadSave's
    encode/decode functions to parse it.
*/
class LoadSave {
 public:
	//! This is the amount of extra space needed to store a string (int for len of string plus 1 for @code '\0' @endcode termination)
	static const unsigned int stringpad=sizeof(unsigned int)+1;

	//!@name Constructors/Destructors
	/*! @brief constructor */
	LoadSave() {}
	virtual ~LoadSave(); //!< destructor
	//@}

	/*! @brief These are useful for sending the data across a network as well as to a file.\n
	 *  These three functions (getBinSize(), loadBuffer(), saveBuffer() ) are the only ones that MUST be
	 *  overridden, as the file stream versions can be based on calling these.  However, you can override
	 *  the file stream versions as well if speed or temp. memory is tight. */
	//!@name Buffer Operations
	
	//! Calculates space needed to save - if you can't precisely add up the size, just make sure to overestimate and things will still work.
	/*! getBinSize is used for reserving buffers during serialization, but does not necessarily determine
	 *  the actual size of what is written -- the return value of saveBuffer() specifies that after the data
	 *  actually has been written.  If getBinSize overestimates, the extra memory allocation is only
	 *  temporary, no extra filler bytes are actually stored.
	 *  @return number of bytes read/written, 0 if error (or empty) */
	virtual unsigned int getBinSize() const =0;
	//! Load from a saved buffer in memory
	/*! @param buf pointer to the memory where you should begin loading
	 *  @param len length of @a buf available (this isn't necessarily all yours, there might be other things following your data)
	 *  @return the number of bytes actually used */
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL)=0;
	//! Save to a given buffer in memory
	/*! @param buf pointer to the memory where you should begin writing
	 *  @param len length of @a buf available.  (this isn't necessarily all yours, constrain yourself to what you returned in getBinSize() )
	 *  @return the number of bytes actually used */
	virtual unsigned int saveBuffer(char buf[], unsigned int len) const =0;
	//@}

	//!These are called to load and save to files
	//!@name File Operations
	/*!@brief initiate opening of the specified file and loading/saving of all appropriate information.
	 * @param filename the file to load/save @return number of bytes read/written, 0 if error (or empty)*/
	virtual unsigned int loadFile(const char* filename);
	virtual unsigned int saveFile(const char* filename) const;
	
	//!Used recursively on member objects once a file is already open - DON'T CLOSE the file in your overridden functions
	/*! @param f a pointer to the file to load
	 *	@return number of bytes read, 0 if error (or empty) */
	virtual unsigned int loadFileStream(FILE* f, const char* filename=NULL);
	//!Used recursively on member objects once a file is already open - DON'T CLOSE the file in your overridden functions
	/*! @param f a pointer to the file to save
	 *	@return number of bytes written, 0 if error (or empty) */
	virtual unsigned int saveFileStream(FILE* f) const;
	
	//! Writes into a std::ostream, does not flush
	virtual unsigned int saveStream(std::ostream& os) const;

	//! deprecated, use loadFile() instead (refactored to standardize capitalization style)
	virtual unsigned int LoadFile(const char* filename) ATTR_deprecated;
	//! deprecated, use saveFile() instead (refactored to standardize capitalization style)
	virtual unsigned int SaveFile(const char* filename) const ATTR_deprecated;
	//@}

	//! Handy for checking results from functions which manipulate a buffer (see also encodeInc()/decodeInc() )  If res is 0, returns false
	/*! Doesn't have to be used with encode/decode, also handy with snprintf, sscanf type operations using %n
	 *  @param res number of bytes used, or 0 if error
	 *  @param buf pointer to current position in buffer, will be incremented by @a res bytes
	 *  @param len number of bytes remaining between current place and end of buffer, will be decremented by @a res bytes
	 *  @return true if everything worked, false otherwise */
	static inline bool checkInc(int res, const char*& buf, unsigned int& len) throw();
	
	//! Handy for checking results from functions which manipulate a buffer (see also encodeInc()/decodeInc() )  If res is 0, displays the specified message on stderr and returns false.
	/*! Doesn't have to be used with encode/decode, also handy with snprintf, sscanf type operations using %n
	 *  @param res number of bytes used, or 0 if error
	 *  @param buf pointer to current position in buffer, will be incremented by @a res bytes
	 *  @param len number of bytes remaining between current place and end of buffer, will be decremented by @a res bytes
	 *  @param msg Error to display if res is less than or equal to zero
	 *  @return true if everything worked, false otherwise */
	static inline bool checkInc(int res, const char*& buf, unsigned int& len, const char* msg, ...) throw() __attribute__((format(printf,4,5)));
	
	//! Handy for checking results from functions which manipulate a buffer (see also encodeInc()/decodeInc() ).  If res is 0, returns false
	/*! Doesn't have to be used with encode/decode, also handy with snprintf, sscanf type operations using %n
	 *  @param res number of bytes used, or 0 if error
	 *  @param buf pointer to current position in buffer, will be incremented by @a res bytes
	 *  @param len number of bytes remaining between current place and end of buffer, will be decremented by @a res bytes
	 *  @return true if everything worked, false otherwise */
	static inline bool checkInc(int res, char*& buf, unsigned int& len) throw();
	
	//! Handy for checking results from functions which manipulate a buffer (see also encodeInc()/decodeInc() ).  If res is 0, displays the specified message on stderr and returns false.
	/*! Doesn't have to be used with encode/decode, also handy with snprintf, sscanf type operations using %n
	 *  @param res number of bytes used, or 0 if error
	 *  @param buf pointer to current position in buffer, will be incremented by @a res bytes
	 *  @param len number of bytes remaining between current place and end of buffer, will be decremented by @a res bytes
	 *  @param msg Error to display if res is less than or equal to zero
	 *  @return true if everything worked, false otherwise */
	static inline bool checkInc(int res, char*& buf, unsigned int& len, const char* msg, ...) throw() __attribute__((format(printf,4,5)));
	
	//! Handy for checking results from functions which manipulate a buffer (see also encodeInc()/decodeInc() )  If res is 0, throws a std::length_error with the specified message.
	/*! Doesn't have to be used with encode/decode, also handy with snprintf, sscanf type operations using %n
	 *  @param res number of bytes used, or 0 if error
	 *  @param buf pointer to current position in buffer, will be incremented by @a res bytes
	 *  @param len number of bytes remaining between current place and end of buffer, will be decremented by @a res bytes
	 *  @param msg Error message to throw in the std::length_error if res is less than or equal to zero */
	static inline void checkIncT(int res, const char*& buf, unsigned int& len, const char* msg="LoadSave::check underflow", ...) throw(std::length_error) __attribute__((format(printf,4,5)));
	
	//! Handy for checking results from functions which manipulate a buffer (see also encodeInc()/decodeInc() ).  If res is 0, throws a std::length_error with the specified message.
	/*! Doesn't have to be used with encode/decode, also handy with snprintf, sscanf type operations using %n
	 *  @param res number of bytes used, or 0 if error
	 *  @param buf pointer to current position in buffer, will be incremented by @a res bytes
	 *  @param len number of bytes remaining between current place and end of buffer, will be decremented by @a res bytes
	 *  @param msg Error message to throw in the std::length_error if res is less than or equal to zero */
	static inline void checkIncT(int res, char*& buf, unsigned int& len, const char* msg="LoadSave::check underflow", ...) throw(std::length_error) __attribute__((format(printf,4,5)));
	
	//! Encodes @a value into the buffer and if successful, increments the the buffer position and decrements the capacity.  If unsuccessful, returns false
	/*! @param value the value to encode, must be a primitive or a LoadSave subclass (i.e. a value for which encode() is defined)
	 *  @param buf pointer to current position in buffer, will be incremented by the serialized size of @a value
	 *  @param cap number of bytes remain between current place and end of buffer, will be decremented by the serialized size of @a value
	 *  @return true if everything worked, false otherwise */
	template <class T> static inline bool encodeInc(const T& value, char*& buf, unsigned int& cap) throw();

	//! Encodes @a value into the buffer and if successful, increments the the buffer position and decrements the capacity.  If unsuccessful, displays the specified message on stderr and returns false.
	/*! @param value the value to encode, must be a primitive or a LoadSave subclass (i.e. a value for which encode() is defined)
	 *  @param buf pointer to current position in buffer, will be incremented by the serialized size of @a value
	 *  @param cap number of bytes remain between current place and end of buffer, will be decremented by the serialized size of @a value
	 *  @param msg Error to display if @a buf did not have enough capacity
	 *  @return true if everything worked, false otherwise */
	template <class T> static inline bool encodeInc(const T& value, char*& buf, unsigned int& cap, const char* msg, ...) throw() __attribute__((format(printf,4,5)));

	//! Decodes @a value from the buffer and if successful, increments the the buffer position and decrements the capacity.  If unsuccessful, returns false
	/*! @param value the value to decode into, must be a primitive or a LoadSave subclass (i.e. a value for which decode() is defined)
	 *  @param buf pointer to current position in buffer, will be incremented by the serialized size of @a value
	 *  @param cap number of bytes remain between current place and end of buffer, will be decremented by the serialized size of @a value
	 *  @return true if everything worked, false otherwise */
	template <class T> static inline bool decodeInc(T& value, const char*& buf, unsigned int& cap) throw();
	
	//! Decodes @a value from the buffer and if successful, increments the the buffer position and decrements the capacity.  If unsuccessful, displays the specified message on stderr and returns false.
	/*! @param value the value to decode into, must be a primitive or a LoadSave subclass (i.e. a value for which decode() is defined)
	 *  @param buf pointer to current position in buffer, will be incremented by the serialized size of @a value
	 *  @param cap number of bytes remain between current place and end of buffer, will be decremented by the serialized size of @a value
	 *  @param msg Error to display if @a buf did not have enough capacity
	 *  @return true if everything worked, false otherwise */
	template <class T> static inline bool decodeInc(T& value, const char*& buf, unsigned int& cap, const char* msg, ...) throw() __attribute__((format(printf,4,5)));
	
	//! Decodes @a value from the buffer and if successful, increments the the buffer position and decrements the capacity.  If unsuccessful, returns false.
	/*! @param value the value to decode into, must be a primitive or a LoadSave subclass (i.e. a value for which decode() is defined)
	 *  @param buf pointer to current position in buffer, will be incremented by the serialized size of @a value
	 *  @param cap number of bytes remain between current place and end of buffer, will be decremented by the serialized size of @a value
	 *  @return true if everything worked, false otherwise */
	template <class T> static inline bool decodeInc(T& value, char*& buf, unsigned int& cap) throw();
	
	//! Decodes @a value from the buffer and if successful, increments the the buffer position and decrements the capacity.  If unsuccessful, displays the specified message on stderr and returns false.
	/*! @param value the value to decode into, must be a primitive or a LoadSave subclass (i.e. a value for which decode() is defined)
	 *  @param buf pointer to current position in buffer, will be incremented by the serialized size of @a value
	 *  @param cap number of bytes remain between current place and end of buffer, will be decremented by the serialized size of @a value
	 *  @param msg Error to display if @a buf did not have enough capacity
	 *  @return true if everything worked, false otherwise */
	template <class T> static inline bool decodeInc(T& value, char*& buf, unsigned int& cap, const char* msg, ...) throw() __attribute__((format(printf,4,5)));
	
	//! Encodes @a value into the buffer and if successful, increments the the buffer position and decrements the capacity.  If unsuccessful, throws a std::length_error with the specified message.
	/*! @param value the value to encode, must be a primitive or a LoadSave subclass (i.e. a value for which encode() is defined)
	 *  @param buf pointer to current position in buffer, will be incremented by the serialized size of @a value
	 *  @param cap number of bytes remain between current place and end of buffer, will be decremented by the serialized size of @a value
	 *  @param msg Error to display if @a buf did not have enough capacity */
	template <class T> static inline void encodeIncT(const T& value, char*& buf, unsigned int& cap, const char* msg="LoadSave::encode overflow", ...) throw(std::length_error) __attribute__((format(printf,4,5)));

	//! Decodes @a value from the buffer and if successful, increments the the buffer position and decrements the capacity.  If unsuccessful, throws a std::length_error with the specified message.
	/*! @param value the value to decode into, must be a primitive or a LoadSave subclass (i.e. a value for which decode() is defined)
	 *  @param buf pointer to current position in buffer, will be incremented by the serialized size of @a value
	 *  @param cap number of bytes remain between current place and end of buffer, will be decremented by the serialized size of @a value
	 *  @param msg Error to display if @a buf did not have enough capacity */
	template <class T> static inline void decodeIncT(T& value, const char*& buf, unsigned int& cap, const char* msg="LoadSave::decode underflow", ...) throw(std::length_error) __attribute__((format(printf,4,5)));
	
	//! Decodes @a value from the buffer and if successful, increments the the buffer position and decrements the capacity.  If unsuccessful, throws a std::length_error with the specified message.
	/*! @param value the value to decode into, must be a primitive or a LoadSave subclass (i.e. a value for which decode() is defined)
	 *  @param buf pointer to current position in buffer, will be incremented by the serialized size of @a value
	 *  @param cap number of bytes remain between current place and end of buffer, will be decremented by the serialized size of @a value
	 *  @param msg Error to display if @a buf did not have enough capacity */
	template <class T> static inline void decodeIncT(T& value, char*& buf, unsigned int& cap, const char* msg="LoadSave::decode underflow", ...) throw(std::length_error) __attribute__((format(printf,4,5)));
	
	//! deprecated, use checkInc() instead (provides less error-prone interface (NULL not allowed), mixes better with other new *Inc varients)
	static bool chkAdvance(int res, const char** buf, unsigned int* len, const char* msg, ...) ATTR_deprecated __attribute__((format(printf,4,5)));

	/*! @brief These are expected to be called from within your own getBinSize implementation in order to add up the size of all the member fields.
	 *  Use these instead of sizeof() because this allows proper handling of some oddball conditions (bool isn't 1 byte on some platforms, use strlen on char*, etc.) */
	//! @name Methods to detect the size member fields
	/*! @brief returns the serialized size of the argument */
	inline static unsigned int getSerializedSize(const LoadSave& x) throw() { return x.getBinSize(); }
	inline static unsigned int getSerializedSize(const std::string& x) throw() { return x.size()+stringpad; }
	inline static unsigned int getSerializedSize(const char* x) throw() { unsigned int sz=strlen(x); return sz+stringpad; }
	inline static unsigned int getSerializedSize(const void*) throw() { return sizeof(unsigned long long); }
	inline static unsigned int getSerializedSize(const bool&) throw() { return sizeof(char); }
	template <class T> inline static unsigned int getSerializedSize(const T& x) throw() { return sizeof(x); }
	//! this version lets you get the theoretical size of a type, but beware it will throw invalid_argument if you pass a string type! (can't tell the size of the string without an actual instance...)
	template <class T> inline static unsigned int getSerializedSize() { throw std::invalid_argument("The template argument passed to getSerializedSize() is not supported by LoadSave"); }
	//@}
	
	
	/*! @brief These are for putting creator codes (a uniquely identifying string, e.g. the name of the class) at the beginning of your data -- 
	 *  doing so is a good idea to allow polymorphism, version detection (backward compatability), or just a sanity check */
	//!@name Creator Utilities

	/*!@brief Returns size of the creator code
	 * @param creator a string to use for the creator
	 * @return the size to leave for the creator code */
	virtual unsigned int creatorSize(const char creator[]) const { return strlen(creator)+stringpad; }
	//! Compares the creator code in the buffer to the one given
	/*!@param creator what the creator should be
	 * @param buf the buffer to check
	 * @param len the size remaining in the buffer
	 * @param isLoading set this to true if you want to output a warning if it doesn't match
	 * @return the number of bytes used by the creator, or 0 if it didn't match */
	virtual unsigned int checkCreator(const char* creator, const char buf[], unsigned int len, bool isLoading=true) const throw();
	//! Compares the creator code in the buffer to the one given, increments buf and decrements len if it matches
	/*!@param creator what the creator should be
	 * @param buf the buffer to check
	 * @param len the size remaining in the buffer
	 * @param isLoading set this to true if you want to output a warning if it doesn't match
	 * @return true if it matched, false otherwise */
	virtual bool checkCreatorInc(const char* creator, const char*& buf, unsigned int& len, bool isLoading=true) const throw();
	//! Compares the creator code in the buffer to the one given, increments buf and decrements len if it matches, throws std::runtime_error if it doesn't match
	/*!@param creator what the creator should be
	 * @param buf the buffer to check
	 * @param len the size remaining in the buffer
	 * @param isLoading set this to true if you want to output a warning if it doesn't match */
	virtual void checkCreatorIncT(const char* creator, const char*& buf, unsigned int& len, bool isLoading=true) const throw(std::runtime_error);
	//! Compares the creator code in the file to the one given, will attempt to reset the file position if fails (so you can check for one of several types)
	/*!@param creator what the creator should be
	 * @param f the file pointer to check
	 * @param isLoading set this to true if you want to output a warning if it doesn't match
	 * @return the number of bytes consumed by the creator code, or 0 if it didn't match */
	virtual unsigned int checkCreator(const char* creator, FILE* f, bool isLoading=true) const throw();
	//! Saves a creator code to a buffer
	/*!@param creator the string to use for the creator code
	 * @param buf the buffer to save the code into
	 * @param len the space available in the buffer
	 * @return the number of bytes consumed */
	virtual unsigned int saveCreator(const char* creator, char buf[], unsigned int len) const throw();
	//! Saves a creator code to a buffer, increments buf and decrements len by the amount used
	/*!@param creator the string to use for the creator code
	 * @param buf the buffer to save the code into
	 * @param len the space available in the buffer
	 * @return true if successful, false otherwise */
	virtual bool saveCreatorInc(const char* creator, char*& buf, unsigned int& len) const throw();
	//! Saves a creator code to a buffer, increments buf and decrements len by the amount used
	/*!@param creator the string to use for the creator code
	 * @param buf the buffer to save the code into
	 * @param len the space available in the buffer */
	virtual void saveCreatorIncT(const char* creator, char*& buf, unsigned int& len) const throw(std::runtime_error);
	//! Saves a creator code directly to a file
	/*!@param creator the string to use for the creator code
	 * @param f the file to save the code into
	 * @return the number of bytes consumed */
	virtual unsigned int saveCreator(const char* creator, FILE* f) const throw();
	//@}

	/* //if you want to have a default behavior template like this (look up template specialization) (i thought i needed this, nevermind)
		template<class T> inline static unsigned int encode(const T x, char buf[], unsigned int cap) { cout << "*** WARNING attempted to encode non-primitive and non-LoadSave" << endl; return 0; }
		template<class T> inline static unsigned int decode(T& x, const char buf[], unsigned int cap) { cout << "*** WARNING attempted to decode non-primitive and non-LoadSave" << endl; return 0; }
		template<class T> inline static unsigned int encode(const T x, FILE* f) { cout << "*** WARNING attempted to encode non-primitive and non-LoadSave" << endl; return 0; }
		template<class T> inline static unsigned int decode(T& x, FILE* f) { cout << "*** WARNING attempted to decode non-primitive and non-LoadSave" << endl; return 0; }
	*/

	//!encode/decode cross-platform compatable (byte order consistancy)
	//!@name Encode/Decode Utils
	/*!@brief encode or decode with byte order consistency*/
	inline static unsigned int encode(const LoadSave& x, char buf[], unsigned int cap) { return x.saveBuffer(buf,cap); }
	inline static unsigned int decode(LoadSave& x, const char buf[], unsigned int cap) { return x.loadBuffer(buf,cap); }
	inline static unsigned int encode(const LoadSave& x, FILE* f)                 { return x.saveFileStream(f); }
	inline static unsigned int decode(LoadSave& x, FILE* f)                       { return x.loadFileStream(f); }
	
#if LOADSAVE_SWAPBYTES
	
	inline static unsigned int encode(const double x, char buf[], unsigned int cap) throw()         { if(cap<sizeof(x)) return 0; byteswap(*(double*)buf,x); return sizeof(x); }
	inline static unsigned int decode(double& x, const char buf[], unsigned int cap) throw()        { if(cap<sizeof(x)) return 0; byteswap(x,*(const double*)buf); return sizeof(x);}
	inline static unsigned int encode(const double x, FILE* f) throw()                         { double t=0; byteswap(t,x); return sizeof(x)*fwrite(&t,sizeof(x),1,f); }
	inline static unsigned int decode(double& x, FILE* f) throw()                              { double t=0; if(fread(&t,sizeof(x),1,f)==0) return 0; byteswap(x,t); return sizeof(x);}
	
	inline static unsigned int encode(const float x, char buf[], unsigned int cap) throw()          { if(cap<sizeof(x)) return 0; byteswap(*(float*)buf,x); return sizeof(x); }
	inline static unsigned int decode(float& x, const char buf[], unsigned int cap) throw()         { if(cap<sizeof(x)) return 0; byteswap(x,*(const float*)buf); return sizeof(x);}
	inline static unsigned int encode(const float x, FILE* f) throw()                          { float t=0; byteswap(t,x); return sizeof(x)*fwrite(&t,sizeof(x),1,f); }
	inline static unsigned int decode(float& x, FILE* f) throw()                               { float t=0; if(fread(&t,sizeof(x),1,f)==0) return 0; byteswap(x,t); return sizeof(x);}
	
	inline static unsigned int encode(const long long x, char buf[], unsigned int cap) throw()           { if(cap<sizeof(x)) return 0; byteswap(*(long long*)buf,x); return sizeof(x); }
	inline static unsigned int decode(long long& x, const char buf[], unsigned int cap) throw()          { if(cap<sizeof(x)) return 0; byteswap(x,*(const long long*)buf); return sizeof(x);}
	inline static unsigned int encode(const long long x, FILE* f) throw()                           { long long t=0; byteswap(t,x); return sizeof(x)*fwrite(&t,sizeof(x),1,f); }
	inline static unsigned int decode(long long& x, FILE* f) throw()                                { long long t=0; if(fread(&t,sizeof(x),1,f)==0) return 0; byteswap(x,t); return sizeof(x);}
	inline static unsigned int encode(const unsigned long long x, char buf[], unsigned int cap) throw()  { if(cap<sizeof(x)) return 0; byteswap(*(unsigned long long*)buf,x); return sizeof(x); }
	inline static unsigned int decode(unsigned long long& x, const char buf[], unsigned int cap) throw() { if(cap<sizeof(x)) return 0; byteswap(x,*(const unsigned long long*)buf); return sizeof(x);}
	inline static unsigned int encode(const unsigned long long x, FILE* f) throw()                  { unsigned long long t=0; byteswap(t,x); return sizeof(x)*fwrite(&t,sizeof(x),1,f); }
	inline static unsigned int decode(unsigned long long& x, FILE* f) throw()                       { unsigned long long t=0; if(fread(&t,sizeof(x),1,f)==0) return 0; byteswap(x,t); return sizeof(x);}
	
	inline static unsigned int encode(const long x, char buf[], unsigned int cap) throw()           { if(cap<sizeof(x)) return 0; byteswap(*(long*)buf,x); return sizeof(x); }
	inline static unsigned int decode(long& x, const char buf[], unsigned int cap) throw()          { if(cap<sizeof(x)) return 0; byteswap(x,*(const long*)buf); return sizeof(x);}
	inline static unsigned int encode(const long x, FILE* f) throw()                           { long t=0; byteswap(t,x); return sizeof(x)*fwrite(&t,sizeof(x),1,f); }
	inline static unsigned int decode(long& x, FILE* f) throw()                                { long t=0; if(fread(&t,sizeof(x),1,f)==0) return 0; byteswap(x,t); return sizeof(x);}
	inline static unsigned int encode(const unsigned long x, char buf[], unsigned int cap) throw()  { if(cap<sizeof(x)) return 0; byteswap(*(unsigned long*)buf,x); return sizeof(x); }
	inline static unsigned int decode(unsigned long& x, const char buf[], unsigned int cap) throw() { if(cap<sizeof(x)) return 0; byteswap(x,*(const unsigned long*)buf); return sizeof(x);}
	inline static unsigned int encode(const unsigned long x, FILE* f) throw()                  { unsigned long t=0; byteswap(t,x); return sizeof(x)*fwrite(&t,sizeof(x),1,f); }
	inline static unsigned int decode(unsigned long& x, FILE* f) throw()                       { unsigned long t=0; if(fread(&t,sizeof(x),1,f)==0) return 0; byteswap(x,t); return sizeof(x);}
	
	inline static unsigned int encode(const int x, char buf[], unsigned int cap) throw()            { if(cap<sizeof(x)) return 0; byteswap(*(int*)buf,x); return sizeof(x); }
	inline static unsigned int decode(int& x, const char buf[], unsigned int cap) throw()           { if(cap<sizeof(x)) return 0; byteswap(x,*(const int*)buf); return sizeof(x);}
	inline static unsigned int encode(const int x, FILE* f) throw()                            { int t=0; byteswap(t,x); return sizeof(x)*fwrite(&t,sizeof(x),1,f); }
	inline static unsigned int decode(int& x, FILE* f) throw()                                 { int t=0; if(fread(&t,sizeof(x),1,f)==0) return 0; byteswap(x,t); return sizeof(x);}
	inline static unsigned int encode(const unsigned int x, char buf[], unsigned int cap) throw()   { if(cap<sizeof(x)) return 0; byteswap(*(unsigned int*)buf,x); return sizeof(x); }
	inline static unsigned int decode(unsigned int& x, const char buf[], unsigned int cap) throw()  { if(cap<sizeof(x)) return 0; byteswap(x,*(const unsigned int*)buf); return sizeof(x);}
	inline static unsigned int encode(const unsigned int x, FILE* f) throw()                   { unsigned int t=0; byteswap(t,x); return sizeof(x)*fwrite(&t,sizeof(x),1,f); }
	inline static unsigned int decode(unsigned int& x, FILE* f) throw()                        { unsigned int t=0; if(fread(&t,sizeof(x),1,f)==0) return 0; byteswap(x,t); return sizeof(x);}
	
	inline static unsigned int encode(const short x, char buf[], unsigned int cap) throw()          { if(cap<sizeof(x)) return 0; byteswap(*(short*)buf,x); return sizeof(x); }
	inline static unsigned int decode(short& x, const char buf[], unsigned int cap) throw()         { if(cap<sizeof(x)) return 0; byteswap(x,*(const short*)buf); return sizeof(x);}
	inline static unsigned int encode(const short x, FILE* f) throw()                          { short t; byteswap(t,x); return sizeof(x)*fwrite(&t,sizeof(x),1,f); }
	inline static unsigned int decode(short& x, FILE* f) throw()                               { short t=0; if(fread(&t,sizeof(x),1,f)==0) return 0; byteswap(x,t); return sizeof(x);}
	inline static unsigned int encode(const unsigned short x, char buf[], unsigned int cap) throw() { if(cap<sizeof(x)) return 0; byteswap(*(unsigned short*)buf,x); return sizeof(x); }
	inline static unsigned int decode(unsigned short& x, const char buf[], unsigned int cap) throw(){ if(cap<sizeof(x)) return 0; byteswap(x,*(const unsigned short*)buf); return sizeof(x);}
	inline static unsigned int encode(const unsigned short x, FILE* f) throw()                 { unsigned short t; byteswap(t,x); return sizeof(x)*fwrite(&t,sizeof(x),1,f); }
	inline static unsigned int decode(unsigned short& x, FILE* f) throw()                      { unsigned short t=0; if(fread(&t,sizeof(x),1,f)==0) return 0; byteswap(x,t); return sizeof(x);}

#else
	
	inline static unsigned int encode(const double x, char buf[], unsigned int cap) throw()        { if(cap<sizeof(x)) return 0; memcpy(buf,&x,sizeof(x)); return sizeof(x); }
	inline static unsigned int decode(double& x, const char buf[], unsigned int cap) throw()       { if(cap<sizeof(x)) return 0; memcpy(&x,buf,sizeof(x)); return sizeof(x); }
	inline static unsigned int encode(const double x, FILE* f) throw()                        { return sizeof(x)*fwrite(&x,sizeof(x),1,f); }
	inline static unsigned int decode(double& x, FILE* f) throw()                             { return sizeof(x)*fread(&x,sizeof(x),1,f); }
	
	inline static unsigned int encode(const float x, char buf[], unsigned int cap) throw()         { if(cap<sizeof(x)) return 0; memcpy(buf,&x,sizeof(x)); return sizeof(x); }
	inline static unsigned int decode(float& x, const char buf[], unsigned int cap) throw()        { if(cap<sizeof(x)) return 0; memcpy(&x,buf,sizeof(x)); return sizeof(x); }
	inline static unsigned int encode(const float x, FILE* f) throw()                         { return sizeof(x)*fwrite(&x,sizeof(x),1,f); }
	inline static unsigned int decode(float& x, FILE* f) throw()                              { return sizeof(x)*fread(&x,sizeof(x),1,f); }

	inline static unsigned int encode(const long long x, char buf[], unsigned int cap) throw()          { if(cap<sizeof(x)) return 0; memcpy(buf,&x,sizeof(x)); return sizeof(x); }
	inline static unsigned int decode(long long& x, const char buf[], unsigned int cap) throw()         { if(cap<sizeof(x)) return 0; memcpy(&x,buf,sizeof(x)); return sizeof(x); }
	inline static unsigned int encode(const long long x, FILE* f) throw()                          { return sizeof(x)*fwrite(&x,sizeof(x),1,f); }
	inline static unsigned int decode(long long& x, FILE* f) throw()                               { return sizeof(x)*fread(&x,sizeof(x),1,f); }
	inline static unsigned int encode(const unsigned long long x, char buf[], unsigned int cap) throw() { if(cap<sizeof(x)) return 0; memcpy(buf,&x,sizeof(x)); return sizeof(x); }
	inline static unsigned int decode(unsigned long long& x, const char buf[], unsigned int cap) throw(){ if(cap<sizeof(x)) return 0; memcpy(&x,buf,sizeof(x)); return sizeof(x); }
	inline static unsigned int encode(const unsigned long long x, FILE* f) throw()                 { return sizeof(x)*fwrite(&x,sizeof(x),1,f); }
	inline static unsigned int decode(unsigned long long& x, FILE* f) throw()                      { return sizeof(x)*fread(&x,sizeof(x),1,f); }
	
	inline static unsigned int encode(const long x, char buf[], unsigned int cap) throw()          { if(cap<sizeof(x)) return 0; memcpy(buf,&x,sizeof(x)); return sizeof(x); }
	inline static unsigned int decode(long& x, const char buf[], unsigned int cap) throw()         { if(cap<sizeof(x)) return 0; memcpy(&x,buf,sizeof(x)); return sizeof(x); }
	inline static unsigned int encode(const long x, FILE* f) throw()                          { return sizeof(x)*fwrite(&x,sizeof(x),1,f); }
	inline static unsigned int decode(long& x, FILE* f) throw()                               { return sizeof(x)*fread(&x,sizeof(x),1,f); }
	inline static unsigned int encode(const unsigned long x, char buf[], unsigned int cap) throw() { if(cap<sizeof(x)) return 0; memcpy(buf,&x,sizeof(x)); return sizeof(x); }
	inline static unsigned int decode(unsigned long& x, const char buf[], unsigned int cap) throw(){ if(cap<sizeof(x)) return 0; memcpy(&x,buf,sizeof(x)); return sizeof(x); }
	inline static unsigned int encode(const unsigned long x, FILE* f) throw()                 { return sizeof(x)*fwrite(&x,sizeof(x),1,f); }
	inline static unsigned int decode(unsigned long& x, FILE* f) throw()                      { return sizeof(x)*fread(&x,sizeof(x),1,f); }
	
	inline static unsigned int encode(const int x, char buf[], unsigned int cap) throw()           { if(cap<sizeof(x)) return 0; memcpy(buf,&x,sizeof(x)); return sizeof(x); }
	inline static unsigned int decode(int& x, const char buf[], unsigned int cap) throw()          { if(cap<sizeof(x)) return 0; memcpy(&x,buf,sizeof(x)); return sizeof(x); }
	inline static unsigned int encode(const int x, FILE* f) throw()                           { return sizeof(x)*fwrite(&x,sizeof(x),1,f); }
	inline static unsigned int decode(int& x, FILE* f) throw()                                { return sizeof(x)*fread(&x,sizeof(x),1,f); }
	inline static unsigned int encode(const unsigned int x, char buf[], unsigned int cap) throw()  { if(cap<sizeof(x)) return 0; memcpy(buf,&x,sizeof(x)); return sizeof(x); }
	inline static unsigned int decode(unsigned int& x, const char buf[], unsigned int cap) throw() { if(cap<sizeof(x)) return 0; memcpy(&x,buf,sizeof(x)); return sizeof(x); }
	inline static unsigned int encode(const unsigned int x, FILE* f) throw()                  { return sizeof(x)*fwrite(&x,sizeof(x),1,f); }
	inline static unsigned int decode(unsigned int& x, FILE* f) throw()                       { return sizeof(x)*fread(&x,sizeof(x),1,f); }
	
	inline static unsigned int encode(const short x, char buf[], unsigned int cap) throw()         { if(cap<sizeof(x)) return 0; memcpy(buf,&x,sizeof(x)); return sizeof(x); }
	inline static unsigned int decode(short& x, const char buf[], unsigned int cap) throw()        { if(cap<sizeof(x)) return 0; memcpy(&x,buf,sizeof(x)); return sizeof(x); }
	inline static unsigned int encode(const short x, FILE* f) throw()                         { return sizeof(x)*fwrite(&x,sizeof(x),1,f); }
	inline static unsigned int decode(short& x, FILE* f) throw()                              { return sizeof(x)*fread(&x,sizeof(x),1,f); }
	inline static unsigned int encode(const unsigned short x, char buf[], unsigned int cap) throw(){ if(cap<sizeof(x)) return 0; memcpy(buf,&x,sizeof(x)); return sizeof(x); }
	inline static unsigned int decode(unsigned short& x, const char buf[], unsigned int cap){ if(cap<sizeof(x)) return 0; memcpy(&x,buf,sizeof(x)); return sizeof(x); }
	inline static unsigned int encode(const unsigned short x, FILE* f) throw()                { return sizeof(x)*fwrite(&x,sizeof(x),1,f); }
	inline static unsigned int decode(unsigned short& x, FILE* f) throw()                     { return sizeof(x)*fread(&x,sizeof(x),1,f); }

#endif //end of big/little endian differences
	
	/* These next two functions will allow you to serialize pointer values, but they are left out for safety -- if you're
	 * serializing a pointer, you probably meant to serialize the _data at the pointer_ not the pointer itself!
	 * If you really want to serialize the address in memory (say as an object ID of some sort), it is recommended
	 * to cast the pointer to unsigned long long (for 64 bit compatability) in your own code, and still not use these. */
	
	//inline static unsigned int encode(const void* x, char buf[], unsigned int cap) throw() { return encode(static_cast<unsigned long long>(x),buf,cap); }
	//inline static unsigned int decode(void*& x, const char buf[], unsigned int cap) throw() { unsigned long long tmp; unsigned int used=decode(tmp,buf,cap); if(used==0) return 0; x=reinterpret_cast<void*>(tmp); return used; }
	
	inline static unsigned int encode(const std::string& x, char buf[], unsigned int cap) throw() { if(cap<sizeof(unsigned int)+x.size()+1) return 0; memcpy(buf+encode(static_cast<unsigned int>(x.size()),buf,cap),x.c_str(),x.size()+1); return x.size()+stringpad; }
	inline static unsigned int decode(std::string& x, const char buf[], unsigned int cap) throw() { if(cap<sizeof(unsigned int)) return 0; unsigned int sz=0; decode(sz,buf,cap); if(cap<sizeof(unsigned int)+sz+1) return 0; x.assign(buf+sizeof(unsigned int),sz); return x.size()+stringpad; }
	inline static unsigned int encode(const std::string& x, FILE* f) throw()                 { encode(static_cast<unsigned int>(x.size()),f); return sizeof(unsigned int)+fwrite(x.c_str(),1,sizeof(x)+1,f); }
	inline static unsigned int decode(std::string& x, FILE* f) throw()                       { unsigned int sz=0; decode(sz,f); char *tmp=new char[sz+1]; if(fread(tmp,1,sz+1,f)!=sz+1) { delete [] tmp; return 0; } x.assign(tmp,sz); delete [] tmp; return sz+stringpad; }
	
	inline static unsigned int encode(const char* x, char buf[], unsigned int cap) throw()     { unsigned int sz=strlen(x); if(cap<sizeof(unsigned int)+sz+1) return 0; memcpy(buf+encode(sz,buf,cap),x,sz+1); return sz+stringpad; }
	inline static unsigned int decode(char*& x, const char buf[], unsigned int cap) throw()    { if(cap<sizeof(unsigned int)) return 0; unsigned int sz=0; decode(sz,buf,cap); if(cap<sizeof(unsigned int)+sz+1) return 0; x=new char[sz+1]; strncpy(x,buf+sizeof(unsigned int),sz+1); return sz+stringpad; }
	inline static unsigned int encode(const char* x, FILE* f) throw()                     { unsigned int sz=strlen(x); encode(sz,f); return sizeof(unsigned int)+fwrite(x,1,sz+1,f); }
	inline static unsigned int decode(char*& x, FILE* f) throw()                          { unsigned int sz=0; decode(sz,f); x=new char[sz+1]; if(fread(x,1,sz+1,f)!=sz+1) { delete [] x; x=NULL; return 0; } return sz+stringpad; }
	
	inline static unsigned int encode(const char x, char buf[], unsigned int cap) throw()         { if(cap<sizeof(x)) return 0; buf[0]=x; return sizeof(x); }
	inline static unsigned int decode(char& x, const char buf[], unsigned int cap) throw()        { if(cap<sizeof(x)) return 0; x=buf[0]; return sizeof(x);}
	inline static unsigned int encode(const char x, FILE* f) throw()                         { return sizeof(x)*fwrite(&x,sizeof(x),1,f); }
	inline static unsigned int decode(char& x, FILE* f) throw()                              { return sizeof(x)*fread(&x,sizeof(x),1,f); }
	inline static unsigned int encode(const unsigned char x, char buf[], unsigned int cap) throw(){ if(cap<sizeof(x)) return 0; buf[0]=(char)x; return sizeof(x); }
	inline static unsigned int decode(unsigned char& x, const char buf[], unsigned int cap){ if(cap<sizeof(x)) return 0; x=(unsigned char)buf[0]; return sizeof(x);}
	inline static unsigned int encode(const unsigned char x, FILE* f) throw()                { return sizeof(x)*fwrite(&x,sizeof(x),1,f); }
	inline static unsigned int decode(unsigned char& x, FILE* f) throw()                     { return sizeof(x)*fread(&x,sizeof(x),1,f); }
	
	inline static unsigned int encode(const bool x, char buf[], unsigned int cap) throw()         { if(cap<sizeof(char)) return 0; buf[0]=(char)(x?1:0); return sizeof(char); }
	inline static unsigned int decode(bool& x, const char buf[], unsigned int cap) throw()        { if(cap<sizeof(char)) return 0; x=(buf[0]!=(char)0); return sizeof(char);}
	inline static unsigned int encode(const bool x, FILE* f) throw()                         { char t=(char)(x?1:0); return sizeof(char)*fwrite(&t,sizeof(char),1,f); }
	inline static unsigned int decode(bool& x, FILE* f) throw()                              { char t='\0'; if(fread(&t,sizeof(char),1,f)!=1) return 0; x=(t!=(char)0); return sizeof(char); }
	//@}	
protected:
	//!templated function to swap byte ordering, should allow compiler to unroll the loop; warning don't use this if src==dst!!!
	template<class T> inline static void byteswap(T& dstc, const T& srcc) throw() {
		char* dst=(char*)&dstc;
		const char* src=(const char*)&srcc;
		for(unsigned int i=0; i<sizeof(T); i++)
			dst[sizeof(T)-1-i]=src[i];
	}
// It appears that aperios does have vasprintf, it's just not exposed in the header files.
// Thus, for Aperios, we provide a prototype above instead of this implementation, but it's here if needed on another platform.
#if defined(NEED_ASPRINTF)
	int vasprintf(char** ret, const char* format, va_list al) {
		va_list tmpal;
		va_copy(tmpal,al);
		int nc=vsnprintf(NULL,0,format,tmpal);
		*ret = (char*)malloc(nc+1);
		if(*ret==NULL) return -1;
		return vsnprintf(*ret,nc+1,format,al);
	}
	int asprintf(char **ret, const char *format, ...) {
		va_list ap;
		va_start (ap, format);
		int nc= vasprintf(ret, format, ap);
		va_end(ap);
		return nc;
	}
#endif
};

//hide from doxygen
#ifndef __DOXYGEN__
//! template specialization to return serialized size of the specified type
template <> inline unsigned int LoadSave::getSerializedSize<void*>() { return sizeof(unsigned long long); }
template <> inline unsigned int LoadSave::getSerializedSize<bool>() { return sizeof(char); }
template <> inline unsigned int LoadSave::getSerializedSize<char>() { return sizeof(char); }
template <> inline unsigned int LoadSave::getSerializedSize<unsigned char>() { return sizeof(unsigned char); }
template <> inline unsigned int LoadSave::getSerializedSize<short>() { return sizeof(short); }
template <> inline unsigned int LoadSave::getSerializedSize<unsigned short>() { return sizeof(unsigned short); }
template <> inline unsigned int LoadSave::getSerializedSize<int>() { return sizeof(int); }
template <> inline unsigned int LoadSave::getSerializedSize<unsigned int>() { return sizeof(unsigned int); }
template <> inline unsigned int LoadSave::getSerializedSize<long>() { return sizeof(long); }
template <> inline unsigned int LoadSave::getSerializedSize<unsigned long>() { return sizeof(unsigned long); }
template <> inline unsigned int LoadSave::getSerializedSize<long long>() { return sizeof(long long); }
template <> inline unsigned int LoadSave::getSerializedSize<unsigned long long>() { return sizeof(unsigned long long); }
template <> inline unsigned int LoadSave::getSerializedSize<float>() { return sizeof(float); }
template <> inline unsigned int LoadSave::getSerializedSize<double>() { return sizeof(double); }

template <> inline unsigned int LoadSave::getSerializedSize<char*>() { throw std::invalid_argument("Cannot pass string as template arg to getSerializedSize() -- need instance to know length!"); }
template <> inline unsigned int LoadSave::getSerializedSize<std::string>() { throw std::invalid_argument("Cannot pass string as template arg to getSerializedSize() -- need instance to know length!"); }
template <> inline unsigned int LoadSave::getSerializedSize<LoadSave>() { throw std::invalid_argument("Cannot pass LoadSave subclass as template arg to getSerializedSize() -- need instance to know length!\nIf the subclass in question has a static size, you could add a getSerializedSize() template specialization to do the size calculation (and have getBinSize return that)."); }
#endif

bool LoadSave::checkInc(int res, const char*& buf, unsigned int& len) throw() {
	if(res<0 || (unsigned int)res>len)
		return false;
	buf+=res;
	len-=res;
	return true;
}

bool LoadSave::checkInc(int res, const char*& buf, unsigned int& len, const char* msg, ...) throw() {
	if(checkInc(res,buf,len))
		return true;
	if(msg[0]!='\0') {
		if((unsigned int)res>len)
			fprintf(stderr,"*** WARNING: reported check result length exceeded remaining buffer size; %u (signed %d) vs %u\n",(unsigned int)res,res,len);
		va_list al;
		va_start(al,msg);
		vfprintf(stderr,msg,al);
		va_end(al);
	}
	return false;
}

bool LoadSave::checkInc(int res, char*& buf, unsigned int& len) throw() {
	if(res<0 || (unsigned int)res>len)
		return false;
	buf+=res;
	len-=res;
	return true;
}

bool LoadSave::checkInc(int res, char*& buf, unsigned int& len, const char* msg, ...) throw() {
	if(checkInc(res,buf,len))
		return true;
	if(msg[0]!='\0') {
		if((unsigned int)res>len)
			fprintf(stderr,"*** WARNING: reported check result length exceeded remaining buffer size; %u (signed %d) vs %u\n",(unsigned int)res,res,len);
		va_list al;
		va_start(al,msg);
		vfprintf(stderr,msg,al);
		va_end(al);
	}
	return false;
}

void LoadSave::checkIncT(int res, const char*& buf, unsigned int& len, const char* msg, ...) throw(std::length_error) {
	if(res>0 && (unsigned int)res<=len) {
		buf+=res;
		len-=res;
	} else {
		if((unsigned int)res>len && msg[0]!='\0')
			fprintf(stderr,"*** WARNING: reported check result length exceeded remaining buffer size; %u (signed %d) vs %u\n",(unsigned int)res,res,len);
		va_list al;
		va_start(al,msg);
		char * errmsg;
		int nc = vasprintf(&errmsg,msg,al);
		va_end(al);
		if(errmsg==NULL)
			throw std::length_error("unspecified");
		std::string serrmsg=errmsg;
		free(errmsg);
		if(nc<0)
			throw std::length_error("error generating error message!");
		throw std::length_error(serrmsg);
	}
}

void LoadSave::checkIncT(int res, char*& buf, unsigned int& len, const char* msg, ...) throw(std::length_error) {
	if(res>0 && (unsigned int)res<=len) {
		buf+=res;
		len-=res;
	} else {
		if((unsigned int)res>len && msg[0]!='\0')
			fprintf(stderr,"*** WARNING: reported check result length exceeded remaining buffer size; %u (signed %d) vs %u\n",(unsigned int)res,res,len);
		va_list al;
		va_start(al,msg);
		char * errmsg;
		int nc = vasprintf(&errmsg,msg,al);
		va_end(al);
		if(errmsg==NULL)
			throw std::length_error("unspecified");
		std::string serrmsg=errmsg;
		free(errmsg);
		if(nc<0)
			throw std::length_error("error generating error message!");
		throw std::length_error(serrmsg);
	}
}

template <class T>
bool LoadSave::encodeInc(const T& value, char*& buf, unsigned int& cap) throw() {
	unsigned int res=encode(value,buf,cap);
	if(res==0)
		return false;
#ifdef LOADSAVE_DEBUG
	if(res>cap) {
		fprintf(stderr,"*** WARNING: reported encode result length exceeded remaining buffer size; %u (signed %d) vs %u\n",res,(int)res,cap);
		return false;
	}
#endif
	buf+=res;
	cap-=res;
	return true;
}
template <class T>
bool LoadSave::encodeInc(const T& value, char*& buf, unsigned int& cap, const char* msg, ...) throw() {
	if(encodeInc(value,buf,cap))
		return true;
	if(msg[0]!='\0') {
		va_list al;
		va_start(al,msg);
		vfprintf(stderr,msg,al);
		va_end(al);
	}
	return false;
}
template <class T>
bool LoadSave::decodeInc(T& value, const char*& buf, unsigned int& cap) throw() {
	unsigned int res=decode(value,buf,cap);
	if(res==0)
		return false;
#ifdef LOADSAVE_DEBUG
	if(res>cap) {
		fprintf(stderr,"*** WARNING: reported decode result length exceeded remaining buffer size; %u (signed %d) vs %u\n",res,(int)res,cap);
		return false;
	}
#endif
	buf+=res;
	cap-=res;
	return true;
}
template <class T>
bool LoadSave::decodeInc(T& value, const char*& buf, unsigned int& cap, const char* msg, ...) throw() {
	if(decodeInc(value,buf,cap))
		return true;
	if(msg[0]!='\0') {
		va_list al;
		va_start(al,msg);
		vfprintf(stderr,msg,al);
		va_end(al);
	}
	return false;
}
template <class T>
bool LoadSave::decodeInc(T& value, char*& buf, unsigned int& cap) throw() {
	unsigned int res=decode(value,buf,cap);
	if(res==0)
		return false;
#ifdef LOADSAVE_DEBUG
	if(res>cap) {
		fprintf(stderr,"*** WARNING: reported decode result length exceeded remaining buffer size; %u (signed %d) vs %u\n",res,(int)res,cap);
		return false;
	}
#endif
	buf+=res;
	cap-=res;
	return true;
}
template <class T>
bool LoadSave::decodeInc(T& value, char*& buf, unsigned int& cap, const char* msg, ...) throw() {
	if(decodeInc(value,buf,cap))
		return true;
	if(msg[0]!='\0') {
		va_list al;
		va_start(al,msg);
		vfprintf(stderr,msg,al);
		va_end(al);
	}
	return false;
}

template <class T>
void LoadSave::encodeIncT(const T& value, char*& buf, unsigned int& cap, const char* msg, ...) throw(std::length_error) {
	unsigned int res=encode(value,buf,cap);
#ifdef LOADSAVE_DEBUG
	if(res==0 || res>cap) {
		if(res>cap)
			fprintf(stderr,"*** WARNING: encode reported result length exceeded remaining buffer size; %u (signed %d) vs %u\n",res,(int)res,cap);
#else
	if(res==0) {
#endif
		va_list al;
		va_start(al,msg);
		char * errmsg;
		vasprintf(&errmsg,msg,al);
		va_end(al);
		if(errmsg==NULL)
			throw std::length_error("unspecified");
		std::string serrmsg=errmsg;
		free(errmsg);
		throw std::length_error(serrmsg);
// this macro check is just to balance the {}'s for less-than-genius editors
#ifdef LOADSAVE_DEBUG
	}
#else
	}
#endif
	buf+=res;
	cap-=res;
}
template <class T>
void LoadSave::decodeIncT(T& value, const char*& buf, unsigned int& cap, const char* msg, ...) throw(std::length_error) {
	unsigned int res=decode(value,buf,cap);
#ifdef LOADSAVE_DEBUG
	if(res==0 || res>cap) {
		if(res>cap)
			fprintf(stderr,"*** WARNING: decode reported result length exceeded remaining buffer size; %u (signed %d) vs %u\n",res,(int)res,cap);
#else
	if(res==0) {
#endif
		va_list al;
		va_start(al,msg);
		char * errmsg;
		vasprintf(&errmsg,msg,al);
		va_end(al);
		if(errmsg==NULL)
			throw std::length_error("unspecified");
		std::string serrmsg=errmsg;
		free(errmsg);
		throw std::length_error(serrmsg);
// this macro check is just to balance the {}'s for less-than-genius editors
#ifdef LOADSAVE_DEBUG
	}
#else
	}
#endif
	buf+=res;
	cap-=res;
}
template <class T>
void LoadSave::decodeIncT(T& value, char*& buf, unsigned int& cap, const char* msg, ...) throw(std::length_error) {
	unsigned int res=decode(value,buf,cap);
#ifdef LOADSAVE_DEBUG
	if(res==0 || res>cap) {
		if(res>cap)
			fprintf(stderr,"*** WARNING: decode reported result length exceeded remaining buffer size; %u (signed %d) vs %u\n",res,(int)res,cap);
#else
	if(res==0) {
#endif
		va_list al;
		va_start(al,msg);
		char * errmsg;
		vasprintf(&errmsg,msg,al);
		va_end(al);
		if(errmsg==NULL)
			throw std::length_error("unspecified");
		std::string serrmsg=errmsg;
		free(errmsg);
		throw std::length_error(serrmsg);
// this macro check is just to balance the {}'s for less-than-genius editors
#ifdef LOADSAVE_DEBUG
	}
#else
	}
#endif
	buf+=res;
	cap-=res;
}

/*! @file
 * @brief Describes LoadSave, inherit from this to use a standard interface for loading and saving
 * @author ejt (Creator)
 * @author Daniel Höh (Revisor)
 */

#endif
