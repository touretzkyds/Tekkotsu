#ifndef DummySocket_h_DEFINED
#define DummySocket_h_DEFINED

#include <stdarg.h>

class Wireless;

//! Tekkotsu wireless DummySocket class
/*! 
 * This class provides a DummySocket that doesn't really do anything
 * It exists to maintain code readability; while supporting dogs without
 * wireless
 * Of course, since it does nothing, it doesn't need documentation
 */

//! provides a no-op implementation of Socket for disabled operation
class DummySocket : public Socket {
  friend class Wireless;

public:
	//! constructor
  DummySocket (int sockn) : Socket (sockn) { }
	//! destructor
  virtual ~DummySocket () { }
  
	byte* getWriteBuffer(int /*bytesreq*/) { return NULL; } //!< returns NULL
  void write(int /*size*/) { } //!< no-op
  int read() { return -1; } //!< no-op (returns -1)
  byte* getReadBuffer() { return NULL; }; //!< no-op (returns NULL)
  void init() { } //!< no-op
  int setFlushType(FlushType_t /*fType*/) { return 0; } //!< no-op
  void setTextForward() { } //!< no-op
  void setForward(DummySocket * /*forsock*/) { } //!< sets another socket to send data to if this one is not connected
  void setVerbosity(int /*verbose*/) { } //!< no-op
  int write(const byte * /*buf*/, int /*size*/) { return -1; } //!< no-op (returns -1)
  int read(byte * /*buf*/, int /*size*/) { return -1; } //!< no-ip (returns -1)
  int printf(const char */*fmt*/, ...) { return 0; } //!< no-op
  int vprintf(const char */*fmt*/, va_list /*al*/) { return 0; } //!< no-op
  int pprintf(int /*vlevel*/, const char * /*fmt*/, ...) { return 0; } //!< no-op
  void flush() { } //!< no-op
};

/*! @file
 * @brief Defines Tekkotsu wireless DummySocket class
 * @author alokl (Creator)
 */

#endif
