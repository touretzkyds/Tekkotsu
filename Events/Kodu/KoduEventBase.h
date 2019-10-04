#ifndef _INCLUDE_KoduEventBase_h_
#define _INCLUDE_KoduEventBase_h_

#include "Events/EventBase.h"

class KoduEventBase : public EventBase {
 protected:
	int _destID;

 public:
	static const char* classID;

	static KoduEventBase* getInstance(const char buf[], unsigned int size);

	KoduEventBase() : EventBase(koduEGID, (size_t)-1, statusETID), _destID(-1) {}

	~KoduEventBase() {};

	int getDestination();
	void setDestination(int destID);

	virtual std::string getDescription(bool showTypeSpecific, unsigned int verbosity) const;
	virtual unsigned int getBinSize() const;
	virtual unsigned int saveBinaryBuffer(char buf[], unsigned int len) const;
	virtual unsigned int loadBinaryBuffer(const char buf[], unsigned int len);
        
};

/*! @file
 *
 * @brief Events involved with the Kodu Interpreter
 *
 * @author afranchu
 * @author medee
 */

#endif
