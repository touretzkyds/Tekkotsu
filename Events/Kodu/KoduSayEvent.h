#ifndef _INCLUDE_KoduSayEvent_h_
#define _INCLUDE_KoduSayEvent_h_

#include <string>

#include "Events/Kodu/KoduEventBase.h"

class KoduSayEvent : public KoduEventBase {
 private:
	std::string _phrase;

 public:
	static const char* classID;

	KoduSayEvent() : _phrase("") {}

	KoduSayEvent(const std::string& phrase) : _phrase(phrase) {}
	
	~KoduSayEvent() {}

	virtual std::string getDescription(bool showTypeSpecific, unsigned int verbosity) const;
	virtual unsigned int getBinSize() const;
	virtual unsigned int saveBinaryBuffer(char buf[], unsigned int len) const;
	virtual unsigned int loadBinaryBuffer(const char buf[], unsigned int len);

	std::string getPhrase() const;

};

/*! @file
 * @brief Kodu says something in the world
 *
 * @author afranchu
 * @author medee
 */

#endif
