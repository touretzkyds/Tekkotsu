#ifndef _INCLUDE_KoduGiveEvent_h_
#define _INCLUDE_KoduGiveEvent_h_

#include <string>

#include "Events/Kodu/KoduEventBase.h"

class KoduGiveEvent : public KoduEventBase {
 private:
    float _turn;
    std::string _objType;

 public:
	static const char* classID;

	KoduGiveEvent() : _turn(0), _objType("") {}

	~KoduGiveEvent() {}

	virtual std::string getDescription(bool showTypeSpecific, unsigned int verbosity) const;
	virtual unsigned int getBinSize() const;
	virtual unsigned int saveBinaryBuffer(char buf[], unsigned int len) const;
	virtual unsigned int loadBinaryBuffer(const char buf[], unsigned int len);

    void setTurnAndObject(float turn, std::string objType);
    float getTurn() const;
    std::string getObjType() const;
};

/*! @file
 * @brief Kodu gives something to another character
 *
 * @author afranchu
 * @author medee
 */

#endif
