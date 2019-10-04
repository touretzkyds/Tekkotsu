//-*-c++-*-
#ifndef INCLUDED_SensorStateAccessor_h_
#define INCLUDED_SensorStateAccessor_h_

#include "local/DataSource.h"

//! Not a real DataSource, but an interface for the HAL command line to support set/print of values
class SensorStateAccessor : public DataSource {
public:
	virtual unsigned int nextTimestamp() { return 0; }
	virtual const std::string& nextName() { static std::string empty; return empty; }
	virtual bool advance() { return false; }
	
	bool set(const std::string& key, const std::string& value) const;
	bool print(const std::string& key) const;
};

/*! @file
 * @brief Describes SensorStateAccessor, not a real DataSource, but an interface for the HAL command line to support set/print of values
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
