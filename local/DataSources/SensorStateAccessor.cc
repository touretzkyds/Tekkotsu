#include "SensorStateAccessor.h"
#include "Shared/plist.h"
#include "Shared/RobotInfo.h"
#include <sstream>
#include <string>

using namespace std; 
using namespace plist;

static const string BUTTON_PREFIX = "WorldState.Buttons.";
static const string SENSOR_PREFIX = "WorldState.Sensors.";

bool SensorStateAccessor::set(const std::string& key, const std::string& value) const {
	float val;
	if(!(stringstream(value) >> val)) {
		cerr << "Bad value '" << value << "', need numeric" << endl;
		return false;
	}
	
	if(key.substr(0,BUTTON_PREFIX.size())==BUTTON_PREFIX) {
		string entry = key.substr(BUTTON_PREFIX.size());
		for(unsigned int i=0; i<NumButtons; ++i) {
			if(entry==buttonNames[i]) {
				setButtonValue(i, val);
				return true;
			}
		}
		unsigned int i;
		if(stringstream(entry) >> i) {
			setButtonValue(i, val);
			return true;
		}
		cerr << "Unknown button '" << entry << "', could not set" << endl;
		return false;
		
	} else if(key.substr(0,SENSOR_PREFIX.size())==SENSOR_PREFIX) {
		string entry = key.substr(SENSOR_PREFIX.size());
		for(unsigned int i=0; i<NumSensors; ++i) {
			if(entry==sensorNames[i]) {
				setSensorValue(i, val);
				return true;
			}
		}
		unsigned int i;
		if(stringstream(entry) >> i) {
			setSensorValue(i, val);
			return true;
		}
		cerr << "Unknown sensor '" << entry << "', could not set" << endl;
		return false;
		
	} else {
		cerr << "Unknown WorldState entry '" << key << "'" << endl;
		return false;
		
	}
}

bool SensorStateAccessor::print(const std::string& key) const {
	bool printButtons=true, printSensors=true;
	
	if(key.substr(0,BUTTON_PREFIX.size()-1)==BUTTON_PREFIX.substr(0,BUTTON_PREFIX.size()-1)) {
		if(key.size()<=BUTTON_PREFIX.size()) {
			printSensors=false;
		} else {
			string entry = key.substr(BUTTON_PREFIX.size());
			for(unsigned int i=0; i<NumButtons; ++i) {
				if(entry==buttonNames[i]) {
					cout << getButtonValue(i) << endl;
					return true;
				}
			}
			unsigned int i;
			if(stringstream(entry) >> i) {
				cout << getButtonValue(i) << endl;
				return true;
			}
			cerr << "Unknown button '" << entry << "'" << endl;
			return false;
		}
		
	} else if(key.substr(0,SENSOR_PREFIX.size()-1)==SENSOR_PREFIX.substr(0,SENSOR_PREFIX.size()-1)) {
		if(key.size()<=SENSOR_PREFIX.size()) {
			printButtons=false;
		} else {
			string entry = key.substr(SENSOR_PREFIX.size());
			for(unsigned int i=0; i<NumSensors; ++i) {
				if(entry==sensorNames[i]) {
					cout << getSensorValue(i) << endl;
					return true;
				}
			}
			unsigned int i;
			if(stringstream(entry) >> i) {
				cout << getSensorValue(i) << endl;
				return true;
			}
			cerr << "Unknown sensor '" << entry << "', could not set" << endl;
			return false;
		}
	}
	
	if(printButtons) {
		Dictionary b;
		for(unsigned int i=0; i<NumButtons; ++i) {
			stringstream ss;
			ss << setw(2) << i << '/' << buttonNames[i];
			b.addValue(ss.str(), getButtonValue(i));
		}
		Dictionary r;
		r.addEntry("Buttons", b);
		if(printSensors)
			plist::filteredDisplay(cout,r,"^[^.].*",REG_EXTENDED,3);
		else
			plist::filteredDisplay(cout,b,"^[^.].*",REG_EXTENDED,3);
	}
	if(printButtons && printSensors)
		cout << '\n';
	if(printSensors) {
		Dictionary s;
		for(unsigned int i=0; i<NumSensors; ++i) {
			stringstream ss;
			ss << setw(2) << i << '/' << sensorNames[i];
			s.addValue(ss.str(), getSensorValue(i));
		}
		Dictionary r;
		r.addEntry("Sensors", s);
		if(printButtons)
			plist::filteredDisplay(cout,r,"^[^.].*",REG_EXTENDED,3);
		else
			plist::filteredDisplay(cout,s,"^[^.].*",REG_EXTENDED,3);
	}
	cout << flush;
	return true;
}


/*! @file
 * @brief Implements SensorStateAccessor, not a real DataSource, but an interface for the HAL command line to support set/print of values
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
