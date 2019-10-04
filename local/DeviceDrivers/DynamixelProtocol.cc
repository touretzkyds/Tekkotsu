#include "DynamixelProtocol.h"
#include "Events/EventRouter.h"
#include "IPC/FailsafeThread.h"
#include "Shared/RobotInfo.h"
#include "Shared/debuget.h"
#include <iostream>

using namespace std;

namespace DynamixelProtocol {
	const char* MODEL_UNKNOWN_NAME="UNKNOWN";
	
	const size_t NUM_KNOWN_MODELS=14;
	std::pair<DynamixelProtocol::ModelID_t, const std::string> knownModels[NUM_KNOWN_MODELS] = {
		std::make_pair(MODEL_DX113, "DX-113"),
		std::make_pair(MODEL_DX116, "DX-116"),
		std::make_pair(MODEL_DX117, "DX-117"),
		std::make_pair(MODEL_AX12, "AX-12"),
		std::make_pair(MODEL_AX18, "AX-18"),
		std::make_pair(MODEL_AXS1, "AX-S1"),
		std::make_pair(MODEL_RX10, "RX-10"),
		std::make_pair(MODEL_RX24, "RX-24"),
		std::make_pair(MODEL_RX28, "RX-28"),
		std::make_pair(MODEL_RX64, "RX-64"),
		std::make_pair(MODEL_MX28, "MX-28"),
		std::make_pair(MODEL_MX64, "MX-64"),
		std::make_pair(MODEL_MX106, "MX-106"),
		std::make_pair(MODEL_EX106P, "EX-106+"),
	};
	const std::map<DynamixelProtocol::ModelID_t, const std::string> dynamixelModels(knownModels,&knownModels[NUM_KNOWN_MODELS]);

	const char* ResponseErrorNames[9] = {
		"VOLTAGE_ERROR",
		"ANGLE_ERROR",
		"HEAT_ERROR",
		"RANGE_ERROR",
		"CHECKSUM_ERROR",
		"LOAD_ERROR",
		"INSTRUCTION_ERROR",
		"UNKNOWN_ERROR",
		NULL
	};

	void reportErrors(unsigned int servoID, unsigned int offset, unsigned char err) {
		if(err==0)
			return;
		if ( err & ~(HEAT_ERROR | LOAD_ERROR) ) { // dont' report load or heat errors here; ResetServos will handle those
			std::cerr << "Dynamixel module " << servoID;
			if(offset<NumOutputs)
				std::cerr << " (" << outputNames[offset] << ")";
			std::cerr << " reported error " << (int)err << ":";
			for(unsigned int i=0; i<sizeof(err)*8; ++i)
				if( (err>>i) & 1 )
					std::cerr << ' ' << ResponseErrorNames[i];
			std::cerr << std::endl;
		}
		if(erouter!=NULL && offset<NumOutputs)
			erouter->postEvent(EventBase(EventBase::servoEGID, offset, EventBase::statusETID, 0,"Servo Error",(float)err));
	}
	
	long PingThread::timeout = 150;
	
	void * PingThread::run() {
		ocomm.write(ReadModelCmd(sid),sizeof(ReadModelCmd)).flush();
		//cout << "Ping " << (int)sid << ' ' << sizeof(response) << " bytes" << endl;
		FailsafeThread failsafe(*this,(long)timeout,true); // wait for servo response up to timeout
		if(!readResponse(icomm,response,output))
			return NULL;
		
		const ModelID_t responseModel = static_cast<ModelID_t>(response.getModelNumber());
		//cout << "Ping " << (int)sid << " response model " << responseModel << std::endl;
		
		// check AX-S1 sensors
		if (responseModel==MODEL_AXS1 && infoS1!=NULL) {
			ocomm.write(ReadAXS1SensorsCmd(sid),sizeof(ReadAXS1SensorsCmd)).flush();
			if(!ocomm) {
				std::cerr << "DynamixelDriver unable to send initial sensor poll request, lost comm?" << std::endl;
				return NULL;
			}
			AXS1SensorsResponse axS1response;
			if(readResponse(icomm,axS1response,output))
				*infoS1 = axS1response;
		// check servo sensors
		} else if(responseModel!=MODEL_UNKNOWN  && info!=NULL) {
			// it's an common servo module, get the current sensor values
			ocomm.write(ReadServoSensorsCmd(sid),sizeof(ReadServoSensorsCmd)).flush();
			if(!ocomm) {
				std::cerr << "DynamixelDriver unable to send initial sensor poll request, lost comm?" << std::endl;
				return NULL;
			}
			ServoSensorsResponse servoresponse;
			if(readResponse(icomm,servoresponse,output))
				*info = servoresponse;
		}
		
		if(response.getModelString()!=MODEL_UNKNOWN_NAME)
			return const_cast<char*>(response.getModelString());
		else {
			unknownModelName = response.getModelString();
			unknownModelName.append("=0x");
			if(response.modelh>>4)
				unknownModelName.append(1,debuget::hexdigit(response.modelh>>4));
			if(response.modelh&15)
				unknownModelName.append(1,debuget::hexdigit(response.modelh&15));
			if(response.modell>>4)
				unknownModelName.append(1,debuget::hexdigit(response.modell>>4));
			unknownModelName.append(1,debuget::hexdigit(response.modell&15));
			return const_cast<char*>(unknownModelName.c_str());
		}
	}

	void PingThread::cancelled() { icomm.clear(); }
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
