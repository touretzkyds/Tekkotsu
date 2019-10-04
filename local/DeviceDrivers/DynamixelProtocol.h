//-*-c++-*-
#ifndef INCLUDED_DynamixelProtocol_h_
#define INCLUDED_DynamixelProtocol_h_

#include "IPC/Thread.h"
#include "Shared/debuget.h"
#include <cstddef>
#include <iostream>
#include <map>
#include <string.h>

#define IS_MXEX(m) (m==MODEL_MX28 || m==MODEL_MX64 || m==MODEL_MX106 || m==MODEL_EX106P)

//! Contains structures which define the layout of the binary communication with Dynamixel servos
namespace DynamixelProtocol {
	
	const unsigned int MAX_ID=0xFD;
	const unsigned int BROADCAST_ID=0xFE;
	const unsigned int INVALID_ID=0xFF;
	const unsigned int MARKER_VALUE=0xFF;
	
	extern const char* MODEL_UNKNOWN_NAME;
	enum ModelID_t {
		MODEL_DX113=113,
		MODEL_DX116=116,
		MODEL_DX117=117,
		MODEL_AX12=12,
		MODEL_AX18=18,
		MODEL_AXS1=13,
		MODEL_RX10=10,
		MODEL_RX24=24,
		MODEL_RX28=28,
		MODEL_RX64=64,
		MODEL_MX28=29,
		MODEL_MX64=310,
		MODEL_MX106=320,
		MODEL_EX106P=107,
		MODEL_UNKNOWN=static_cast<size_t>(1<<16)
	};
	//! maps model numbers (e.g. from a ServoInfoResponse) to human-readable names
	extern const std::map<DynamixelProtocol::ModelID_t, const std::string> dynamixelModels;
	
	//! symbol names for status response level settings, see SetStatusResponseLevelCmd
	enum StatusResponseLevel {
		RESPOND_NONE=0, //!< don't respond to anything
		RESPOND_READ=1, //!< only respond to read commands (see ReadCmd and GenericResponseHeader subclasses)
		RESPOND_ALL=2 //!< send a response packet for every command (see WriteResponse)
	};
	
	//! Compute a bitwise-negated checksum for a region of memory, with an optional offset
	/*! You can add checksums from multiple calls to this function, then bitwise-negate at the
	*  end to obtain final checksum value */
	inline unsigned char nchecksum(const unsigned char* p, size_t len, size_t off=0) {
		const unsigned char* end=p+len;
		p+=off;
		unsigned char c=0;
		while(p!=end) {
			//debuget::charhexout(*p); cout << ' '; debuget::charhexout(c+*p); cout << endl;
			c+=*p++;
		}
		return c;
	}
	//! 'specialization' of nchecksum for the GenericCmdHeader, which should skip marker fields
	inline unsigned char nchecksum(const struct GenericCmdHeader& p, size_t len);

	//! 'specialization' of nchecksum for the GenericResponseHeader, which should skip marker fields
	inline unsigned char nchecksum(const struct GenericResponseHeader& p, size_t len);

	//! updates the checksum field of the specified structure, call this for a structure after you modify its fields
	template<class T> void updateChecksum(T& cmd) { cmd.checksum=~nchecksum(cmd,sizeof(cmd)-1); }

	//! returns true if markers and checksum are valid
	template<class T> bool validate(const T& msg) {
		const unsigned char MARKER=(unsigned char)-1U;
		if(msg.markerA!=MARKER || msg.markerB!=MARKER) return false;
		if(msg.resplen!=sizeof(msg)-4) return false;
		decltype(msg.checksum) chk = ~nchecksum(msg,sizeof(msg)-1);
		return chk == msg.checksum;
	}
	
	struct GenericCmdHeader {
		GenericCmdHeader(unsigned char bytelen, unsigned char instruction)
		: markerA(MARKER_VALUE), markerB(MARKER_VALUE), servoid(BROADCAST_ID), cmdlen(bytelen),  cmdid(instruction) {}
		operator const char*() const { return reinterpret_cast<const char*>(&markerA); }
		operator const unsigned char*() const { return &markerA; }
		unsigned char markerA;
		unsigned char markerB;
		unsigned char servoid;
		unsigned char cmdlen;
		unsigned char cmdid;
	};
	inline unsigned char nchecksum(const struct GenericCmdHeader& p, size_t len) { return nchecksum(p,len,2); }
	
	
	//! Allows you to do a "synchronized write", where all recipients commit the value(s) at the same time, and do not send any response packets
	/*! To do a synchronized write, first send a SyncWriteHeader indicating the type of write command
	 *  being sent (T for example below) and the number of entries.
	 *  Then send the corresponing entries themselves, followed by a checksum:
	 * - unsigned char checksum = 0;
	 * - write( os, SyncWriteHeader<T>(n), checksum );
	 * - for(int i=0; i<n; ++i) { write( os, t[i], checksum ); }
	 * - os.put(~checksum); // note bitwise-not!
	 */
	template<class T>
	struct SyncWriteHeader : public GenericCmdHeader {
		SyncWriteHeader(unsigned char len)
		: GenericCmdHeader(sizeof(T)*len+4,0x83), addr(T::ADDRESS), writelen(sizeof(T)-1) {}
		unsigned char addr;
		unsigned char writelen;
	};

	struct SyncWriteEntry {
		SyncWriteEntry() : servoid() {}
		SyncWriteEntry(unsigned char sid) : servoid(sid) {}
		operator const char*() const { return reinterpret_cast<const char*>(&servoid); }
		operator const unsigned char*() const { return &servoid; }
		unsigned char servoid;
	};

	//! An element that follows a SyncWriteHeader, controls position and speed
	struct SyncWritePosSpeedEntry : public SyncWriteEntry {
		SyncWritePosSpeedEntry() : SyncWriteEntry(), posl(), posh(), speedl(), speedh() {}
		SyncWritePosSpeedEntry(unsigned char sid, unsigned short pos, unsigned short speed)
		: SyncWriteEntry(sid), posl(pos), posh(pos>>8), speedl(speed), speedh(speed>>8) {}
		static const unsigned char ADDRESS=0x1E;
		unsigned char posl;
		unsigned char posh;
		unsigned char speedl;
		unsigned char speedh;
	};

	//! An element that follows a SyncWriteHeader, enables or disables continuous rotation mode
	struct SyncWriteContinuousRotationEntry : public SyncWriteEntry {
		SyncWriteContinuousRotationEntry() : SyncWriteEntry(), ccwlimitl(), ccwlimith() {}
		SyncWriteContinuousRotationEntry(unsigned char sid, bool enable, ModelID_t model) :
		  SyncWriteEntry(sid), ccwlimitl(enable?0:0xFF), 
		  ccwlimith(IS_MXEX(model) ? (enable?0:0x0F) : (enable?0:0x03) ) {}
                static const unsigned char ADDRESS=0x8;
		unsigned char ccwlimitl;
		unsigned char ccwlimith;
	};

	//! An element that follows a SyncWriteHeader, toggles LED value
	struct SyncWriteLEDEntry : public SyncWriteEntry {
		SyncWriteLEDEntry() : SyncWriteEntry(), led() {}
		SyncWriteLEDEntry(unsigned char sid, bool enable) : SyncWriteEntry(sid), led(enable?1:0) {}
		static const unsigned char ADDRESS=0x19;
		unsigned char led;
	};

	struct SyncWriteComplianceEntry : public SyncWriteEntry {
		SyncWriteComplianceEntry() : SyncWriteEntry(), cwmargin(), ccwmargin(), cwslope(), ccwslope() {}
		SyncWriteComplianceEntry(unsigned char sid, unsigned char margin, unsigned char slope) : SyncWriteEntry(sid), cwmargin(margin), ccwmargin(margin), cwslope(slope), ccwslope(slope) {}
		static const unsigned char ADDRESS=0x1A;
		unsigned char cwmargin;
		unsigned char ccwmargin;
		unsigned char cwslope;
		unsigned char ccwslope;
	};

	struct SyncWritePunchEntry : public SyncWriteEntry {
		SyncWritePunchEntry() : SyncWriteEntry(), punchl(), punchh() {}
		SyncWritePunchEntry(unsigned char sid, unsigned short punch) : SyncWriteEntry(sid), punchl(punch&0xFF), punchh(punch>>8) {}
		static const unsigned char ADDRESS=0x30;
		unsigned char punchl;
		unsigned char punchh;
	};

	//! For MX series servos, not AX or RX
	struct SyncWritePIDEntry : public SyncWriteEntry {
		SyncWritePIDEntry() : SyncWriteEntry(), dvalue(), ivalue(), pvalue() {}
		SyncWritePIDEntry(unsigned char sid, unsigned char p, unsigned char i, unsigned char d) :
			SyncWriteEntry(sid), dvalue(d), ivalue(i), pvalue(p) {}
		static const unsigned char ADDRESS=0x1A;
		unsigned char dvalue;
		unsigned char ivalue;
		unsigned char pvalue;
	};

	struct SyncWriteTorqueEntry : public SyncWriteEntry {
		SyncWriteTorqueEntry() : SyncWriteEntry(), maxTorqueL(), maxTorqueH() {}
		SyncWriteTorqueEntry(unsigned char sid, unsigned short max) : SyncWriteEntry(sid), maxTorqueL(max), maxTorqueH(max>>8) {}
		static const unsigned char ADDRESS=0x22;
		unsigned char maxTorqueL;
		unsigned char maxTorqueH;
	};

	//! clears the sound data max hold field of an AX-S1
	struct SyncWriteSoundHoldEntry : public SyncWriteEntry {
		SyncWriteSoundHoldEntry() : SyncWriteEntry(), sndMaxHold(0) {}
		SyncWriteSoundHoldEntry(unsigned char sid) : SyncWriteEntry(sid), sndMaxHold(0) {}
		static const unsigned char ADDRESS=0x24;
		unsigned char sndMaxHold; //!< the maximum microphone value from microphone (requires reset)
	};

	//! clears the sound detected count field of an AX-S1
	/* This can be sent after the count becomes non-zero to clear it so you can detect another count of the same magnitude.
	 *  In other words, the AX-S1 count stays at zero until a time after the last increment, when the total count is posted to the register.
	 *  You don't receive updates for individual increments to the counter — you receive only the total count at the end.
	 *  So once you have read the total count, you should reset it. */
	struct SyncWriteSoundCountEntry : public SyncWriteEntry {
		SyncWriteSoundCountEntry() : SyncWriteEntry(), sndCount(0) {}
		SyncWriteSoundCountEntry(unsigned char sid) : SyncWriteEntry(sid), sndCount(0) {}
		static const unsigned char ADDRESS=0x25;
		unsigned char sndCount; //!< number of times the microphone reading has exceeded a threshold
	};

	//! clears the sound data max hold and sound detected count fields of an AX-S1 at the same time, see ClearSoundHold and ClearSoundCount
	struct SyncWriteSoundHoldAndCountEntry : public SyncWriteEntry {
		SyncWriteSoundHoldAndCountEntry() : SyncWriteEntry(), sndMaxHold(0), sndCount(0) {}
		SyncWriteSoundHoldAndCountEntry(unsigned char sid) : SyncWriteEntry(sid), sndMaxHold(0), sndCount(0) {}
		static const unsigned char ADDRESS=0x24;
		unsigned char sndMaxHold; //!< the maximum microphone value from microphone (requires reset)
		unsigned char sndCount; //!< number of times the microphone reading has exceeded a threshold
	};
	
	struct WriteHeader : public GenericCmdHeader {
		WriteHeader(unsigned char address, unsigned char len)
		: GenericCmdHeader(len+3,0x3), addr(address) {}
		unsigned char addr;
	};

	//! Broadcasts a 'torque enable' (or disable) command to all servos using a write command to the appropriate address
	struct BroadcastTorqueCmd : public WriteHeader {
		//! constructor, pass true to enable torque, false to cut power
		BroadcastTorqueCmd(bool enable)
		: WriteHeader(0x18,1), torqueEnable(enable?1:0), checksum() { updateChecksum(*this); }
		
		unsigned char torqueEnable; //!< non-zero to enable torque of the servo
		char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};

	//! Broadcasts a 'max torque' value to all servos using a write command to the appropriate address
	struct BroadcastTorqueEntry : public WriteHeader {
		//! constructor, pass true to enable torque, false to cut power
		BroadcastTorqueEntry(unsigned char mtorqL=0xff, unsigned char mtorqH=0x03)  // *** warning: hard-coded max torque value
  	: WriteHeader(0x22,2), maxTorqueL(mtorqL), maxTorqueH(mtorqH), checksum() { updateChecksum(*this); }
		unsigned char maxTorqueL; //!< maximum torque low byte
		unsigned char maxTorqueH; //!< maximum torque high byte
		char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};

	//! Broadcasts a command to set controller parameters to 0, also disabling torque (see also BroadcastNoPunchCmd)
	struct BroadcastFullComplianceCmd : public WriteHeader {
		BroadcastFullComplianceCmd()
		: WriteHeader(0x1A,4), cwmargin(0), ccwmargin(0), cwslope(0), ccwslope(0), checksum() { updateChecksum(*this); }
		unsigned char cwmargin;
		unsigned char ccwmargin;
		unsigned char cwslope;
		unsigned char ccwslope;
		char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};

	//! Sets 'punch' parameter to 0, eliminating torque jump (see also BroadcastFullComplianceCmd)
	struct BroadcastNoPunchCmd : public WriteHeader {
		BroadcastNoPunchCmd()
		: WriteHeader(0x30,2), punchl(0), punchh(0), checksum() { updateChecksum(*this); }
		unsigned char punchl;
		unsigned char punchh;
		char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};

	//! Sets 'speed' parameter to 0, ensuring no motion will occur from leftover 'free spin' commands
	struct BroadcastZeroSpeedCmd : public WriteHeader {
		BroadcastZeroSpeedCmd()
		: WriteHeader(0x20,2), speedl(0), speedh(0), checksum() { updateChecksum(*this); }
		unsigned char speedl;
		unsigned char speedh;
		char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};

	//! Sets 'baud rate' parameter to the specified value, where resulting bits per second will be 2M / (baud+1)
	struct BroadcastBaudCmd : public WriteHeader {
		BroadcastBaudCmd(unsigned char divisor) : WriteHeader(0x04,1), baudDivisor(divisor), checksum() { updateChecksum(*this); }
		unsigned char baudDivisor;
		char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};

	//! Sets 'return delay time' parameter to the specified value, where actual delay time is 2µs * value
	struct SetReturnDelayTimeCmd : public WriteHeader {
		SetReturnDelayTimeCmd() : WriteHeader(0x05,1), delayTime(0), checksum() { updateChecksum(*this); }
		SetReturnDelayTimeCmd(unsigned char delay) : WriteHeader(0x05,1), delayTime(delay), checksum() { updateChecksum(*this); }
		SetReturnDelayTimeCmd(unsigned char sid, unsigned char delay) : WriteHeader(0x05,1), delayTime(delay), checksum() { servoid=sid; updateChecksum(*this); }
		unsigned char delayTime; //!< amount of time to delay before responding, in units of 2µs
		char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};	

	//! Sets 'status response level' parameter to the specified value, see StatusResponseLevel
	struct SetStatusResponseLevelCmd : public WriteHeader {
		SetStatusResponseLevelCmd() : WriteHeader(0x10,1), responseLevel(0), checksum() { updateChecksum(*this); }
		SetStatusResponseLevelCmd(StatusResponseLevel level) : WriteHeader(0x05,1), responseLevel(level), checksum() { updateChecksum(*this); }
		SetStatusResponseLevelCmd(unsigned char sid, StatusResponseLevel level) : WriteHeader(0x05,1), responseLevel(level), checksum() { servoid=sid; updateChecksum(*this); }
		unsigned char responseLevel; //!< see StatusResponseLevel
		char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};	

	//! Sends a command to change a servo's ID value (use carefully!)
	struct SetServoIDCmd : public WriteHeader {
		SetServoIDCmd(unsigned char tgtsid, unsigned char newsid)
		: WriteHeader(0x03, 1), newservoid(newsid), checksum() { servoid=tgtsid; updateChecksum(*this); }
		unsigned char newservoid;
		char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};

	//! Sets the position and speed for a single servo
	struct SetPosSpeedCmd : public WriteHeader {
		SetPosSpeedCmd() : WriteHeader(0x1E,4), posl(), posh(), speedl(), speedh(), checksum() { updateChecksum(*this); }
		SetPosSpeedCmd(unsigned char sid, unsigned short pos, unsigned short speed)
		: WriteHeader(0x1E,4), posl(pos), posh(pos>>8), speedl(speed), speedh(speed>>8), checksum() { servoid=sid; updateChecksum(*this); }
		unsigned char posl;
		unsigned char posh;
		unsigned char speedl;
		unsigned char speedh;
		char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};

	//! clears the sound data max hold field of an AX-S1
	struct ClearSoundHoldCmd : public WriteHeader {
		ClearSoundHoldCmd() : WriteHeader(0x24,1), sndMaxHold(0), checksum() { updateChecksum(*this); }
		ClearSoundHoldCmd(unsigned char sid) : WriteHeader(0x24,1), sndMaxHold(0), checksum() { servoid=sid; updateChecksum(*this); }
		unsigned char sndMaxHold; //!< the maximum microphone value from microphone (requires reset)
		char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};

	//! clears the sound detected count field of an AX-S1
	/* This can be sent after the count becomes non-zero to clear it so you can detect another count of the same magnitude.
	 *  In other words, the AX-S1 count stays at zero until a time after the last increment, when the total count is posted to the register.
	 *  You don't receive updates for individual increments to the counter — you receive only the total count at the end.
	 *  So once you have read the total count, you should reset it. */
	struct ClearSoundCountCmd : public WriteHeader {
		ClearSoundCountCmd() : WriteHeader(0x25,1), sndCount(0), checksum() { updateChecksum(*this); }
		ClearSoundCountCmd(unsigned char sid) : WriteHeader(0x25,1), sndCount(0), checksum() { servoid=sid; updateChecksum(*this); }
		unsigned char sndCount; //!< number of times the microphone reading has exceeded a threshold
		char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};

	//! clears the sound data max hold and sound detected count fields of an AX-S1 at the same time, see ClearSoundHold and ClearSoundCount
	struct ClearSoundHoldAndCountCmd : public WriteHeader {
		ClearSoundHoldAndCountCmd() : WriteHeader(0x24,2), sndMaxHold(0), sndCount(0), checksum() { updateChecksum(*this); }
		ClearSoundHoldAndCountCmd(unsigned char sid) : WriteHeader(0x24,2), sndMaxHold(0), sndCount(0), checksum() { servoid=sid; updateChecksum(*this); }
		unsigned char sndMaxHold; //!< the maximum microphone value from microphone (requires reset)
		unsigned char sndCount; //!< number of times the microphone reading has exceeded a threshold
		char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};
	
	//! Provides bitmasks for checking various error conditions in GenericResponseHeader::error
	enum ResponseError_t {
		VOLTAGE_ERROR=1, ANGLE_ERROR=2, HEAT_ERROR=4,
		RANGE_ERROR=8, CHECKSUM_ERROR=16, LOAD_ERROR=32, 
		INSTRUCTION_ERROR=64
	};
	extern const char* ResponseErrorNames[9];
	
	//! contains fields global to all response packets
	struct GenericResponseHeader {
		GenericResponseHeader() : markerA(0), markerB(0), servoid(0), resplen(0), error(0) {}
		operator char*() { return reinterpret_cast<char*>(&markerA); } //!< easy serialization
		operator const char*() const { return reinterpret_cast<const char*>(&markerA); } //!< easy serialization
		operator const unsigned char*() const { return &markerA; } //!< easy serialization
		unsigned char markerA;  //!< should always be 0xFF
		unsigned char markerB;  //!< should always be 0xFF
		unsigned char servoid;  //!< the ID of the servo who made the response
		unsigned char resplen;  //!< number of data bytes which follow, including error field and final checksum
		unsigned char error;    //!< error bitmask, see ResponseError_t
	};
	inline unsigned char nchecksum(const struct GenericResponseHeader& p, size_t len) { return nchecksum(p,len,2); }
	
	const unsigned char RESPONSE_HEADER_LEN = sizeof(GenericResponseHeader)+1;
	
	//! Expected response from a Write command (when sent to non-broadcast address)
	struct WriteResponse : public GenericResponseHeader {
		unsigned char checksum;
		WriteResponse() : checksum(0) {}
	};
	//! Expected response from a ReadServoSensorsCmd, common across AX, RX, and EX series at least
	struct ServoSensorsResponse : public GenericResponseHeader {
		//! constructor
		ServoSensorsResponse() : GenericResponseHeader(), posl(), posh(), speedl(), speedh(), 
														 loadl(), loadh(), voltage(), temp(), checksum() {}
		
		//! returns current position in 'tics', 10 bit resolution
		unsigned short getPosition() const { return (static_cast<unsigned short>(posh)<<8) + posl; }
		//! returns current speed as 'tics' per second, 11 bit signed resolution
		short getSpeed() const { short x = (static_cast<short>(speedh&0x3)<<8) | speedl; return (speedh&4) ? -x : x; }
		//! returns an indication of how hard the servo is working, 11 bit signed resolution; note we flip the sign from what the hardware provides
		short getLoad() const { short x = (static_cast<short>(loadh&0x3)<<8) | loadl; return (loadh&4) ? x : -x; }
		
		unsigned char posl;     	//!< low byte of position
		unsigned char posh;     	//!< high byte of position
		unsigned char speedl;     	//!< low byte of speed
		unsigned char speedh;     	//!< high byte of speed
		unsigned char loadl;     	//!< low byte of load
		unsigned char loadh;     	//!< high byte of load
		unsigned char voltage;     	//!< voltage times 10
		unsigned char temp;     	//!< temperature (Celsius)
		unsigned char checksum;     	//!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};

	//! Expected response from a ReadAXS1SensorsCmd
	struct AXS1SensorsResponse : public GenericResponseHeader {
		//! constructor
		AXS1SensorsResponse() : GenericResponseHeader(),
			leftIR(), centerIR(), rightIR(), leftLum(), centerLum(), rightLum(),
			obsFlag(), lumFlag(), _robotisReserved(), sndData(), sndMaxHold(), sndCount(),
			/*sndTimeLow(), sndTimeHigh(), buzzerIndex(), buzzerTime(), voltage(), temp(), */
			checksum()
		{}
		
		/*
		//! timestamp of last #detCount increment, 16MHz timing: advances by 16000 per ms
		unsigned short getDetectedTime() {
			return (static_cast<unsigned short>(sndTimeHigh)<<8) + sndTimeLow;
		}about:startpage
		 */

		unsigned char leftIR; //!< reflectance from the left
		unsigned char centerIR; //!< reflectance from the center
		unsigned char rightIR; //!< reflectance from the right
		unsigned char leftLum; //!< external illumination from the left
		unsigned char centerLum; //!< external illumination from the center
		unsigned char rightLum; //!< external illumination from the right
		unsigned char obsFlag; //!< bitset indicating whether #leftIR, #centerIR, or #rightIR have exceeded a threshold (unused, requires reset)
		unsigned char lumFlag; //!< bitset indicating whether #leftLum, #centerLum, or #rightLum have exceeded a threshold (unused, requires reset)
		unsigned char _robotisReserved; //!< unused
		unsigned char sndData; //!< current pressure reading from microphone (unused, not sampled frequently enough to be useful)
		unsigned char sndMaxHold; //!< the maximum microphone value from microphone (requires reset)
		unsigned char sndCount; //!< number of times the microphone reading has exceeded a threshold
		/*
		unsigned char sndTimeLow; //!< timestamp of last #detCount increment, 16MHz timing: advances by 16000 per ms (low order byte)
		unsigned char sndTimeHigh; //!< timestamp of last #detCount increment, 16MHz timing: advances by 16000 per ms (high order byte)
		unsigned char buzzerIndex; //!< the note to play, 0-51 inclusive (command value); if #buzzerTime is set to 255, then 0-26 inclusive to select a score (command value)
		unsigned char buzzerTime; //!< time to play the note for in tenths of a second, 0-50; 254 plays continuously, 255 plays scores via #buzzerIndex (command value)
		unsigned char voltage; //!< voltage times 10
		unsigned char temp; //!< temperature (Celsius)
		 */
		unsigned char checksum; //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};

	//! Expected response from a ReadTorqueCmd
	struct TorqueResponse : public GenericResponseHeader {
		unsigned char torqueEnable;  //!< if non-zero, torque is enabled on the servo (false indicates an error may have occurred)
		unsigned char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};

	//! Expected response from a ReadModelCmd
	struct ServoInfoResponse : public GenericResponseHeader {
		ServoInfoResponse() : GenericResponseHeader(), modell(0), modelh(0), version(0), checksum() {}
		unsigned short getModelNumber() const { return (static_cast<unsigned short>(modelh)<<8) | modell; }
		const char* getModelString() const {
			typedef decltype(dynamixelModels) modelmap_t;
			modelmap_t::const_iterator it = dynamixelModels.find((ModelID_t)getModelNumber());
			if(it==dynamixelModels.end()) {
				return (getModelNumber()==0 && version==0) ? "INVALID RESPONSE" : MODEL_UNKNOWN_NAME;
			} else {
				return it->second.c_str();
			}
		}
		unsigned char modell;  //!< indicates low byte of model number
		unsigned char modelh;  //!< indicates high byte of model number
		unsigned char version;  //!< firmware version
		unsigned char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};
	
	//! Contains fields which are global to all read commands, which can read a block of parameters from the unit
	struct ReadCmd : public GenericCmdHeader {
		//! constructor, pass the unit ID, the starting address, and the number of bytes to read
		ReadCmd(unsigned char servoID, unsigned char address, unsigned char len)
		  : GenericCmdHeader(4,0x2), addr(address), readlen(len), checksum() { servoid=servoID; updateChecksum(*this); }
		unsigned char addr;  //!< address to start reading from on the unit
		unsigned char readlen;  //!< number of bytes to read
		unsigned char checksum;  //!< checksum value (bitwise negated sum of all non-marker fields), see nchecksum()
	};
	//! Requests a block of servo sensor values be sent, servo should respond with a ServoSensorsResponse
	struct ReadServoSensorsCmd : public ReadCmd {
		//! constructor, pass the ID of the servo you want to query
		ReadServoSensorsCmd(unsigned char servoID) : ReadCmd(servoID,0x24,sizeof(ServoSensorsResponse)-RESPONSE_HEADER_LEN) {}
	};
	//! Requests a block of AX-S1 sensor values be sent, module should respond with a AXS1SensorsResponse
	struct ReadAXS1SensorsCmd : public ReadCmd {
		//! constructor, pass the ID of the servo you want to query
		ReadAXS1SensorsCmd(unsigned char servoID) : ReadCmd(servoID,0x1A,sizeof(AXS1SensorsResponse)-RESPONSE_HEADER_LEN) {}
	};
	//! Requests the 'torque enable' status be sent, servo should respond with a TorqueResponse
	struct ReadTorqueCmd : public ReadCmd {
		//! constructor, pass the ID of the servo you want to query
		ReadTorqueCmd(unsigned char servoID) : ReadCmd(servoID,0x18,sizeof(TorqueResponse)-RESPONSE_HEADER_LEN) {}
	};
	//! Requests the 'model number' be sent, servo should respond with a ServoInfoResponse
	struct ReadModelCmd : public ReadCmd {
		//! constructor, pass the ID of the servo you want to query
		ReadModelCmd(unsigned char servoID) : ReadCmd(servoID,0x00,sizeof(ServoInfoResponse)-RESPONSE_HEADER_LEN) {}
	};
	
	void reportErrors(unsigned int servoID, unsigned int offset, unsigned char err);
	
	//! reads a response from an input stream, attempts to handle line noise before response
	/*! returns false if an invalid response type is received */
	template<class R> bool readResponse(std::istream& is, R& response, unsigned int offset) {
		//std::cout << "reading markers..." << std::endl;
		is.read((char*)&response.markerA,sizeof(response.markerA)*2);
		if(!is || is.gcount()!=sizeof(response.markerA)*2) {
			Thread::testCurrentCancel();
			std::cerr << "Dynamixel protocol bad read! 1" << std::endl;
			is.sync();
			is.clear();
			return false;
		}
		size_t noiseCnt=0;
		//std::cout << "got markers " << (int)response.markerA << ' ' << (int)response.markerB << std::endl;
		while(response.markerA!=DynamixelProtocol::MARKER_VALUE || response.markerB!=DynamixelProtocol::MARKER_VALUE) {
			//std::cout << "re-reading markers " << (int)response.markerA << ' ' << (int)response.markerB << std::endl;
			++noiseCnt;
			response.markerA=response.markerB;
			try {
				is.read((char*)&response.markerB,sizeof(response.markerB));
				if(!is || is.gcount()!=sizeof(response.markerB)) {
					Thread::testCurrentCancel();
					std::cerr << "Dynamixel protocol bad read! 2" << std::endl;
					is.sync();
					is.clear();
				}
			} catch(...) {
				std::cerr << "Dynamixel protocol couldn't find packet start, skipped " << noiseCnt << " bytes of line noise." << std::endl;
				throw;
			}
		}
		if(noiseCnt!=0)
			std::cerr << "Dynamixel protocol skipping " << noiseCnt << " bytes of line noise" << std::endl;
		const size_t HEADER_SIZE = sizeof(response.servoid)+sizeof(response.resplen)+sizeof(response.error);
		//std::cout << "reading header " << HEADER_SIZE << std::endl;
		is.read((char*)&response.servoid,HEADER_SIZE);
		if(!is || (size_t)is.gcount()!=HEADER_SIZE) {
			Thread::testCurrentCancel();
			std::cerr << "Dynamixel protocol bad read! 3" << std::endl;
			is.sync();
			is.clear();
			return false;
		}
		if(response.resplen<2) {
			std::cerr << "Dynamixel protocol got bad packet, too short! (" << (int)response.resplen << ")" << std::endl;
			return false;
		}
		//std::cout << "got header from " << (int)response.servoid << std::endl;
		if(response.resplen != sizeof(R)-sizeof(GenericResponseHeader)+1) {
			unsigned char tmpbuf[256];
			memcpy(tmpbuf,&response.servoid,HEADER_SIZE);
			is.read((char*)tmpbuf+HEADER_SIZE,response.resplen-sizeof(response.error));
			if(!is || (size_t)is.gcount()!=response.resplen-sizeof(response.error)) {
				Thread::testCurrentCancel();
				std::cerr << "Dynamixel protocol bad read! 4" << std::endl;
				is.sync();
				is.clear();
				return false;
			}
			unsigned char rchksum = tmpbuf[HEADER_SIZE+response.resplen-sizeof(response.error)-1];
			unsigned char lchksum = ~nchecksum(tmpbuf,HEADER_SIZE+response.resplen-sizeof(response.error)-1,0);
			if( lchksum == rchksum) {
				if(response.error==0)
					std::cerr << "Dynamixel protocol: invalid response (expected " << (sizeof(R)-sizeof(GenericResponseHeader)+1) << ", length=" << (int)response.resplen << ")" << std::endl;
				else
					reportErrors(response.servoid,offset,response.error);
			} else {
				std::cerr << "Dynamixel line noise: sensor checksum failed" << std::endl;
				is.sync();
			}
			return false;
		}
		
		//std::cout << "Header complete, reading payload size " << response.resplen-sizeof(response.error) << std::endl;
		is.read((char*)(&response.error+1),response.resplen-sizeof(response.error));
		if(!is || (size_t)is.gcount()!=response.resplen-sizeof(response.error)) {
			Thread::testCurrentCancel();
			std::cerr << "Dynamixel protocol bad read! 5" << std::endl;
			is.sync();
			is.clear();
			return false;
		} else if(!validate(response)) {
			/*std::cerr << "Sensor checksum failed, response was: " << std::flush; 
			debuget::hexout(response,sizeof(response));
			std::cerr << "checksum: " << std::flush;
			debuget::charhexout(~nchecksum(response,response.resplen+3)); std::cout.flush();
			std::cerr << std::endl;*/
			std::cerr << "Dynamixel line noise: sensor checksum failed" << std::endl;
			is.sync();
			return false;
		}
		
		if(response.error!=0)
			reportErrors(response.servoid,offset,response.error);
		return true;
	}
	
	//! Attempts to ping the specified servo by reading its model number, and if successful, current sensor values
	/*! The run() call returns with the model string if successful, or NULL if the ping timed out */
	class PingThread : public Thread {
	public:
		//! constructor, pass input and output streams, the servo id, and optional sensor response to be filled in
		PingThread(std::istream& is, std::ostream& os, unsigned char servoid, unsigned int outputOffset, ServoSensorsResponse* servoinfo=NULL, AXS1SensorsResponse* servoinfoS1=NULL)
		  : Thread(), response(), icomm(is), ocomm(os), sid(servoid), output(outputOffset), info(servoinfo), infoS1(servoinfoS1), unknownModelName() { start(); }
		//! destructor, stop and join thread
		~PingThread() { if(isStarted()) stop().join(); }
		
		static long getTimeout() { return timeout; } //!< get #timeout
		static void setTimeout(long t) { timeout = t; } //!< set #timeout
		
		ServoInfoResponse response;
		
	protected:
		virtual void * run();
		virtual void cancelled();
		static long timeout; //!< time to wait for a response before giving up (in milliseconds)
		std::istream& icomm;  //!< input stream
		std::ostream& ocomm;  //!< output stream
		unsigned char sid;  //!< servo id value
		unsigned int output; //!< output offset
		ServoSensorsResponse* info;  //!< response to be filled in if successful and the servo is servo following common Dynamixel servo register layout.
		AXS1SensorsResponse* infoS1;  //!< response to be filled in if successful and the servo is an AX-S1 model.
		std::string unknownModelName; //!< temporary storage to append the model number to 

	private:
		PingThread(const PingThread&); //!< dummy
		PingThread& operator=(const PingThread&); //!< dummy
	};
	
	// this is a bad idea because it lets you encode 'pure' headers, where the actual data is just junk values following the header
	//inline std::ostream& operator<<(std::ostream& os, const GenericCmdHeader& cmd) { return os.write(cmd,cmd.cmdlen+4); }
	
	//! writes a command into a stream, returning the stream for convenient ostream::flush call.
	template<class T> std::ostream& write(std::ostream& os, const T& cmd) { return os.write(cmd,sizeof(cmd)); }
	//! writes a command into a stream and incrementing a checksum, returning the stream for convenient ostream::flush call.  Remember to bitwise-not (~) the checksum before transmission!
	template<class T> std::ostream& write(std::ostream& os, const T& cmd, unsigned char& checksum) { checksum+=nchecksum(cmd,sizeof(cmd)); return os.write(cmd,sizeof(cmd)); }
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
