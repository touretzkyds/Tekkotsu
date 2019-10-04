#include "Shared/RobotInfo.h"
#if defined(TGT_IS_KOBUKI)

#include "KobukiDriver.h"
#include "Shared/MarkScope.h"
#include "Shared/get_time.h"
#include "Shared/debuget.h"
#include "Shared/TimeET.h"
#include "Shared/CalliopeUInfo.h"

#include <arpa/inet.h>
#include <stdio.h>

using namespace std; 
using namespace Kobuki;

const std::string KobukiDriver::autoRegisterKobukiDriver = KobukiDriver::getRegistry().registerType<KobukiDriver>("Kobuki");

void KobukiDriver::motionStarting() {
  //	std::cout << "motionStarting called!" << std::endl;
  MotionHook::motionStarting();
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm==NULL)
    std::cerr << "KobukiDriver \"" << instanceName << "\": could not find CommPort \"" << commName << "\"" << std::endl;
  else if(!comm->open())
    std::cerr << "KobukiDriver \"" << instanceName << "\": unable to open comm port \"" << commName << "\"" << std::endl;
  else
    connect();
  motionActive=true;
  commName.addPrimitiveListener(this);
}

bool KobukiDriver::isConnected() {
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  return (comm!=NULL && comm->isWriteable());
}

void KobukiDriver::motionStopping() {
  motionActive=false;
  if(!sensorsActive) // listener count is not recursive, so only remove if we're the last one
    commName.removePrimitiveListener(this);
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm!=NULL)
    comm->close(); // this *is* recursive, so we always close it to match our open() in motionStarting()
  MotionHook::motionStopping();
}


void KobukiDriver::motionCheck(const float outputs[][NumOutputs]) {
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if (comm == NULL || !comm->isWriteable())
		 return;

	BaseControl basecontrol;
	vector<unsigned char> commands(0);
	short output;
	KobukiCommand commandkobuki;
	commandkobuki.Length = 6;
	commandkobuki.commandData = 1;
	commandkobuki.commandDataSize = sizeof(basecontrol);
	float speed = outputs[NumFrames - 1][RobotInfo::SpeedOffset];
	output = (short)speed;
	commandkobuki.speedLow = output & 0xff;
	commandkobuki.speedHigh = output >> 8;
	float radius = outputs[NumFrames - 1][RobotInfo::RadiusOffset];
	output = (short)radius;
	commandkobuki.radiusLow = output & 0xff;
	commandkobuki.radiusHigh = output >> 8;
	
	//std::cout << "KobukiDriver: speed= " << speed << " radius= " << radius << std::endl;

	commands.push_back(0xaa);
	commands.push_back(0x55); 
	commands.push_back((char)commandkobuki.Length);
	commands.push_back((char)commandkobuki.commandData); 
	commands.push_back((char)commandkobuki.commandDataSize); 
	commands.push_back(commandkobuki.speedLow);
	commands.push_back(commandkobuki.speedHigh); 
	commands.push_back(commandkobuki.radiusLow);
	commands.push_back(commandkobuki.radiusHigh);
	
	unsigned char checksum = 0;	
	for (unsigned int i = 2; i < commands.size(); i++)
		checksum ^= commands.at(i);
 
	commands.push_back(checksum);
		
	unsigned int dt = static_cast<unsigned int> (NumFrames * FrameTime / ((getTimeScale() > 0) ?getTimeScale():1.f)); 
		
	sendCommand(commands, dt*3/4);
		
	MotionHook::motionCheck(outputs);
}

bool KobukiDriver::sendCommand(vector<unsigned char> bytes, unsigned int timeout) {
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm==NULL)
    return false;
  Thread::Lock& l = comm->getLock();
	
  //	cout << "KOBUKI: Trying to get lock for write." << endl;
	
  unsigned int t = get_time();
  unsigned int giveup = t + timeout;
	
  while (!l.trylock()) {
    if (get_time() >= giveup) {
      if(MotionHook::verbose>0)
	cerr << "KOBUKI: Unable to send command: couldn't get lock on comm port" << endl;
      return false;
    }
    usleep(1000);
  }
	
  MarkScope autolock(l); l.unlock(); //transfer lock to MarkScope
	
  std::ostream os(&comm->getWriteStreambuf());
  //cout << endl << "W";
  for (unsigned int i = 0; i < bytes.size(); i++) { // sending one byte at a time or the whole buffer?
    os << bytes[i];
    //cout << " " << (int)bytes[i];
  }
  os << std::flush;
  //cout << endl;
	
  return true;
}


void KobukiDriver::connect() {
  vector<unsigned char> commands(6);
  commands[0] = 0xaa;
  commands[1] = 0x55;
  commands[2] = 3;
  commands[3] = 4;
  commands[4] = 1;
  commands[5] = 0;
  sendCommand(commands, 3000000);
  // cout << "Connecting to KOBUKI\n" << flush;
}

unsigned int KobukiDriver::nextTimestamp() {
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm==NULL || !comm->isReadable())
    return -1U;
  return get_time();
}

bool KobukiDriver::readPacket(std::istream &is) {
  
	State currentState = lookingForHeader0;
	unsigned char data[1];
	unsigned char packetChecksum[1];
	unsigned int packetLength;
	unsigned char packet[256];
	
		while (currentState != gotPacket) {
		
			switch (currentState) {
				case lookingForHeader0:
					is.read((char*)data,1);
					if (data[0] == 0xaa)
						currentState = lookingForHeader1;
					break;
				case lookingForHeader1:
					is.read((char*)data,1);
					if (data[0] == 0x55)
						currentState = waitingForPacket;
					else if (data[0] == 0xaa)
						currentState = lookingForHeader1;
					else
						currentState = lookingForHeader0;
					break;
				case waitingForPacket:
					is.read((char*)data,1);
					packetLength = data[0];
					is.read((char*)packet, packetLength);
					currentState = gotPacket;
					is.read((char*)packetChecksum,1);
					break;
				case gotPacket:
					currentState = lookingForHeader0;
				default:
					break;
			}
		}
		
		
	
	  unsigned char checksum = packetLength;
		for (unsigned int i = 0; i < packetLength; i++)
			checksum ^= packet[i];
			
			
		if (checksum == packetChecksum[0]) {
			packetParser(packet, packetLength);
			currentState = lookingForHeader0;
			return true;
		}
		else {
			currentState = lookingForHeader0;
			return false;
		}
}

void KobukiDriver::packetParser(unsigned char packet[], const unsigned int packetLength) {
	
	
	unsigned int index = 0;
		while (index < packetLength-1) {
			switch (packet[index]) {
				case coreSensor:
				{
					CoreSensor *data = (CoreSensor*) &packet[index];
					//unsigned short int time_stamp = (data->timestamp[1] << 8) | data->timestamp[0];
					short int left_encoder = (data->leftEncoder[1] << 8) | data->leftEncoder[0];
					short int right_encoder = (data->rightEncoder[1] << 8) | data->rightEncoder[0];

					//std::cout << "left_encoder = " << left_encoder << " right encoder = " << right_encoder << std::endl;

					//setSensorValue(TimestampOffset, time_stamp);
  				//setSensorValue(CliffOffset, data->cliff);
					setSensorValue(LeftEncoderOffset, left_encoder);
					setSensorValue(RightEncoderOffset, right_encoder);
					setSensorValue(LeftPwmOffset, data->leftPwm);
					setSensorValue(RightPwmOffset, data->rightPwm);
					setSensorValue(ChargerOffset, data->charger);
					setSensorValue(BatteryOffset, data->battery);
					setSensorValue(OverCurrentOffset, data->overCurrent);
					
					
					setButtonValue(B0ButOffset,(data->buttons >> 0) & 0x1);
  				setButtonValue(B1ButOffset,(data->buttons >> 1) & 0x1);
  				setButtonValue(B2ButOffset,(data->buttons >> 2) & 0x1);
  				setButtonValue(DropLeftWheelButOffset,(data->wheelDrop >> 1) & 0x1);
  				setButtonValue(DropRightWheelButOffset,(data->wheelDrop >> 0) & 0x1);
  				setButtonValue(BumpLeftButOffset,(data->bumper >> 2) & 0x1);
  				setButtonValue(BumpRightButOffset,(data->bumper >> 0) & 0x1);
  				setButtonValue(BumpCenterButOffset,(data->bumper >> 1) & 0x1);
  				setButtonValue(CliffLeftButOffset,(data->cliff >> 2) & 0x1);
  				setButtonValue(CliffRightButOffset,(data->cliff >> 0) & 0x1);
  				setButtonValue(CliffCenterButOffset,(data->cliff >> 1) & 0x1);
  				
					index += sizeof(CoreSensor);
					break;
				}
				case dockInfraRed:
				{
					DockInfraRed *data = (DockInfraRed*) &packet[index];;
					
					setSensorValue(Docking0Offset, data->docking[0]);
					setSensorValue(Docking1Offset, data->docking[1]);
					setSensorValue(Docking2Offset, data->docking[2]);
					
					index += sizeof(DockInfraRed);
					break;
				}
				case inertia:
				{
					Inertia *data = (Inertia*) &packet[index];
					short int angle = (data->angle[1] << 8) | data->angle[0];
					short int angle_rate = (data->angleRate[1] << 8) | data->angleRate[0];
					
					setSensorValue(AngleOffset, angle);
					setSensorValue(AngleRateOffset, angle_rate);
					setSensorValue(Acc0Offset, data->acc[0]);
					setSensorValue(Acc1Offset, data->acc[1]);
					setSensorValue(Acc2Offset, data->acc[2]);
					
					index += sizeof(Inertia);
					break; 
				}
				case cliff:
				{
					Cliff *data = (Cliff*) &packet[index];
					short int cliffBottom0 = (data->bottom0[1] << 8) | data->bottom0[0];
					short int cliffBottom1 = (data->bottom1[1] << 8) | data->bottom1[0];
					short int cliffBottom2 = (data->bottom2[1] << 8) | data->bottom2[0];
					
					setSensorValue(Bottom0Offset, cliffBottom0);
					setSensorValue(Bottom1Offset, cliffBottom1);
					setSensorValue(Bottom2Offset, cliffBottom2);
				
					index += sizeof(Cliff);
					break;
				}
				case current:
				{
					Current *data = (Current*) &packet[index];
					
					setSensorValue(Current0Offset, data->current[0]);
					setSensorValue(Current1Offset, data->current[1]);
					
					index += sizeof(Current);
					break;
				}
				case threeAxisGyro:
				{
					ThreeAxisGyro *data = (ThreeAxisGyro*) &packet[index];
						
					setSensorValue(FrameIdOffset, data->frameId);
					setSensorValue(FollowedDataLenghtOffset, data->followedDataLength);
						
					if(data->followedDataLength == 6) {	
						setSensorValue(GyroParam0Offset, (data->parameters[1] << 8) | data->parameters[0]);
						setSensorValue(GyroParam1Offset, (data->parameters[3] << 8) | data->parameters[2]);
						setSensorValue(GyroParam2Offset, (data->parameters[5] << 8) | data->parameters[4]);
						setSensorValue(GyroParam3Offset, (data->parameters[7] << 8) | data->parameters[6]);
						setSensorValue(GyroParam4Offset, (data->parameters[9] << 8) | data->parameters[8]);
						setSensorValue(GyroParam5Offset, (data->parameters[11] << 8) | data->parameters[10]);
					}
					else {
						setSensorValue(GyroParam0Offset, (data->parameters[1] << 8) | data->parameters[0]);
						setSensorValue(GyroParam1Offset, (data->parameters[3] << 8) | data->parameters[2]);
						setSensorValue(GyroParam2Offset, (data->parameters[5] << 8) | data->parameters[4]);
						setSensorValue(GyroParam3Offset, (data->parameters[7] << 8) | data->parameters[6]);
						setSensorValue(GyroParam4Offset, (data->parameters[9] << 8) | data->parameters[8]);
						setSensorValue(GyroParam5Offset, (data->parameters[11] << 8) | data->parameters[10]);
						setSensorValue(GyroParam5Offset, (data->parameters[13] << 8) | data->parameters[12]);
						setSensorValue(GyroParam5Offset, (data->parameters[15] << 8) | data->parameters[14]);
						setSensorValue(GyroParam5Offset, (data->parameters[17] << 8) | data->parameters[16]);
					}
					index += sizeof(ThreeAxisGyro) + 2 * data->followedDataLength;
					break;
				}
				case gpInput:
				{
					GpInput *data = (GpInput*) &packet[index]; 
						
					setSensorValue(DigitalInputOffset, (data->digitalInput[1] << 8) | data->digitalInput[0]);
					setSensorValue(AnalogInput0Offset,(data->analogInput[1] << 8) | data->analogInput[0]);
					setSensorValue(AnalogInput1Offset,(data->analogInput[3] << 8) | data->analogInput[2]);
					setSensorValue(AnalogInput2Offset,(data->analogInput[5] << 8) | data->analogInput[4]);
					setSensorValue(AnalogInput3Offset,(data->analogInput[7] << 8) | data->analogInput[6]);
					setSensorValue(AnalogInput4Offset,(data->analogInput[9] << 8) | data->analogInput[8]);
					setSensorValue(AnalogInput5Offset,(data->analogInput[11] << 8) | data->analogInput[10]);
					setSensorValue(AnalogInput6Offset,(data->analogInput[13] << 8) | data->analogInput[12]);
					
					index += sizeof(GpInput);
					break;
				}
				default:
					index += packetLength;
					break;
			}
		}
}

bool KobukiDriver::advance() {
	CommPort * comm = CommPort::getRegistry().getInstance(commName);
	if (comm == NULL || !comm->isReadable() || !comm->isWriteable())
		return false;
		
	Thread::testCurrentCancel();
	
	std::istream is(&comm->getReadStreambuf());
	
	bool valid = readPacket(is);
	if (valid) {
  	++frameNumber;
  	return true;
  }
  else
  	return false;
}

void KobukiDriver::registerSource() {
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm==NULL) {
    std::cerr << "KobukiDriver \"" << instanceName << "\": could not find CommPort \"" << commName << "\"" << std::endl;
  } else if(!comm->open()) {
    std::cerr << "KobukiDriver \"" << instanceName << "\": unable to open comm port \"" << commName << "\"" << std::endl;
  } else {
    connect();
  }
  sensorsActive=true;
  commName.addPrimitiveListener(this);
}

void KobukiDriver::deregisterSource() {
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm!=NULL)
    comm->close();
  sensorsActive=false;
  if(!motionActive) // listener count is not recursive, so only remove if we're the last one
    commName.removePrimitiveListener(this);
}

void KobukiDriver::doUnfreeze() {
  MarkScope sl(poller.getStartLock());
  if(!poller.isStarted()) {
    //poller.resetPeriod(1.0/(*sensorFramerate));
    poller.start();
  }
  sensorFramerate->addPrimitiveListener(this);
}

void KobukiDriver::doFreeze() {
  MarkScope sl(poller.getStartLock());
  if(poller.isStarted())
    poller.stop().join();
  sensorFramerate->removePrimitiveListener(this);
}

void KobukiDriver::plistValueChanged(const plist::PrimitiveBase& pl) {
  // std::cout << "plistvalueChanged Called!" << std::endl;
  if(&pl==&commName) {
    // if here, then motionStarted or setDataSourceThread has been called, thus when commName changes,
    // need to close old one and reopen new one
    if(poller.isStarted())
      poller.stop().join();
		
    CommPort * comm = CommPort::getRegistry().getInstance(commName.getPreviousValue());
    if(comm!=NULL) {
      // close each of our old references
      if(sensorsActive)
	comm->close();
      if(motionActive)
	comm->close();
    }
    comm = CommPort::getRegistry().getInstance(commName);
    if(comm==NULL) {
      std::cerr << "KobukiDriver \"" << instanceName << "\": could not find CommPort \"" << commName << "\"" << std::endl;
    } else {
      // open each of our new references
      if(sensorsActive)
	comm->open();
      if(motionActive) {
	if(!comm->open())
	  std::cerr << "KobukiDriver \"" << instanceName << "\": unable to open comm port \"" << commName << "\"" << std::endl;
	else
	  connect();
      }
    }
		
    if(getTimeScale()>0) {
      //poller.resetPeriod(1.0/(*sensorFramerate));
      poller.start();
    }
  } else if(&pl==sensorFramerate) {
    //poller.resetPeriod(1.0/(*sensorFramerate));
  }
}
#endif
