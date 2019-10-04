#include "Shared/RobotInfo.h"
#if defined(TGT_IS_CREATE) || defined(TGT_IS_CREATE2)

#include "CreateDriver.h"
#include "CreateCommands.h"
#include "Shared/MarkScope.h"
#include "Shared/get_time.h"
#include "Shared/debuget.h"
#include "Shared/TimeET.h"

#include <arpa/inet.h>
#include <stdio.h>
#include <cmath>

using namespace std; 

#ifdef TGT_IS_CREATE
const std::string CreateDriver::autoRegisterCreateDriver = DeviceDriver::getRegistry().registerType<CreateDriver>("Create");
#else
const std::string CreateDriver::autoRegisterCreateDriver = DeviceDriver::getRegistry().registerType<CreateDriver>("Create2");
#endif

void CreateDriver::motionStarting() {
  //	std::cout << "motionStarting called!" << std::endl;
  MotionHook::motionStarting();
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm==NULL)
    std::cerr << "CreateDriver \"" << instanceName << "\": could not find CommPort \"" << commName << "\"" << std::endl;
  else if(!comm->open())
    std::cerr << "CreateDriver \"" << instanceName << "\": unable to open comm port \"" << commName << "\"" << std::endl;
  else
    connect();
  motionActive=true;
  commName.addPrimitiveListener(this);
}

bool CreateDriver::isConnected() {
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  return (comm!=NULL && comm->isWriteable());
}

void CreateDriver::motionStopping() {
  motionActive=false;
  if(!sensorsActive) // listener count is not recursive, so only remove if we're the last one
    commName.removePrimitiveListener(this);
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm!=NULL)
    comm->close(); // this *is* recursive, so we always close it to match our open() in motionStarting()
  MotionHook::motionStopping();
}

void CreateDriver::motionCheck(const float outputs[][NumOutputs]) {
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm==NULL || !comm->isWriteable())
    return;
	
  vector<unsigned char> commands(0);
	
  commands.push_back(CREATE_DRIVE_DIRECT);
  short output;
  output = (short)outputs[NumFrames-1][RobotInfo::RWheelOffset];
  commands.push_back((unsigned char)(output >> 8));
  commands.push_back((unsigned char)(output & 0xFF));
  output = (short)outputs[NumFrames-1][RobotInfo::LWheelOffset];
  commands.push_back((unsigned char)(output >> 8));
  commands.push_back((unsigned char)(output & 0xFF));
	
  commands.push_back(CREATE_LEDS);
  unsigned char led = 0;
  if (calcLEDValue(RobotInfo::PlayLEDOffset-LEDOffset,outputs[NumFrames-1][RobotInfo::PlayLEDOffset]))
    led = led | 0x2;
  if (calcLEDValue(RobotInfo::AdvanceLEDOffset-LEDOffset,outputs[NumFrames-1][RobotInfo::AdvanceLEDOffset]))
    led = led | 0x8;
  commands.push_back(led);
  float red = outputs[NumFrames-1][RobotInfo::PowerRedLEDOffset];
  float green = outputs[NumFrames-1][RobotInfo::PowerGreenLEDOffset];
  if (red == 0 && green == 0)
    commands.push_back((unsigned char)0);
  else
    commands.push_back((unsigned char)(255 * red / (red + green)));
  commands.push_back((unsigned char)(255 * max(red, green)));
	
  /*
    unsigned char desiredMode = (unsigned char)outputs[NumFrames-1][RobotInfo::ModeOffset];
    if (desiredMode != 0 && desiredMode != lastDesiredMode) {
    if (desiredMode == RobotInfo::MODE_SAFE)
    commands.push_back(CREATE_SAFE);
    if (desiredMode == RobotInfo::MODE_PASSIVE)
    commands.push_back(CREATE_START);
    if (desiredMode == RobotInfo::MODE_FULL)
    commands.push_back(CREATE_FULL);
	 
    lastDesiredMode = desiredMode;
    }
  */
	
  unsigned int dt = static_cast<unsigned int>(NumFrames*FrameTime/((getTimeScale()>0)?getTimeScale():1.f));
  sendCommand(commands, dt*3/4);
  MotionHook::motionCheck(outputs); // updates lastOutputs and isFirstCheck, we ignore its motionUpdated() call
}

bool CreateDriver::sendCommand(vector<unsigned char> bytes, unsigned int timeout) {
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm==NULL)
    return false;
  Thread::Lock& l = comm->getLock();
	
  //	cout << "CREATE: Trying to get lock for write." << endl;
	
  unsigned int t = get_time();
  unsigned int giveup = t + timeout;
	
  while (!l.trylock()) {
    if (get_time() >= giveup) {
      if(MotionHook::verbose>0)
	cerr << "CREATE: Unable to send command: couldn't get lock on comm port" << endl;
      return false;
    }
    usleep(1000);
  }
	
  MarkScope autolock(l); l.unlock(); //transfer lock to MarkScope
	
  std::ostream os(&comm->getWriteStreambuf());
  //cout << endl << "W";
  for (unsigned int i = 0; i < bytes.size(); i++) {
    os << bytes[i];
    //cout << " " << (int)bytes[i];
  }
  os << std::flush;
  //cout << endl;
	
  return true;
}

void CreateDriver::connect() {
  vector<unsigned char> commands(2);
  commands[0] = CREATE_START;
  commands[1] = CREATE_SAFE;
  sendCommand(commands, 3000000);
  // cout << "Connecting to CREATE\n" << flush;
}

unsigned int CreateDriver::nextTimestamp() {
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm==NULL || !comm->isReadable())
    return -1U;
  return get_time();
}

unsigned int CreateDriver::readUnsignedChar(std::istream &is, unsigned char &chk) {
  char b;
	
  // do non-blocking read until given timeout; wait for 100 ms
  TimeET giveup = TimeET() + TimeET((long)100);
  while (!is.readsome(&b,1)) {
    if (giveup < TimeET()) {
      return -1U; //throw std::runtime_error("CREATE DRIVER: Nothing to read");
    }
  }
  chk += (unsigned char)b;
  return (unsigned char)b;
}

unsigned int CreateDriver::readUnsignedShort(std::istream &is, unsigned char &chk) {
  unsigned int a = readUnsignedChar(is, chk);
  if ( a == -1U )
    return a;
  else {
    unsigned int b = readUnsignedChar(is, chk);
    if ( b == -1U )
      return b;
    else
      return (a << 8) | b;
  }
}
int CreateDriver::readPacket(std::istream &is, const char &type, CreateStatus &createStatus, unsigned char &chk) {
  int total = 0;
  unsigned int data = 0;
  int const fail = -1000;
  switch(type) {
  case CREATE_SENSOR_GROUP_0:
    total += readPacket(is, CREATE_SENSOR_DROP, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_WALL, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CLIFF_LEFT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CLIFF_FRONT_LEFT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CLIFF_FRONT_RIGHT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CLIFF_RIGHT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_VIRTUAL_WALL, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_OVERCURRENT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_DIRT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_UNUSED_2, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_IR, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_BUTTONS, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_DISTANCE, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_ANGLE, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CHANGING_STATE, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_VOLTAGE, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CURRENT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_BATTERY_TEMP, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_BATTERY_CHARGE, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_BATTERY_CAPACITY, createStatus, chk);
    if ( total < 0 ) return fail;
    break; 
  case CREATE_SENSOR_GROUP_1:
    total += readPacket(is, CREATE_SENSOR_DROP, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_WALL, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CLIFF_LEFT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CLIFF_FRONT_LEFT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CLIFF_FRONT_RIGHT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CLIFF_RIGHT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_VIRTUAL_WALL, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_OVERCURRENT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_DIRT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_UNUSED_2, createStatus, chk);
    if ( total < 0 ) return fail;
    break;
  case CREATE_SENSOR_GROUP_2:
    total += readPacket(is, CREATE_SENSOR_IR, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_BUTTONS, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_DISTANCE, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_ANGLE, createStatus, chk);
    if ( total < 0 ) return fail;
    break;
  case CREATE_SENSOR_GROUP_3:
    total += readPacket(is, CREATE_SENSOR_CHANGING_STATE, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_VOLTAGE, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CURRENT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_BATTERY_TEMP, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_BATTERY_CHARGE, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_BATTERY_CAPACITY, createStatus, chk);
    if ( total < 0 ) return fail;
    break;
  case CREATE_SENSOR_GROUP_4:
    total += readPacket(is, CREATE_SENSOR_WALL_SIGNAL, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CLIFF_L_SIGNAL, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CLIFF_FL_SIGNAL, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CLIFF_FR_SIGNAL, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CLIFF_R_SIGNAL, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_DIGITAL_IN, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_ANALOG, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CAN_CHARGE, createStatus, chk);
    if ( total < 0 ) return fail;
    break;
  case CREATE_SENSOR_GROUP_5:
    total += readPacket(is, CREATE_SENSOR_OI_MODE, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_SONG_NUMBER, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_SONG_PLAYING, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_STREAM_SIZE, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_REQ_VELOCITY, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_REQ_RADIUS, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_REQ_RIGHT_VELOCITY, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_REQ_LEFT_VELOCITY, createStatus, chk);
    if ( total < 0 ) return fail;
    break;
  case CREATE_SENSOR_GROUP_6:
    total += readPacket(is, CREATE_SENSOR_GROUP_0, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_GROUP_4, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_GROUP_5, createStatus, chk);
    if ( total < 0 ) return fail;
    break;
  #ifdef TGT_IS_CREATE2
  case CREATE_SENSOR_GROUP_100:
    total += readPacket(is, CREATE_SENSOR_GROUP_6, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_GROUP_101, createStatus, chk);
    if ( total < 0 ) return fail;
    break;
  case CREATE_SENSOR_GROUP_101:
    total += readPacket(is, CREATE_SENSOR_ENCODER_LEFT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_ENCODER_RIGHT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_LIGHT_BUMPER, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_GROUP_106, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_IR_LEFT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_IR_RIGHT, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_GROUP_107, createStatus, chk);
    if ( total < 0 ) return fail;
    break;
  case CREATE_SENSOR_GROUP_106:
    total += readPacket(is, CREATE_SENSOR_LIGHT_L_BUMP, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_LIGHT_FL_BUMP, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_LIGHT_CL_BUMP, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_LIGHT_CR_BUMP, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_LIGHT_FR_BUMP, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_LIGHT_R_BUMP, createStatus, chk);
    if ( total < 0 ) return fail;
    break;
  case CREATE_SENSOR_GROUP_107:
    total += readPacket(is, CREATE_SENSOR_CURRENT_L_MOTOR, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CURRENT_R_MOTOR, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CURRENT_MAIN_BRUSH, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_CURRENT_SIDE_BRUSH, createStatus, chk);
    if ( total < 0 ) return fail;
    total += readPacket(is, CREATE_SENSOR_STASIS, createStatus, chk);
    if ( total < 0 ) return fail;
    break;
  #endif
  case CREATE_SENSOR_DROP:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.bumpsWheelDrops = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_WALL:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.wall = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_CLIFF_LEFT:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.cliffLeft = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_CLIFF_FRONT_LEFT:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.cliffFrontLeft = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_CLIFF_FRONT_RIGHT:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.cliffFrontRight = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_CLIFF_RIGHT:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.cliffRight = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_VIRTUAL_WALL:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.virtualWall = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_OVERCURRENT:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.overcurrents = (unsigned char)data;
    total = 1;
    break;
  #ifdef TGT_IS_CREATE2
  case CREATE_SENSOR_DIRT:
      data = readUnsignedChar(is, chk);
      if ( data == -1U ) return fail;
      createStatus.dirt = (char)data;
      total = 1;
      break;
  #else
  case CREATE_SENSOR_UNUSED_1:
  #endif
  case CREATE_SENSOR_UNUSED_2:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    total = 1;
    break;
  case CREATE_SENSOR_IR:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.ir = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_BUTTONS:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.buttons = (unsigned char)data;
    total = 1;
    break;

  #ifdef TGT_IS_CREATE2
  case CREATE_SENSOR_DISTANCE:
  case CREATE_SENSOR_ANGLE:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    total = 1;
    break;
  #else

  case CREATE_SENSOR_DISTANCE:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.distance = (short)data;
    total = 2;
    break;
  case CREATE_SENSOR_ANGLE:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.angle = (short)data;
    total = 2;
    break;
  #endif
  case CREATE_SENSOR_CHANGING_STATE:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.chargingState = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_VOLTAGE:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.voltage = (unsigned short)data;
    total = 2;
    break;
  case CREATE_SENSOR_CURRENT:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.current = (short)data;
    total = 2;
    break;
  case CREATE_SENSOR_BATTERY_TEMP:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.batteryTemperature = (char)data;
    total = 1;
    break;
  case CREATE_SENSOR_BATTERY_CHARGE:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.batteryCharge = (unsigned short)data;
    total = 2;
    break;
  case CREATE_SENSOR_BATTERY_CAPACITY:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.batteryCapacity = (unsigned short)data;
    total = 2;
    break;
  case CREATE_SENSOR_WALL_SIGNAL:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.wallSignal = (unsigned short)data;
    total = 2;
    break;
  case CREATE_SENSOR_CLIFF_L_SIGNAL:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.cliffLeftSignal = (unsigned short)data;
    total = 2;
    break;
  case CREATE_SENSOR_CLIFF_FL_SIGNAL:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.cliffFrontLeftSignal = (unsigned short)data;
    total = 2;
    break;
  case CREATE_SENSOR_CLIFF_FR_SIGNAL:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.cliffFrontRightSignal = (unsigned short)data;
    total = 2;
    break;
  case CREATE_SENSOR_CLIFF_R_SIGNAL:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.cliffRightSignal = (unsigned short)data;
    total = 2;
    break;
  #ifdef TGT_IS_CREATE2
  case CREATE_SENSOR_UNUSED_3:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    total = 1;
    break;
  case CREATE_SENSOR_UNUSED_4:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    total = 1;
    break;
  #else
  case CREATE_SENSOR_DIGITAL_IN:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.userDigitalInputs= (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_ANALOG:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.userAnalogInput = (unsigned short)data;
    total = 2;
    break;
  #endif
  case CREATE_SENSOR_CAN_CHARGE:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.chargingSourcesAvailable = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_OI_MODE:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.oiMode = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_SONG_NUMBER:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.songNumber = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_SONG_PLAYING:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.songPlay = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_STREAM_SIZE:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.streamSize = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_REQ_VELOCITY:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.velocity = (short)data;
    total = 2;
    break;
  case CREATE_SENSOR_REQ_RADIUS:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.radius = (short)data;
    total = 2;
    break;
  case CREATE_SENSOR_REQ_RIGHT_VELOCITY:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.rightVelocity = (short)data;
    total = 2;
    break;
  case CREATE_SENSOR_REQ_LEFT_VELOCITY:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.leftVelocity = (short)data;
    total = 2;
    break;
  #ifdef TGT_IS_CREATE2
  case CREATE_SENSOR_IR_LEFT:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.irLeft = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_IR_RIGHT:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.irRight = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_ENCODER_LEFT:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.encoderLeft = (short)data;
    total = 1;
    break;
  case CREATE_SENSOR_ENCODER_RIGHT:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.encoderRight = (short)data;
    if (firstMessage) {
        firstMessage = false;
    }
    else {
			float const leftEncDelta = createStatus.encoderLeft - prevEncoderLeft;
			float const rightEncDelta = createStatus.encoderRight - prevEncoderRight;
			float const wheelCircumference = M_PI * RobotInfo::wheelDiameter;
			float const leftWheelTravel = leftEncDelta / RobotInfo::encoderTicksPerRev * wheelCircumference;
			float const rightWheelTravel = rightEncDelta / RobotInfo::encoderTicksPerRev * wheelCircumference;
			createStatus.distance = (rightWheelTravel + leftWheelTravel) / 2;
			createStatus.angle = (rightWheelTravel - leftWheelTravel) / RobotInfo::wheelBase * (180 / M_PI);
    }
    prevEncoderLeft = createStatus.encoderLeft;
    prevEncoderRight = createStatus.encoderRight;
    total = 1;
    break;
  case CREATE_SENSOR_LIGHT_BUMPER:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.lightBumper = (unsigned char)data;
    total = 1;
    break;
  case CREATE_SENSOR_LIGHT_L_BUMP:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.lightBumperLeft = (unsigned short)data;
    total = 1;
    break;
  case CREATE_SENSOR_LIGHT_FL_BUMP:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.lightBumperFrontLeft = (unsigned short)data;
    total = 1;
    break;
  case CREATE_SENSOR_LIGHT_CL_BUMP:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.lightBumperCenterLeft = (unsigned short)data;
    total = 1;
    break;
  case CREATE_SENSOR_LIGHT_CR_BUMP:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.lightBumperCenterRight = (unsigned short)data;
    total = 1;
    break;
  case CREATE_SENSOR_LIGHT_FR_BUMP:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.lightBumperFrontRight = (unsigned short)data;
    total = 1;
    break;
  case CREATE_SENSOR_LIGHT_R_BUMP:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.lightBumperRight = (unsigned short)data;
    total = 1;
    break;
  case CREATE_SENSOR_CURRENT_L_MOTOR:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.currentLeftMotor = (short)data;
    total = 1;
    break;
  case CREATE_SENSOR_CURRENT_R_MOTOR:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.currentRightMotor = (short)data;
    total = 1;
    break;
  case CREATE_SENSOR_CURRENT_MAIN_BRUSH:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.currentMainBrush = (short)data;
    total = 1;
    break;
  case CREATE_SENSOR_CURRENT_SIDE_BRUSH:
    data = readUnsignedShort(is, chk);
    if ( data == -1U ) return fail;
    createStatus.currentSideBrush = (short)data;
    total = 1;
    break;
  case CREATE_SENSOR_STASIS:
    data = readUnsignedChar(is, chk);
    if ( data == -1U ) return fail;
    createStatus.stasis = (unsigned char)data;
    total = 1;
    break;
  #endif
  default:
    //std::cerr << "CREATE DRIVER: unknown packet type " << (int) type << std::endl;
    break;
  }
	
  return total;
}

bool CreateDriver::attemptPacketRead(std::istream &is, CreateStatus &createStatus) {
  if(!is)
    return false;
  unsigned char chk;
#ifdef TGT_IS_CREATE2
  int total = readPacket(is, CREATE_SENSOR_GROUP_100, createStatus, chk);
#else
  int total = readPacket(is, CREATE_SENSOR_GROUP_6, createStatus, chk);
#endif
  if ( total > 0 )
    return true;
  else
    return false;
}

// This obsolete code is for reading in streaming mode, which we're
// presently not using because of the Create odometry bug.  It needs
// to be rewritten to follow the new conventions where a read timeout
// returns -1U instead of throwing and exception.
bool CreateDriver::attemptStreamRead(std::istream &is, CreateStatus &createStatus) {
  // Begin packet reading...
  unsigned char in = 0;
  unsigned char type = 0;
  unsigned char numPackets = 0;
  unsigned char chk = 0;
  int i;
	
  if(!is)
    return false;
	
  // cout << "CREATE: Attempting packet read: ";
	
  try {
    type = readUnsignedChar(is, chk);
    int dropped = 0;
    while (type != 19 && dropped < PACKET_LENGTH) {
      dropped++;
      type = readUnsignedChar(is, chk);
    }
		
    if ( dropped > 0 )
      cout << "CREATE: Dropped " << dropped << " bytes searching for start of packet.\n";
		
    if (type != 19) {
      cout << "CREATE: 19 Not found, available data exhausted.\n";
      return false;
    }
		
    // cout << "CREATE: Reading packet...\n";
		
    chk = 19;
    numPackets = readUnsignedChar(is, chk);
		
    for(i = 0; i < numPackets; i+=in) {
      type = readUnsignedChar(is, chk);
			
      in = readPacket(is, type, createStatus, chk) + 1;
    }
		
    if(i != numPackets) {
      //cout << "CREATE: read in different amount than expected! Read " << i << " Expected " << (int)numPackets << endl;
      return false;
    }
		
    readUnsignedChar(is, chk);
    // unsigned char const checksum = readUnsignedChar(is, chk);
    // cout << "CREATE: Finished reading: chksum is " << (int)chk << " and checksum byte is " << (int)checksum << endl;
		
    if (chk != 0) {
      cout << "CREATE: Checksum does not match, discarding packet.\n";
      return false;
    }
  } catch (std::runtime_error &e) {
    // cout << "CREATE: Exception while trying to read data.\n";
    // cout << e.what() << endl;
    return false;
  }
	
  return true;
}

void CreateDriver::resetStatus(CreateStatus &status) {
  status.bumpsWheelDrops = 0;
  status.wall = 0;
  status.cliffLeft = 0;
  status.cliffFrontLeft = 0;
  status.cliffFrontRight = 0;
  status.cliffRight = 0;
  status.virtualWall = 0;
  status.overcurrents = 0;
  status.ir = 0;
  status.buttons = 0;
  status.distance = 0;
  status.angle = 0;
  status.chargingState = 0;
  status.voltage = 0;
  status.current = 0;
  status.batteryTemperature = 0;
  status.batteryCharge = 0;
  status.batteryCapacity = 0;
  status.wallSignal = 0;
  status.cliffLeftSignal = 0;
  status.cliffFrontLeftSignal = 0;
  status.cliffFrontRightSignal = 0;
  status.cliffRightSignal = 0;
  status.chargingSourcesAvailable = 0;
  status.oiMode = 0;
  status.songNumber = 0;
  status.songPlay = 0;
  status.streamSize = 0;
  status.velocity = 0;
  status.radius = 0;
  status.rightVelocity = 0;
  status.leftVelocity = 0;
  #ifdef TGT_IS_CREATE2
  status.dirt = 0;
  status.irLeft = 0;
  status.irRight = 0;
  status.encoderLeft = 0;
  status.encoderRight = 0;
  status.lightBumper = 0;
  status.lightBumperLeft = 0;
  status.lightBumperFrontLeft = 0;
  status.lightBumperCenterLeft = 0;
  status.lightBumperCenterRight = 0;
  status.lightBumperFrontRight = 0;
  status.lightBumperRight = 0;
  status.currentLeftMotor = 0;
  status.currentRightMotor = 0;
  status.currentMainBrush = 0;
  status.currentSideBrush = 0;
  status.stasis = 0;
  #else
  status.userDigitalInputs = 0;
  status.userAnalogInput = 0;
  #endif
}

void CreateDriver::mergeStatus(CreateStatus &oldStatus, CreateStatus &newStatus) {
  oldStatus.bumpsWheelDrops = newStatus.bumpsWheelDrops;
  oldStatus.wall = newStatus.wall;
  oldStatus.cliffLeft = newStatus.cliffLeft;
  oldStatus.cliffFrontLeft = newStatus.cliffFrontLeft;
  oldStatus.cliffFrontRight = newStatus.cliffFrontRight;
  oldStatus.cliffRight = newStatus.cliffRight;
  oldStatus.virtualWall = newStatus.virtualWall;
  oldStatus.overcurrents = newStatus.overcurrents;
  oldStatus.ir = newStatus.ir;
  oldStatus.buttons = newStatus.buttons;
  oldStatus.distance = oldStatus.distance + newStatus.distance;
  oldStatus.angle = oldStatus.angle + newStatus.angle;
  oldStatus.chargingState = newStatus.chargingState;
  oldStatus.voltage = newStatus.voltage;
  oldStatus.current = newStatus.current;
  oldStatus.batteryTemperature = newStatus.batteryTemperature;
  oldStatus.batteryCharge = newStatus.batteryCharge;
  oldStatus.batteryCapacity = newStatus.batteryCapacity;
  oldStatus.wallSignal = newStatus.wallSignal;
  oldStatus.cliffLeftSignal = newStatus.cliffLeftSignal;
  oldStatus.cliffFrontLeftSignal = newStatus.cliffFrontLeftSignal;
  oldStatus.cliffFrontRightSignal = newStatus.cliffFrontRightSignal;
  oldStatus.cliffRightSignal = newStatus.cliffRightSignal;
  oldStatus.chargingSourcesAvailable = newStatus.chargingSourcesAvailable;
  oldStatus.oiMode = newStatus.oiMode;
  oldStatus.songNumber = newStatus.songNumber;
  oldStatus.songPlay = newStatus.songPlay;
  oldStatus.streamSize = newStatus.streamSize;
  oldStatus.velocity = newStatus.velocity;
  oldStatus.radius = newStatus.radius;
  oldStatus.rightVelocity = newStatus.rightVelocity;
  oldStatus.leftVelocity = newStatus.leftVelocity;
  #ifdef TGT_IS_CREATE2
  oldStatus.dirt = newStatus.dirt;
  oldStatus.irLeft = newStatus.irLeft;
  oldStatus.irRight = newStatus.irRight;
  oldStatus.encoderLeft = newStatus.encoderLeft;
  oldStatus.encoderRight = newStatus.encoderRight;
  oldStatus.lightBumper = newStatus.lightBumper;
  oldStatus.lightBumperLeft = newStatus.lightBumperLeft;
  oldStatus.lightBumperFrontLeft = newStatus.lightBumperFrontLeft;
  oldStatus.lightBumperCenterLeft = newStatus.lightBumperCenterLeft;
  oldStatus.lightBumperCenterRight = newStatus.lightBumperCenterRight;
  oldStatus.lightBumperFrontRight = newStatus.lightBumperFrontRight;
  oldStatus.lightBumperRight = newStatus.lightBumperRight;
  oldStatus.currentLeftMotor = newStatus.currentLeftMotor;
  oldStatus.currentRightMotor = newStatus.currentRightMotor;
  oldStatus.currentMainBrush = newStatus.currentMainBrush;
  oldStatus.currentSideBrush = newStatus.currentSideBrush;
  oldStatus.stasis = newStatus.stasis;
  #else
  oldStatus.userAnalogInput = newStatus.userAnalogInput;
  oldStatus.userDigitalInputs = newStatus.userDigitalInputs;
  #endif
}
bool CreateDriver::advance() {
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm==NULL || !comm->isReadable() || !comm->isWriteable())
    return false;
	
  Thread::testCurrentCancel();
	
  CreateStatus tempStatus;
  resetStatus(tempStatus);
	
  // poll for next packet if it's time
  if ( get_time() > lastPollTime + pollInterval ) {
    lastPollTime = get_time();
    vector<unsigned char> commands(2);
    commands[0] = CREATE_SENSORS;
    #ifdef TGT_IS_CREATE2
    commands[1] = CREATE_SENSOR_GROUP_100;
    #else
    commands[1] = CREATE_SENSOR_GROUP_6;
    #endif
    sendCommand(commands, 1000);
    // cout << "Poll\n" << flush;
  }

  // now see if we have anything to read
  std::istream is(&comm->getReadStreambuf());
	
  //cout << "" << get_time() << ": ";
	
  bool valid = false;
  {
    //	  MarkScope autolock(comm->getLock());
    valid = attemptPacketRead(is, tempStatus);
  }
	
  if (!valid) {
    packetFailures++;
		
    if (packetFailures > 0 && packetFailures % 10 == 0) {
      if (packetFailures == 10)
	cout << "CREATE DRIVER: Dropped 10 consecutive packets, attempting to reconnect to Create...\n";
      if (packetFailures % 20 == 0)
	cout << "CREATE DRIVER: Still attempting to reconnect to Create...\n";
      connect();
      usleep(500000);
    }
    return false;
  }
  else {
    packetFailures = 0;
    mergeStatus(globalStatus, tempStatus);
  }
  //std::cout << "got packet\n" << std::flush;
	
  // set back to safe if we need to
  if (tempStatus.oiMode == RobotInfo::MODE_PASSIVE) {
    vector<unsigned char> commands(1); commands[0] = CREATE_SAFE;
    sendCommand(commands, 1000);
  }
  #ifdef TGT_IS_CREATE2	
  setSensorValue(DirtDetectOffset, globalStatus.dirt);
  setSensorValue(IRLeftOffset, globalStatus.irLeft);
  setSensorValue(IRRightOffset, globalStatus.irRight);
  setSensorValue(LeftEncoderOffset, globalStatus.encoderLeft);
  setSensorValue(RightEncoderOffset, globalStatus.encoderRight);
  setSensorValue(LtBumpRightSignalOffset, globalStatus.lightBumperRight);
  setSensorValue(LtBumpFrontRightSignalOffset, globalStatus.lightBumperFrontRight);
  setSensorValue(LtBumpCenterRightSignalOffset, globalStatus.lightBumperCenterRight);
  setSensorValue(LtBumpCenterLeftSignalOffset, globalStatus.lightBumperCenterLeft);
  setSensorValue(LtBumpFrontLeftSignalOffset, globalStatus.lightBumperFrontLeft);
  setSensorValue(LtBumpLeftSignalOffset, globalStatus.lightBumperLeft);
  setSensorValue(LeftMotorCurrentOffset, globalStatus.currentLeftMotor);
  setSensorValue(RightMotorCurrentOffset, globalStatus.currentRightMotor);
  setSensorValue(MainBrushCurrentOffset, globalStatus.currentMainBrush);
  setSensorValue(SideBrushCurrentOffset, globalStatus.currentSideBrush);
  #else
  setSensorValue(DigitalInput0Offset, ((globalStatus.userDigitalInputs >> 0) & 0x1));
  setSensorValue(DigitalInput1Offset, ((globalStatus.userDigitalInputs >> 1) & 0x1));
  setSensorValue(DigitalInput2Offset, ((globalStatus.userDigitalInputs >> 2) & 0x1));
  setSensorValue(DigitalInput3Offset, ((globalStatus.userDigitalInputs >> 3) & 0x1));
  setSensorValue(AnalogSignalOffset,globalStatus.userAnalogInput);
  #endif
  setSensorValue(WallSignalOffset,globalStatus.wallSignal);
  setSensorValue(IRCommOffset,globalStatus.ir);
  setSensorValue(CliffLeftSignalOffset,globalStatus.cliffLeftSignal);
  setSensorValue(CliffFrontLeftSignalOffset,globalStatus.cliffFrontLeftSignal);
  setSensorValue(CliffFrontRightSignalOffset,globalStatus.cliffFrontRightSignal);
  setSensorValue(CliffRightSignalOffset,globalStatus.cliffRightSignal);
  setSensorValue(EncoderDistanceOffset,globalStatus.distance);
  setSensorValue(EncoderAngleOffset,globalStatus.angle);
  setSensorValue(VoltageOffset,globalStatus.voltage);
  setSensorValue(CurrentOffset,globalStatus.current);
  setSensorValue(BatteryChargeOffset,globalStatus.batteryCharge);
  setSensorValue(BatteryTempOffset,globalStatus.batteryTemperature);
  setSensorValue(ChargingStateOffset,globalStatus.chargingState);
  setSensorValue(ModeStateOffset,globalStatus.oiMode);
	
  #ifdef TGT_IS_CREATE2
  setButtonValue(CleanButOffset,(globalStatus.buttons & 0x1));
  setButtonValue(SpotButOffset,(globalStatus.buttons >> 1 ) & 0x1);
  setButtonValue(DockButOffset,(globalStatus.buttons >> 2 ) & 0x1);
  setButtonValue(MinuteButOffset,(globalStatus.buttons >> 3 ) & 0x1);
  setButtonValue(HourButOffset,(globalStatus.buttons >> 4 ) & 0x1);
  setButtonValue(DayButOffset,(globalStatus.buttons >> 5 ) & 0x1);
  setButtonValue(ScheduleButOffset,(globalStatus.buttons >> 6 ) & 0x1);
  setButtonValue(ClockButOffset,(globalStatus.buttons >> 7 ) & 0x1);
  
  setButtonValue(OvercurrentMainBrushOffset,(globalStatus.overcurrents >> 1) & 0x1);
  setButtonValue(OvercurrentSideBrushOffset,(globalStatus.overcurrents >> 0) & 0x1);
  
  setButtonValue(LtBumpRightOffset, (globalStatus.lightBumper >> 5) & 0x1);
  setButtonValue(LtBumpFrontRightOffset, (globalStatus.lightBumper >> 4) & 0x1);
  setButtonValue(LtBumpCenterRightOffset, (globalStatus.lightBumper >> 3) & 0x1);
  setButtonValue(LtBumpCenterLeftOffset, (globalStatus.lightBumper >> 2) & 0x1);
  setButtonValue(LtBumpFrontLeftOffset, (globalStatus.lightBumper >> 1) & 0x1);
  setButtonValue(LtBumpLeftOffset, (globalStatus.lightBumper >> 0 ) & 0x1);
  #else
  setButtonValue(PlayButOffset,(globalStatus.buttons & 0x1));
  setButtonValue(AdvanceButOffset,((globalStatus.buttons >> 2) & 0x1));
  setButtonValue(DropCasterButOffset,((globalStatus.bumpsWheelDrops >> 4) & 0x1));
  setButtonValue(LowSideDriver0ButOffset,((globalStatus.overcurrents >> 2) & 0x1));
  setButtonValue(LowSideDriver1ButOffset,((globalStatus.overcurrents >> 1) & 0x1));
  setButtonValue(LowSideDriver2ButOffset,((globalStatus.overcurrents >> 0) & 0x1));
  #endif
  setButtonValue(WallButOffset,globalStatus.wall);
  setButtonValue(DropLeftWheelButOffset,((globalStatus.bumpsWheelDrops >> 3) & 0x1));
  setButtonValue(DropRightWheelButOffset,((globalStatus.bumpsWheelDrops >> 2) & 0x1));
  setButtonValue(BumpLeftButOffset,((globalStatus.bumpsWheelDrops >> 1) & 0x1));
  setButtonValue(BumpRightButOffset,((globalStatus.bumpsWheelDrops >> 0) & 0x1));
  setButtonValue(OvercurrentLeftWheelOffset,((globalStatus.overcurrents >> 4) & 0x1));
  setButtonValue(OvercurrentRightWheelOffset,((globalStatus.overcurrents >> 3) & 0x1));
  setButtonValue(BaseChargerButOffset,((globalStatus.chargingSourcesAvailable >> 1) & 0x1));
  setButtonValue(InternalChargerButOffset,((globalStatus.chargingSourcesAvailable >> 0) & 0x1));
	
  ++frameNumber;
  return true;
}

void CreateDriver::registerSource() {
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm==NULL) {
    std::cerr << "CreateDriver \"" << instanceName << "\": could not find CommPort \"" << commName << "\"" << std::endl;
  } else if(!comm->open()) {
    std::cerr << "CreateDriver \"" << instanceName << "\": unable to open comm port \"" << commName << "\"" << std::endl;
  } else {
    connect();
  }
  sensorsActive=true;
  commName.addPrimitiveListener(this);
}

void CreateDriver::deregisterSource() {
  CommPort * comm = CommPort::getRegistry().getInstance(commName);
  if(comm!=NULL)
    comm->close();
  sensorsActive=false;
  if(!motionActive) // listener count is not recursive, so only remove if we're the last one
    commName.removePrimitiveListener(this);
}

void CreateDriver::doUnfreeze() {
  MarkScope sl(poller.getStartLock());
  if(!poller.isStarted()) {
    //poller.resetPeriod(1.0/(*sensorFramerate));
    poller.start();
  }
  sensorFramerate->addPrimitiveListener(this);
}

void CreateDriver::doFreeze() {
  MarkScope sl(poller.getStartLock());
  if(poller.isStarted())
    poller.stop().join();
  sensorFramerate->removePrimitiveListener(this);
}

void CreateDriver::plistValueChanged(const plist::PrimitiveBase& pl) {
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
      std::cerr << "CreateDriver \"" << instanceName << "\": could not find CommPort \"" << commName << "\"" << std::endl;
    } else {
      // open each of our new references
      if(sensorsActive)
	comm->open();
      if(motionActive) {
	if(!comm->open())
	  std::cerr << "CreateDriver \"" << instanceName << "\": unable to open comm port \"" << commName << "\"" << std::endl;
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

float CreateDriver::getAnalog(unsigned int /*inputIdx*/, unsigned char s) {
  return s*5.f/256;
}

float CreateDriver::getDigital(unsigned int /*inputIdx*/, unsigned char cur, unsigned char latch) {
  if(cur=='0')
    return 0;
  return (latch=='0') ? 0.5f : 1;
}

/*! @file
 * @brief 
 * @author Benson Tsai (btsai)
 * @author Ethan Tira-Thompson (ejt)
 * @author Alex Grubb (agrubb1)
 */

#endif

