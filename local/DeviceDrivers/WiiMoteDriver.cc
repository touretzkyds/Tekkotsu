/* 
 * This is a little test program to test your bluetooth and wiimote setup.
 * It should compile on any Debian distribution with the following line:
 * 
 * g++ -o test test.c -lwiimote -lbluetooth
 * 
 * If your wiimote or bluetooth libraries are in locations other than /usr, then you should
 * pass the compiler the -I and -L flags of their respective locations.
 * 
 * After that, all you do is run the program.
 * Once you connect, the program will run until you press Ctrl-C.  In the meantime,
 * whenever you are holding down any button on the wiimote, the current accelerometer data
 * will be printed to the screen.  Each accelerometer's values are given as a byte (a char)
 * of data, ranging from 0 to 255, so see how low or high you can get the values to go!
 */

#ifdef __linux__
#ifdef HAVE_CWIID

#include <bluetooth/bluetooth.h>
#include <wiimote.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <assert.h>
#include <signal.h>
#include <math.h>

#include "WiiMoteDriver.h"

#include "Shared/Config.h" //for calibration_{offset,scale}
#include "IPC/Thread.h"

#include <iostream>
#include <sstream>
#include <unistd.h>

const std::string WiiMoteDriver::autoRegisterWiiMoteDriver = DeviceDriver::getRegistry().registerType<WiiMoteDriver>("WiiMote");

DataCache::DataCache():acc_mesg(), btn_mesg(), number(0), state(), states(), 
											 cstate(0), updateCond(), updateLock(), updated(0) {
	
}

unsigned int DataCache::getData(const char *& payload, unsigned int& len, 
																unsigned int& timestamp, std::string& name){
	updateLock.lock();
	if(!updated)
		updateCond.wait(updateLock);
	updated = 0;

	timestamp = get_time();
	number++;

	state.str("");
	state << "#POS\n";
	state << "condensed " << RobotInfo::RobotName << "\n";
	
	state << "meta-info = ";
	state << timestamp;
	state << " ";
	state << number;
	state << "\n";
	
	state << "outputs = 0 0 0 0\n";

	state << "sensors = ";
	state << (unsigned) acc_mesg.x << " ";
	state << (unsigned) acc_mesg.y << " ";
	state << (unsigned) acc_mesg.z << "\n";
	state << "buttons = ";

	state << !!(btn_mesg.buttons & WIIMOTE_BTN_2) << " ";
	state << !!(btn_mesg.buttons & WIIMOTE_BTN_1) << " ";
	state << !!(btn_mesg.buttons & WIIMOTE_BTN_B) << " ";
	state << !!(btn_mesg.buttons & WIIMOTE_BTN_A) << " ";
	state << !!(btn_mesg.buttons & WIIMOTE_BTN_MINUS) << " ";
	state << !!(btn_mesg.buttons & WIIMOTE_BTN_HOME) << " ";
	state << !!(btn_mesg.buttons & WIIMOTE_BTN_LEFT) << " ";
	state << !!(btn_mesg.buttons & WIIMOTE_BTN_RIGHT) << " ";
	state << !!(btn_mesg.buttons & WIIMOTE_BTN_DOWN) << " ";
	state << !!(btn_mesg.buttons & WIIMOTE_BTN_UP) << " ";
	state << !!(btn_mesg.buttons & WIIMOTE_BTN_PLUS) << "\n";

	state << "#END\n";
	//cout << state.str() << endl;
	states[cstate] = state.str();
	len = states[cstate].size();
	name = "(none-sensor)";

	payload = const_cast<char *>(states[cstate].c_str());
	
	cstate = !cstate;
	updateLock.unlock();
	return number;
}

void DataCache::setDataSourceThread(LoadDataThread* p) {
	if(thread==NULL && p!=NULL) {
		// just starting to be used, announce what feedback we provide
		for(unsigned int i=0; i<NumOutputs; i++)
			providingOutput(i);
	} else if(thread!=NULL && p==NULL) {
		// going offline, cancel our announcement
	  for(unsigned int i=0; i<NumOutputs; i++)
		  ignoringOutput(i);
	}
	DataSource::setDataSourceThread(p);
	//if(p!=NULL)
	//  setSource(p->src);
}

void DataCache::updateAccMesg(const wiimote_acc_mesg &mesg) { 
	updateLock.lock();
	acc_mesg = mesg;
	updated = 1;
	updateCond.signal();
	updateLock.unlock();
}

void DataCache::updateBtnMesg(const wiimote_btn_mesg &mesg) {
	updateLock.lock();
	btn_mesg = mesg;
	updated = 1;
	updateCond.signal();
	updateLock.unlock();
}



static void wiimote_mesg_callback(int id, int mesg_count, union wiimote_mesg* mesg[]){
	if(driver){
		driver->mesg_callback(id, mesg_count, mesg);
	}
}

void WiiMoteDriver::mesg_callback(int id, int mesg_count, union wiimote_mesg* mesg[]){
  //printf("Received %d message(s).\n",mesg_count);
  for(int i = 0; i < mesg_count; i++){
    switch(mesg[i]->type){
      case WIIMOTE_MESG_STATUS:{
        printf("It's a status message.  ");
        printf("Battery is at %d.\n",mesg[i]->status_mesg.battery);
        break;
      }
      case WIIMOTE_MESG_BTN:{
				data.updateBtnMesg(mesg[i]->btn_mesg);
        break;
      }
      case WIIMOTE_MESG_ACC:{
				data.updateAccMesg(mesg[i]->acc_mesg);
        break;
      }
      default:
        printf("Error: Unknown type %d\n", mesg[i]->type);
        break;
    }
  }
}

void WiiMoteDriver::motionUpdated(const std::vector<size_t>& changedIndices,
																	const float outputs[][NumOutputs]) {
  if(wiim==NULL)
    return;

	
#ifdef stub_stub_stub_stub
  std::set<size_t> updatedIndices(changedIndices.begin(),changedIndices.end());
	
  for(unsigned int i=LEDOffset; i<LEDOffset+NumLEDs; ++i) {
    float v = outputs[NumFrames-1][i];
    if(v>0 && v<1) // intermediate value, need to flicker activation
      updatedIndices.insert(i);
  }
	
  if(updatedIndices.size()==0)
    return;
  try {
    ::TeRK::LEDCommand ledc;
    ledc.ledMask.assign(NumLEDs,false);
    ledc.ledModes.assign(NumLEDs,::TeRK::LEDOff);
  } catch(...) {
    std::cerr << "error" << std::endl;
    close();
  }
#endif
}

int WiiMoteDriver::init(){
	/*You need an empty bdaddr_t, the bluetooth address*/
	bdaddr_t my_addr;
	memset(my_addr.b,0,sizeof(uint8_t)*6);
	
  /*Initialize your wiimote and id number*/
	int id = -5,status;

	printf("Please press the '1' and '2' buttons on your wiimote simultaneously, until "
		"you see the LEDs at the bottom all start flashing.\n");

	/* This is the first command from the wiimote library, wiimote_connect.
	 * It will block for a few seconds or until it connects to a wiimote.
	 * This can take up to 8 or 9 seconds, or as little as 5.
	 */
	wiim = wiimote_connect(&my_addr,wiimote_mesg_callback,&id);
	
	if(wiim==NULL || id==-5){
		printf("Didn't connect to wiimote.\n");
		return -1;
	}

	printf("Connected to a wiimote.\n");

	/*This method tells the wiimote that we want to turn the LEDs on or off, on in this case.*/
	//status = wiimote_command(wiim,WIIMOTE_CMD_LED,WIIMOTE_LED1_ON);
	//assert(status==0);
	
	/*This method tells the wiimote to vibrate, but not for how long.*/
	//status = wiimote_command(wiim,WIIMOTE_CMD_RUMBLE,1);
	//assert(status==0);
	
	/*So we give it a time to vibrate...about .5 sec.*/
	//usleep(500000);
	
	//status = wiimote_command(wiim,WIIMOTE_CMD_RUMBLE,0);
	//assert(status==0);
	status = wiimote_command(wiim,WIIMOTE_CMD_RPT_MODE,
													 WIIMOTE_RPT_STATUS|WIIMOTE_RPT_BTN|WIIMOTE_RPT_ACC);
	/*
	while(1){
		usleep(1);
	}
	status = wiimote_command(wiim,WIIMOTE_CMD_LED,0);
	assert(status==0);
	printf("Disconnecting from wiimote.\n");
	status = wiimote_disconnect(wiim);
	if(status!=0){
		perror("Disconnect failed:");
		return -1;
	}
	*/
	return 0;
}

#else
#warning "CwiiD library not found! WiiMote driver will not be compiled!"
#endif

#else
#warning "WiiMote driver currently only supports Linux!"
#endif
