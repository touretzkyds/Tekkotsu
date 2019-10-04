#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_POWER_STATUS

#include "BatteryCheckControl.h"
#include "Shared/WorldState.h"
#include "Motion/MMAccessor.h"
#include "Motion/LedMC.h"
#include "NullControl.h"
#include "Events/EventRouter.h"
#include "Wireless/Wireless.h"

ControlBase * BatteryCheckControl::activate(MC_ID display, Socket * gui) {
	erouter->addListener(this,EventBase::powerEGID);
	return ControlBase::activate(display,gui);
}

void BatteryCheckControl::pause() {
	erouter->removeListener(this);
	display_id=invalid_MC_ID;
}

void BatteryCheckControl::refresh() {
	report();
	if(gui_comm!=NULL && wireless->isConnected(gui_comm->sock)) {
		char tmp[20];
		sprintf(tmp,"%d",(unsigned int)(state->sensors[PowerRemainOffset]*100));
		//		pushSlot(new NullControl(std::string("Power remain: ")+tmp+std::string("%"),"See console output for details"));
		std::string s("refresh\n");
		s+=getName()+"\n1\n0\n0\nPower remain: ";
		s+=tmp;
		s+="%\n0\nSee console output for details\n";
		s+="status\n1\nPower remaining: ";
		s+=tmp;
		s+="%\n";
		gui_comm->write((const byte*)s.c_str(),s.size());
	}
}

void BatteryCheckControl::deactivate() {
	erouter->removeListener(this);
	display_id=invalid_MC_ID;
}

void BatteryCheckControl::processEvent(const EventBase& event) {
	if(event.getSourceID()!=PowerSrcID::VibrationSID)
		refresh();
}

void BatteryCheckControl::report() {
	unsigned int tmp;
	sout->printf("BATTERY REPORT:\n");
	
	tmp = capabilities.findSensorOffset("PowerRemain");
	if(tmp!=-1U)
		sout->printf("\tPower Remain:\t%d%%\n",(int)(state->sensors[tmp]*100));
	
	tmp = capabilities.findSensorOffset("PowerCapacity");
	if(tmp!=-1U)
		sout->printf("\tCapacity:\t%g\n",state->sensors[tmp]);
	
	tmp = capabilities.findSensorOffset("PowerVoltage");
	if(tmp!=-1U)
		sout->printf("\tVoltage:\t%g\n",state->sensors[tmp]);
	
	tmp = capabilities.findSensorOffset("PowerCurrent");
	if(tmp!=-1U)
		sout->printf("\tCurrent:\t%g\n",state->sensors[tmp]);
	
	tmp = capabilities.findSensorOffset("PowerThermo");
	if(tmp!=-1U)
		sout->printf("\tTemperature:\t%g\n",state->sensors[tmp]);
	
	sout->printf("\tFlags:\t");
	if(state->powerFlags[PowerSrcID::BatteryConnectSID])
		sout->printf("BatteryConnect ");
	if(state->powerFlags[PowerSrcID::DischargingSID])
		sout->printf("Discharging ");
	if(state->powerFlags[PowerSrcID::ChargingSID])
		sout->printf("Charging ");
	if(state->powerFlags[PowerSrcID::ExternalPowerSID])
		sout->printf("ExternalPower ");
	if(state->powerFlags[PowerSrcID::PowerGoodSID])
		sout->printf("PowerGood ");
	if(state->powerFlags[PowerSrcID::LowPowerWarnSID])
		sout->printf("LowPowerWarn ");
	if(state->powerFlags[PowerSrcID::BatteryEmptySID])
		sout->printf("BatteryEmpty ");
	sout->printf("\n");
	
	tmp = capabilities.findSensorOffset("PowerRemain");
	if(tmp!=-1U && display_id!=invalid_MC_ID) {
		MMAccessor<LedMC> disp(display_id);
		disp->displayPercent(state->sensors[tmp],LedEngine::major,LedEngine::major);
	}
}

REGISTER_CONTROL(BatteryCheckControl,"Status Reports");

#endif

/*! @file
 * @brief Implements BatteryCheckControl, which will spew a power report to stdout upon activation
 * @author ejt (Creator)
 */
