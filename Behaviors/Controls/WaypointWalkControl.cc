#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_LEGS) || defined(TGT_HAS_WHEELS)

#include "WaypointWalkControl.h"
#include "ToggleControl.h"
#include "NullControl.h"
#include "FileInputControl.h"
#include "StringInputControl.h"
#include "ValueEditControl.h"
#include "Motion/MotionManager.h"
#include "Motion/WaypointWalkMC.h"
#include "Motion/WalkMC.h"
#include "Motion/MMAccessor.h"
#include "Motion/WaypointEngine.h"
#include "Sound/SoundManager.h"

WaypointWalkControl::WaypointWalkControl()
	: ControlBase("WaypointWalkControl","Allows interactive control and execution of a set of waypoints"),
		isRunning(false), startstopCtl(NULL), loopCtl(NULL), addEgoWPCtl(NULL),
		addOffWPCtl(NULL), addAbsWPCtl(NULL), loadCtl(NULL), saveCtl(NULL),
		localizationCtl(NULL), listOffset(0), walk_id(invalid_MC_ID)
{
	pushSlot(startstopCtl=new NullControl("Execute","Begin running waypoint list"));
	pushSlot(loopCtl=new ToggleControl("Loop Waypoints","When last waypoint is reached, start over"));
	pushSlot(addEgoWPCtl=new NullControl("Add Egocentric Waypoint","Appends a new egocentric waypoint (heading and location relative) at the end of the list"));
	pushSlot(addOffWPCtl=new NullControl("Add Offset Waypoint","Appends a new offset waypoint (location relative) at the end of the list"));
	pushSlot(addAbsWPCtl=new NullControl("Add Absolute Waypoint","Appends a new absolute waypoint at the end of the list"));
	pushSlot(loadCtl=new FileInputControl("Load Waypoints","Reads a path from a file",config->portPath(config->motion.root)));
	loadCtl->setFilter("*.wyp");
	pushSlot(saveCtl=new StringInputControl("Save Waypoints","Writes the current path to a file"));
	pushSlot(localizationCtl=new StringInputControl("Drift Error Correction","Enter 3 numbers 'x y a' reprenting current error"));
	pushSlot(NULL);
	listOffset=slotsSize();
}

ControlBase *
WaypointWalkControl::activate(MC_ID disp_id, Socket * gui) {
	if(walk_id==invalid_MC_ID) {
		SharedObject<WaypointWalkMC> walk;
		walk_id=motman->addPersistentMotion(walk);
	}
	return ControlBase::activate(disp_id,gui);
}

void
WaypointWalkControl::refresh() {
	if(saveCtl->getLastInput().size()>0) {
		std::string filename=saveCtl->getLastInput();
		if(filename.find(".")==std::string::npos)
			filename+=".wyp";
		std::string path=config->motion.makePath(filename);
		sout->printf("Attempting save to %s...\n",path.c_str());
		MMAccessor<WaypointWalkMC>(walk_id)->SaveWaypointFile(path.c_str());
		saveCtl->clearLastInput();
	}
	if(loadCtl->getLastInput().size()>0) {
		sout->printf("Attempting load from %s...\n",loadCtl->getLastInput().c_str());
		MMAccessor<WaypointWalkMC>(walk_id)->LoadWaypointFile(loadCtl->getLastInput().c_str());
		loadCtl->clearLastInput();
	}
	if(localizationCtl->getLastInput().size()>0) {
		float x=0,y=0,a=0;
		sscanf(localizationCtl->getLastInput().c_str(),"%g %g %g",&x,&y,&a);
		MMAccessor<WaypointWalkMC> walk(walk_id);
		walk->setCurPos(x+walk->getCurX(),y+walk->getCurY(),a+walk->getCurA());
		std::cout << "Position is now " << walk->getCurX() << ' ' << walk->getCurY() << ' ' << walk->getCurA() << std::endl;
		localizationCtl->clearLastInput();
	}
	
	MMAccessor<WaypointWalkMC> walk(walk_id);

	loopCtl->setStatus(walk->getIsLooping());

	//rebuild waypoint list

	//clear old entries
	for(unsigned int i=listOffset; i<slotsSize(); i++)
		delete options[i];
	options.resize(listOffset);

	//add new entries
	WaypointWalkMC::WaypointList_t& wplist=walk->getWaypointList();
	unsigned int wpcnt=1;
	for(WaypointWalkMC::WaypointListIter_t it=wplist.begin(); it!=wplist.end(); it++) {
		char cname[50];
		sprintf(cname,"Waypoint %d",wpcnt++);
		char desc[100];
		const char * pt=NULL;
		switch(it->posType) {
		case Waypoint::POSTYPE_EGOCENTRIC:
			pt="^="; break; //uhh, the '^' is supposed to imply it's heading-relative :-}
		case Waypoint::POSTYPE_OFFSET:
			pt="+="; break;
		case Waypoint::POSTYPE_ABSOLUTE:
			pt="="; break;
		}
		sprintf(desc,"x%s%g, y%s%g, a%s%g, arc=%g", pt,it->x,pt,it->y,
						(it->angleIsRelative?"+=":"="),it->angle,it->arc);
		pushSlot(new WaypointEditControl(cname,desc,walk_id,it));
	}
	ControlBase::refresh();
}

void
WaypointWalkControl::deactivate() {
	motman->removeMotion(walk_id);
	walk_id=invalid_MC_ID;
	ControlBase::deactivate();
}

ControlBase*
WaypointWalkControl::doSelect() {
	for(unsigned int i=0; i<hilights.size(); i++) {
		ControlBase * curctl=options[hilights[i]];
		if(curctl==startstopCtl) {
			if(isRunning) {
				isRunning=false;
				startstopCtl->setName("Execute");
				startstopCtl->setDescription("Begin running waypoint list");
				MMAccessor<WaypointWalkMC>(walk_id)->pause();
			} else {
				isRunning=true;
				startstopCtl->setName("Stop");
				startstopCtl->setDescription("Halt locomotion");
				MMAccessor<WaypointWalkMC>(walk_id)->go();
			}
			sndman->playFile(config->controller.select_snd);
			return curctl;
		} else if(curctl==loopCtl) {
			MMAccessor<WaypointWalkMC>(walk_id)->setIsLooping(!loopCtl->getStatus());
			sndman->playFile(config->controller.select_snd);
			return curctl;
		} else if(curctl==addEgoWPCtl) {
			MMAccessor<WaypointWalkMC>(walk_id)->addEgocentricWaypoint(0,0,false,true,100.f);
			sndman->playFile(config->controller.select_snd);
			return curctl;
		} else if(curctl==addOffWPCtl) {
			MMAccessor<WaypointWalkMC>(walk_id)->addOffsetWaypoint(0,0,false,true,100.f);
			sndman->playFile(config->controller.select_snd);
			return curctl;
		} else if(curctl==addAbsWPCtl) {
			MMAccessor<WaypointWalkMC>(walk_id)->addAbsoluteWaypoint(0,0,false,true,100.f);
			sndman->playFile(config->controller.select_snd);
			return curctl;
		}
	}
	return ControlBase::doSelect();
}
	
WaypointWalkControl::WaypointEditControl::WaypointEditControl(const std::string& n, const std::string& d, MC_ID walkid, WaypointEngine::WaypointListIter_t waypointid)
	: ControlBase(n,d), walk_id(walkid), waypoint_id(waypointid), up(NULL), down(NULL), del(NULL), set(NULL)
{
	pushSlot(up=new NullControl("Up (Move up)","Moves up in waypoint list"));
	pushSlot(down=new NullControl("Down (Move down)","Moves down in waypoint list"));
	pushSlot(del=new NullControl("Delete","Removes from waypoint list"));
	pushSlot(set=new NullControl("Set as current goal","Starts trying to reach this location"));
	pushSlot(NULL);
	MMAccessor<WaypointWalkMC> walk(walk_id);
	Waypoint& curway=*waypoint_id;
	pushSlot(new ValueEditControl<float>("X",&curway.x));
	pushSlot(new ValueEditControl<float>("Y",&curway.y));
	pushSlot(new ValueEditControl<float>("A",&curway.angle));
	pushSlot(new ValueEditControl<float>("Arc",&curway.arc));
	pushSlot(new ValueEditControl<float>("Speed (in mm/s)",&curway.speed));
	pushSlot(new ValueEditControl<float>("Turn Speed (in rad/s)",&curway.turnSpeed));
	char desc[256];
	snprintf(desc,256,"Types: EGO=%d, OFF=%d, ABS=%d",Waypoint::POSTYPE_EGOCENTRIC,Waypoint::POSTYPE_OFFSET,Waypoint::POSTYPE_ABSOLUTE);
	pushSlot(new ValueEditControl<Waypoint::posType_t>("Pos. coord. system",desc,&curway.posType));
	ToggleControl * tmptgl;
	pushSlot(tmptgl=new ToggleControl("Angle is relative"));
	tmptgl->setStatus(curway.angleIsRelative);
	tmptgl->setStore(&curway.angleIsRelative);
	pushSlot(tmptgl=new ToggleControl("Track path"));
	tmptgl->setStatus(curway.trackPath);
	tmptgl->setStore(&curway.trackPath);
}

ControlBase* WaypointWalkControl::WaypointEditControl::doSelect() {
	for(unsigned int i=0; i<hilights.size(); i++) {
		ControlBase * curctl=options[hilights[i]];
		if(curctl==up) {
			std::swap(*waypoint_id,*(waypoint_id-1));
			sndman->playFile(config->controller.select_snd);
			return NULL;
		} else if(curctl==down) {
			std::swap(*waypoint_id,*(waypoint_id+1));
			sndman->playFile(config->controller.select_snd);
			return NULL;
		} else if(curctl==del) {
			MMAccessor<WaypointWalkMC>(walk_id)->getWaypointList().erase(waypoint_id);
			sndman->playFile(config->controller.select_snd);
			return NULL;
		} else if(curctl==set) {
			MMAccessor<WaypointWalkMC>(walk_id)->setTargetWaypoint(waypoint_id);
			sndman->playFile(config->controller.select_snd);
			return NULL;
		}
	}
	return ControlBase::doSelect();
}


		/* //basic
			walk->addEgocentricWaypoint(1,0,0,true,.1);
			walk->addEgocentricWaypoint(0,0,M_PI/2,true,.1);
			walk->addEgocentricWaypoint(1,0,0,true,.1);
			walk->addEgocentricWaypoint(0,0,M_PI/2,true,.1);
			walk->addEgocentricWaypoint(1,0,0,true,.1);
			walk->addEgocentricWaypoint(0,0,M_PI/2,true,.1);
			walk->addEgocentricWaypoint(1,0,0,true,.1);
			walk->addEgocentricWaypoint(0,0,M_PI/2,true,.1);
		*/

		/*
			walk->addEgocentricWaypoint(1,0,0,true,.1);
			walk->addEgocentricWaypoint(0,1,-M_PI/2,true,.1);
			walk->addEgocentricWaypoint(-1,0,0,false,.1);
			walk->addEgocentricWaypoint(0,0,-M_PI/2,true,.1);
			walk->addEgocentricWaypoint(1,0,0,true,.1);
			walk->addEgocentricWaypoint(0,0,M_PI/2,true,.1);
		*/

		/*
			walk->addEgocentricWaypoint(1,0,0,true,.1);
			walk->addEgocentricWaypoint(0,1,0,true,.1);
			walk->addEgocentricWaypoint(0,1,-M_PI/2,true,.1);
			walk->addEgocentricWaypoint(-1,0,M_PI/2,false,.1);
			walk->addEgocentricWaypoint(0,0,-M_PI/2,true,.1);
		*/

		/* //circle1
			walk->addEgocentricArc(0,.5,0,true,.1,M_PI);
			walk->addEgocentricWaypoint(0,0,M_PI/2,true,.1);
			walk->addEgocentricArc(.5,0,M_PI/2,true,.1,M_PI);
			walk->addEgocentricWaypoint(0,0,-M_PI/2,true,.1);
		*/

    /* //circle2
			walk->addEgocentricArc(0,.25,0,true,.1,M_PI);
			walk->addEgocentricWaypoint(0,0,M_PI/2,true,.075);
			walk->addEgocentricArc(.25,0,M_PI/2,true,.1,M_PI);
			walk->addEgocentricWaypoint(0,0,-M_PI/2,true,.075);
		*/
		


/*! @file
 * @brief Implements WaypointWalkControl, which allows interactive control and execution of a set of waypoints 
 * @author ejt (Creator)
 */

#endif
