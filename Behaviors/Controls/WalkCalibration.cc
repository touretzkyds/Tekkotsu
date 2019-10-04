#include "WalkCalibration.h"
#include "FileInputControl.h"
#include "StringInputControl.h"
#include "NullControl.h"
#include "Wireless/Wireless.h"
#include "Behaviors/Controller.h"
#include "Shared/WorldState.h"
#include "Shared/string_util.h"
#include "Shared/Config.h"
#include "Sound/SoundManager.h"
#include <stdlib.h>
#include <fstream>

using namespace std;

const char * WalkCalibration::datanames[WalkCalibration::NUM_SRC] = { "fs","fr","sr","br","bs","rr" };


WalkCalibration::WalkCalibration()
	: ControlBase("Interactive Calibration","Leads you through the process of calibrating the current walk"),
		st(ROOT), curType(NUM_SRC), 
		old_x(0), old_y(0), old_a(0), startTime(0), stopTime(0), 
		help(NULL), load(NULL), save(NULL), measure(NULL), clear(NULL), polar(NULL), rect(NULL), isPolar(true),
		lastLoad(), firstIn(0), secondIn(0), status()
{
	help=new ControlBase("Help");
	help->pushSlot(new NullControl("This control will"));
	help->pushSlot(new NullControl("lead you through the"));
	help->pushSlot(new NullControl("process of taking"));
	help->pushSlot(new NullControl("measurements."));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("In order to take"));
	help->pushSlot(new NullControl("accurate measurements,"));
	help->pushSlot(new NullControl("the types of motion"));
	help->pushSlot(new NullControl("have been broken"));
	help->pushSlot(new NullControl("down into 6"));
	help->pushSlot(new NullControl("categories:"));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("fs - forward/strafe"));
	help->pushSlot(new NullControl("fr - forward/rotate"));
	help->pushSlot(new NullControl("sr - strafe/rotate"));
	help->pushSlot(new NullControl("br - backwards/rotate"));
	help->pushSlot(new NullControl("bs - backwards/strafe"));
	help->pushSlot(new NullControl("rr - pure rotation"));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("In each of these"));
	help->pushSlot(new NullControl("categories, you will"));
	help->pushSlot(new NullControl("set the aibo to walk"));
	help->pushSlot(new NullControl("in a direction using"));
	help->pushSlot(new NullControl("mainly the two noted"));
	help->pushSlot(new NullControl("parameters, and then"));
	help->pushSlot(new NullControl("tweak the \"unused\""));
	help->pushSlot(new NullControl("parameter to cancel"));
	help->pushSlot(new NullControl("out any drift in"));
	help->pushSlot(new NullControl("that direction."));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("For example, if"));
	help->pushSlot(new NullControl("taking an 'fs'"));
	help->pushSlot(new NullControl("measurement, you"));
	help->pushSlot(new NullControl("would pick any"));
	help->pushSlot(new NullControl("forward and strafe"));
	help->pushSlot(new NullControl("values, and then"));
	help->pushSlot(new NullControl("fiddle with the"));
	help->pushSlot(new NullControl("rotational speed"));
	help->pushSlot(new NullControl("until it maintained"));
	help->pushSlot(new NullControl("a constant heading."));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("If measuring one of"));
	help->pushSlot(new NullControl("the rotational"));
	help->pushSlot(new NullControl("types, you will"));
	help->pushSlot(new NullControl("tweak the unused"));
	help->pushSlot(new NullControl("directional movement"));
	help->pushSlot(new NullControl("so the aibo is"));
	help->pushSlot(new NullControl("facing exactly 180"));
	help->pushSlot(new NullControl("degrees from its"));
	help->pushSlot(new NullControl("original heading"));
	help->pushSlot(new NullControl("whenever it is on"));
	help->pushSlot(new NullControl("the opposite side of"));
	help->pushSlot(new NullControl("the circle from"));
	help->pushSlot(new NullControl("where it started."));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("For pure rotations,"));
	help->pushSlot(new NullControl("tweak both strafe"));
	help->pushSlot(new NullControl("and forward so the"));
	help->pushSlot(new NullControl("rotation is centered"));
	help->pushSlot(new NullControl("around the middle of"));
	help->pushSlot(new NullControl("the body.  Take"));
	help->pushSlot(new NullControl("displacement"));
	help->pushSlot(new NullControl("readings for other"));
	help->pushSlot(new NullControl("measurements"));
	help->pushSlot(new NullControl("relative to this"));
	help->pushSlot(new NullControl("point."));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("Enter angular"));
	help->pushSlot(new NullControl("measurements in"));
	help->pushSlot(new NullControl("degrees.  Enter"));
	help->pushSlot(new NullControl("distances as "));
	help->pushSlot(new NullControl("centimeters."));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("You will almost"));
	help->pushSlot(new NullControl("always enter"));
	help->pushSlot(new NullControl("positive values for"));
	help->pushSlot(new NullControl("all measurements."));
	help->pushSlot(new NullControl("Enter negative"));
	help->pushSlot(new NullControl("values only when"));
	help->pushSlot(new NullControl("using cartesian"));
	help->pushSlot(new NullControl("measurements with a"));
	help->pushSlot(new NullControl("right strafe or"));
	help->pushSlot(new NullControl("backward motion, or"));
	help->pushSlot(new NullControl("when a rotation is"));
	help->pushSlot(new NullControl("opposite of the"));
	help->pushSlot(new NullControl("intended direction."));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("Remember that"));
	help->pushSlot(new NullControl("POSITIVE strafe is"));
	help->pushSlot(new NullControl("to the Aibo's LEFT."));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("Finally, when you"));
	help->pushSlot(new NullControl("have taken at least"));
	help->pushSlot(new NullControl("6 (I suggest at"));
	help->pushSlot(new NullControl("least 16)"));
	help->pushSlot(new NullControl("measurements of each"));
	help->pushSlot(new NullControl("type, you can save"));
	help->pushSlot(new NullControl("the data set."));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("You can also save"));
	help->pushSlot(new NullControl("intermediate sets"));
	help->pushSlot(new NullControl("and concatenate them"));
	help->pushSlot(new NullControl("later."));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("IMPORTANT: It is"));
	help->pushSlot(new NullControl("recommended to keep"));
	help->pushSlot(new NullControl("a telnet connection"));
	help->pushSlot(new NullControl("to port 59000 open"));
	help->pushSlot(new NullControl("so you can log data"));
	help->pushSlot(new NullControl("samples there, in"));
	help->pushSlot(new NullControl("case of an"));
	help->pushSlot(new NullControl("unexpected crash,"));
	help->pushSlot(new NullControl("freeze, or shutdown."));
	help->pushSlot(new NullControl("Each data sample is"));
	help->pushSlot(new NullControl("a line of 6 numbers."));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("You will then need"));
	help->pushSlot(new NullControl("to run the matlab"));
	help->pushSlot(new NullControl("function in"));
	help->pushSlot(new NullControl("tools/WalkCalibration.m"));
	help->pushSlot(new NullControl("(Linear Least"));
	help->pushSlot(new NullControl("Squares) on these"));
	help->pushSlot(new NullControl("files to get the"));
	help->pushSlot(new NullControl("calibration matrix."));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("Copy this matrix"));
	help->pushSlot(new NullControl("back on to the"));
	help->pushSlot(new NullControl("memory stick and use"));
	help->pushSlot(new NullControl("the \"Load"));
	help->pushSlot(new NullControl("Calibration\""));
	help->pushSlot(new NullControl("control to apply it"));
	help->pushSlot(new NullControl("to the current walk."));
	help->pushSlot(new NullControl(""));
	help->pushSlot(new NullControl("You will then need"));
	help->pushSlot(new NullControl("to save the walk"));
	help->pushSlot(new NullControl("itself to retain the"));
	help->pushSlot(new NullControl("calibration for the"));
	help->pushSlot(new NullControl("next reboot."));


	load=new FileInputControl("Load Data Set","Load data files - select any file from the set, all will be loaded","/");
	save=new StringInputControl("Save Data Set","Saves current data","Enter the base name (up to 6 char)");
	measure=new NullControl("Take Measurements","Begins the data gathering process");
	measure->pushSlot(new NullControl());
	clear=new NullControl("Clear Data","Clear the current data and start over");
	clear->pushSlot(new NullControl());
	for(unsigned int i=0; i<NUM_SRC; i++)
		cnts[i]=0;
	setupRoot();
}


WalkCalibration::~WalkCalibration() {
	options.clear();
	delete help;
	delete load;
	delete save;
	delete measure;
	delete clear;
}

ControlBase * WalkCalibration::activate(MC_ID disp_id, Socket * gui) {
	erouter->addListener(this,EventBase::locomotionEGID);
	return ControlBase::activate(disp_id, gui);
}


void WalkCalibration::refresh() {
	if(load->getLastInput()!=lastLoad) {
		for(int i=0; i<NUM_SRC; i++) {
			unsigned int strlen=load->getLastInput().size();
			loadData(load->getLastInput().substr(0,strlen-6)+datanames[i]+std::string(".txt"),data[i]);
			cnts[i]=data[i].size();
		}
		lastLoad=load->getLastInput();
	}
	if(save->getLastInput().size()>0) {
		for(int i=0; i<NUM_SRC; i++)
			saveData(save->getLastInput()+datanames[i]+std::string(".txt"),data[i]);
		save->takeInput("");
	}
	if(st==READY && (curType==fs || curType==bs))
		isPolar=polar->getStatus();
	ControlBase::refresh();
	if(gui_comm!=NULL && wireless->isConnected(gui_comm->sock)) {
		if(status.size()==0)
			gui_comm->printf("status\n1\n# Samples: fs=%d fr=%d sr=%d br=%d bs=%d r=%d\n",cnts[0], cnts[1], cnts[2], cnts[3], cnts[4], cnts[5]);
		else
			gui_comm->printf("status\n%td\n%s\n",std::count(status.begin(),status.end(),'\n'),status.c_str());
	}
}

ControlBase* WalkCalibration::doSelect() {
	if(st==ROOT) {
		if(hilights.size()==0)
			return this;
		if(options[hilights.front()]==measure) {
			sndman->playFile(config->controller.select_snd);
			setupChoose();
			refresh();
			return this;
		} else if(options[hilights.front()]==clear) {
			sndman->playFile(config->controller.select_snd);
			setupClear();
			refresh();
			return this;
		} else {
			return ControlBase::doSelect();
		}
	} else if(st==CLEAR) {
		if(hilights.size()==0)
			return this;
		sndman->playFile(config->controller.select_snd);
		if(hilights.front()!=0) {
			sout->printf("Clearing data...\n");
			sndman->playFile(config->controller.cancel_snd);
			for(int i=0; i<NUM_SRC; i++) {
				clearData(data[i]);
				cnts[i]=0;
			}
		}
		setupRoot();
		refresh();
		return this;
	} else if(st==CHOOSE) {
		if(hilights.size()==0)
			return this;
		if(hilights.front()-2<NUM_SRC) {
			curType=static_cast<dataSource>(hilights.front()-2);
			sndman->playFile(config->controller.select_snd);
			setupReady();
			refresh();
			return this;
		}
		sndman->playFile(config->controller.cancel_snd);
		setupRoot();
		refresh();
		return this;
	} else if(st==READY) {
		if(hilights.size()==0 || options[hilights.front()]->getName()=="Go!") {
			sndman->playFile(config->controller.select_snd);
			setupMoving();
			refresh();
			return this;
		}
		if(options[hilights.front()]==polar || options[hilights.front()]==rect) {
			return ControlBase::doSelect();
		}
		sndman->playFile(config->controller.cancel_snd);
		setupChoose();
		refresh();
		return this;
	} else {
		sndman->playFile(config->controller.cancel_snd);
		setupChoose();
		refresh();
		return this;
	}
}

void WalkCalibration::processEvent(const EventBase& e) {
	if(st==MOVING) {
		stopTime=e.getTimeStamp();
		sout->printf("Ran for %g seconds\n",(stopTime-startTime)/1000.f);
		sndman->playFile(config->controller.select_snd);
		setupReading1();
		refresh();
		if(curType!=5)
			doReadStdIn(std::string("Enter ")+getFirstMeasure(curType));
		else
			doReadStdIn(std::string("Enter ")+getSecondMeasure(curType));
	}
}
 

ControlBase * WalkCalibration::takeInput(const std::string& msg) {
	if(st!=READING_1 && st!=READING_2)
		return ControlBase::takeInput(msg);
	else {
		char * end;
		const char * str=msg.c_str();
		if(st==READING_1) {
			firstIn=(float)strtod(str,&end);
			if(end==str) {
				err("Invalid input: "+msg);
				return doReadStdIn(std::string("Enter ")+getFirstMeasure(curType));
			}
			sndman->playFile(config->controller.select_snd);
			setupReading2();
			refresh();
			if(*end!='\0')
				return takeInput(end);
			else
				return doReadStdIn(std::string("Enter ")+getSecondMeasure(curType));
		}
		if(st==READING_2) {
			secondIn=(float)strtod(str,&end);
			if(end==str) {
				err("Invalid input: "+msg);
				return doReadStdIn(std::string("Enter ")+getSecondMeasure(curType));
			}
			addSample();
			sndman->playFile(config->controller.select_snd);
			setupReady();
			refresh();
			return this;
		}
		return NULL;
	}
}
 

void WalkCalibration::setHilights(const std::vector<unsigned int>& hi) {
	if(st==ROOT || st==CLEAR)
		ControlBase::setHilights(hi);
	else {
		std::vector<unsigned int> h;
		for(unsigned int i=0; i<hi.size(); i++)
			if((options[hi[i]]!=NULL && options[hi[i]]->slotsSize()>0) || (st==READY && (options[hi[i]]==polar || options[hi[i]]==rect)))
				h.push_back(hi[i]);
		ControlBase::setHilights(h);
	}
}
 

void WalkCalibration::hilightFirst() {
	if(st==ROOT || st==CLEAR)
		ControlBase::hilightFirst();
	else {
		std::vector<unsigned int> h;
		for(unsigned int i=0; i<options.size(); i++)
			if(options[i]!=NULL && options[i]->slotsSize()>0) {
				h.push_back(i);
				break;
			}
		ControlBase::setHilights(h);		
	}
}
 
void WalkCalibration::loadData(const std::string& n, std::vector<float*>& dat) {
	clearData(dat);
	sout->printf("Loading data from '%s'...\n",n.c_str());
	std::ifstream in(n.c_str());
	if(!in) {
		err("Could not open file for saving");
		return;
	}
	while(in) {
		dat.push_back(new float[6]);
		for(unsigned int c=0; c<6; c++) {
			in >> dat.back()[c];
			if(!in) {
				if(c!=0)
					err("Data file ended mid row");
				break;
			}
		}
	}
	delete [] dat.back();
	dat.pop_back();
	in.close();
}

void WalkCalibration::saveData(const std::string& n, const std::vector<float*>& dat) {
	std::string p=config->portPath(n);
	sout->printf("Saving data to '%s'...\n",p.c_str());
	std::ofstream out(p.c_str());
	if(!out) {
		err("Could not open file for saving");
		return;
	}
	unsigned int i=0;
	for(; i<dat.size(); i++) {
		for(unsigned int c=0; c<6; c++)
			out << dat[i][c] << ' ';
		out << endl;
		if(!out) {
			err("It appears the memory stick ran out of space?");
			break;
		}
	}
	out.close();
	sout->printf("%d rows written\n",i);
}

void WalkCalibration::clearData(std::vector<float*>& dat) {
	for(unsigned int i=0; i<dat.size(); i++)
		delete [] dat[i];
	dat.clear();
}

void WalkCalibration::setupRoot() {
	clearSlots();
	setName("Interactive Calibration");
	status="";
	pushSlot(help);
	pushSlot(load);
	pushSlot(save);
	pushSlot(measure);
	pushSlot(clear);
	st=ROOT;
}

void WalkCalibration::setupChoose() {
	bool isFromRoot=(st==ROOT);
	if(isFromRoot) {
		options.clear();
		hilights.clear();
	} else
		clearSlots();
	pushSlot(new NullControl("Choose Category"));
	pushSlot(NULL);
	NullControl * tmp;
	for(unsigned int i=0; i<NUM_SRC; i++) {
		dataSource s=static_cast<dataSource>(i);
		if(s==r)
			pushSlot(tmp=new NullControl(getIndexName(getSecondIndex(s))));
		else
			pushSlot(tmp=new NullControl(getIndexName(getFirstIndex(s))+std::string("/")+getIndexName(getSecondIndex(s))));
		tmp->pushSlot(NULL);
	}
	pushSlot(NULL);
	pushSlot(tmp=new NullControl(isFromRoot?"Cancel":"Done"));
	tmp->pushSlot(NULL);
	st=CHOOSE;
}

void WalkCalibration::setupReady() {
	polar=rect=NULL;
	clearSlots();
	pushSlot(new NullControl("Ready to record"));
	if(curType==r)
		pushSlot(new NullControl(string_util::makeUpper(getIndexName(getSecondIndex(curType)))));
	else
		pushSlot(new NullControl(string_util::makeUpper(getIndexName(getFirstIndex(curType)))+std::string("/")+string_util::makeUpper(getIndexName(getSecondIndex(curType)))));
	pushSlot(NULL);
	switch(curType) {
	case fs:
		pushSlot(new NullControl("Choose any forward"));
		pushSlot(new NullControl("and sideways vel."));
		pushSlot(new NullControl("Then tweak rotation"));
		pushSlot(new NullControl("to maintain heading"));
		polar=new ToggleControl("Take polar measurements","Check this if you want to take measurements as displacement,heading",new ToggleControl::RadioGroup());
		rect=new ToggleControl("Take cartesian measurements","Check this if you want to take measurements as x,y",polar->getRadioGroup());
		if(isPolar)
			polar->setStatus(true);
		else
			rect->setStatus(true);
		pushSlot(polar);
		pushSlot(rect);
		break;
	case fr:
		pushSlot(new NullControl("Choose any rotational"));
		pushSlot(new NullControl("and forward vel."));
		pushSlot(new NullControl("Tweak sideways vel."));
		pushSlot(new NullControl("to line up heading"));
		pushSlot(new NullControl("on opposite side of"));
		pushSlot(new NullControl("circle."));
		break;
	case sr:
		pushSlot(new NullControl("Choose any sideways"));
		pushSlot(new NullControl("vel.  Tweak forward"));
		pushSlot(new NullControl("vel. to cancel drift"));
		pushSlot(new NullControl("Then choose any"));
		pushSlot(new NullControl("rotational vel."));
		break;
	case br:
		pushSlot(new NullControl("Choose any rotational"));
		pushSlot(new NullControl("and sideways vel."));
		pushSlot(new NullControl("Tweak forward vel."));
		pushSlot(new NullControl("to line up heading"));
		pushSlot(new NullControl("on opposite side of"));
		pushSlot(new NullControl("circle."));
		break;
	case bs:
		pushSlot(new NullControl("Choose any forward"));
		pushSlot(new NullControl("and sideways vel."));
		pushSlot(new NullControl("Then tweak rotation"));
		pushSlot(new NullControl("to maintain heading"));
		polar=new ToggleControl("Take polar measurements","Check this if you want to take measurements as displacement,heading",new ToggleControl::RadioGroup());
		rect=new ToggleControl("Take cartesian measurements","Check this if you want to take measurements as x,y",polar->getRadioGroup());
		polar->setStatus(true);
		if(isPolar)
			polar->setStatus(true);
		else
			rect->setStatus(true);
		pushSlot(polar);
		pushSlot(rect);
		break;
	case r:
		pushSlot(new NullControl("Choose any"));
		pushSlot(new NullControl("rotational vel."));
		pushSlot(new NullControl("Tweak others to"));
		pushSlot(new NullControl("center rotation"));
		break;
	case NUM_SRC:
		{
			pushSlot(new NullControl("An error has occured."));
			pushSlot(new NullControl("Please go back and"));
			pushSlot(new NullControl("try again"));
			pushSlot(NULL);
			NullControl * tmp;
			pushSlot(tmp=new NullControl((st==CHOOSE)?"Cancel":"Done"));
			tmp->pushSlot(new NullControl());
			st=READY;
			return;
		}
	}
	pushSlot(NULL);
	NullControl * tmp;
	pushSlot(tmp=new NullControl("Go!"));
	tmp->pushSlot(new NullControl());
	pushSlot(tmp=new NullControl((st==CHOOSE)?"Cancel":"Done"));
	tmp->pushSlot(new NullControl());
	st=READY;
}

void WalkCalibration::setupMoving() {
	old_x=state->vel_x;
	old_y=state->vel_y;
	old_a=state->vel_a;
	if(old_x+old_y+old_a==0) {
		err("The WorldState velocities are all 0.\n\n"
				"Make sure whatever code is causing the motion is throwing\n"
				"a LocomotionEvent, and that the EStop is off.  You can use\n"
				"the EventLogger to check for LocomotionEvents.");
		return;
	}
	switch(curType) {
	case fs:
	case fr:
		if(old_x<0) {
			err("Have to be moving FORWARD for this category");
			return;
		}
		break;
	case bs:
	case br:
		if(old_x>0) {
			err("Have to be moving BACKWARD for this category");
			return;
		}
		break;
	case NUM_SRC:
		err("Bad category");
		return;
	case sr:
	case r:
		;
	}
	sout->printf("Velocity is: %g %g %g\n",old_x,old_y,old_a);
	clearSlots();
	pushSlot(new NullControl("Running..."));
	if(curType!=5)
		pushSlot(new NullControl(std::string(getIndexName(getFirstIndex(curType)))+"/"+std::string(getIndexName(getSecondIndex(curType)))));
	else
		pushSlot(new NullControl(getIndexName(getSecondIndex(curType))));
	pushSlot(NULL);
	pushSlot(new NullControl("Recording will stop"));
	pushSlot(new NullControl("on the next"));
	pushSlot(new NullControl("LocomotionEvent."));
	pushSlot(new NullControl("EStop is recommended."));
	char str[255];
	sprintf(str,"vel_x=%g",old_x);
	pushSlot(new NullControl(str));
	sprintf(str,"vel_y=%g",old_y);
	pushSlot(new NullControl(str));
	sprintf(str,"vel_a=%g",old_a);
	pushSlot(new NullControl(str));
	pushSlot(NULL);
	NullControl * tmp;
	pushSlot(tmp=new NullControl("Cancel"));
	tmp->pushSlot(new NullControl());
	st=MOVING;
	startTime=get_time();
}

void WalkCalibration::setupReading1() {
	if(curType==5) {
		setupReading2();
		return;
	}
	clearSlots();
	pushSlot(new NullControl("Reading first input"));
	pushSlot(NULL);
	pushSlot(new NullControl("Please enter:"));
	pushSlot(new NullControl(getFirstMeasure(curType)));
	pushSlot(NULL);
	NullControl * tmp;
	pushSlot(tmp=new NullControl("Cancel"));
	tmp->pushSlot(new NullControl());
	st=READING_1;
}

void WalkCalibration::setupReading2() {
	clearSlots();
	if(curType==5)
		pushSlot(new NullControl("Reading input"));
	else
		pushSlot(new NullControl("Reading second input"));
	pushSlot(NULL);
	pushSlot(new NullControl("Please enter:"));
	pushSlot(new NullControl(getSecondMeasure(curType)));
	pushSlot(NULL);
	NullControl * tmp;
	pushSlot(tmp=new NullControl("Cancel"));
	tmp->pushSlot(new NullControl());
	st=READING_2;
}



void WalkCalibration::setupClear() {
	options.clear();
	setName("Clear Data");
	char tmp[255];
	sprintf(tmp,"There are %d samples.  Are you sure?",cnts[0]+cnts[1]+cnts[2]+cnts[3]+cnts[4]+cnts[5]);
	status=tmp;
	pushSlot(new NullControl("No","Don't clear the data"));
	pushSlot(new NullControl("Yes","Start over"));
	hilightFirst();
	st=CLEAR;
}

unsigned int WalkCalibration::getFirstIndex(WalkCalibration::dataSource t) {
	switch(t) {
	case fs:
	case fr:
		return 0;
	case sr:
		return 1;
	case bs:
	case br:
		return 3;
	case r:
	case NUM_SRC:
		return -1U;
	}
	return -1U;
}

unsigned int WalkCalibration::getSecondIndex(WalkCalibration::dataSource t) {
	switch(t) {
	case fs:
	case bs:
		return 1;
	case fr:
	case sr:
	case br:
	case r:
		return 2;
	case NUM_SRC:
		return -1U;
	}
	return -1U;
}

const char * WalkCalibration::getIndexName(unsigned int t) {
	switch(t) {
	case 0:
		return "Forward";
	case 1:
		return "Strafe";
	case 2:
		return "Rotate";
	case 3:
		return "Backward";
	}
	return NULL;
}

const char * WalkCalibration::getFirstMeasure(WalkCalibration::dataSource t) {
	switch(t) {
	case fs:
		if(isPolar)
			return "displacement";
		else
			return "forward distance";
	case fr:
	case sr:
	case br:
		return "displacement";
	case bs:
		if(isPolar)
			return "displacement";
		else
		return "backward distance";
	case r:
		return NULL;
	case NUM_SRC:
		return NULL;
	}
	return NULL;
}

const char * WalkCalibration::getSecondMeasure(WalkCalibration::dataSource t) {
	switch(t) {
	case fs:
	case bs:
		if(isPolar)
			return "bearing";
		else
			return "sideways distance";
	case fr:
	case sr:
	case br:
	case r:
		return "angular distance";
	case NUM_SRC:
		return NULL;
	}
	return NULL;
}

float WalkCalibration::arclen(float d, float a, float sign) {
	if(fabs(a)<=1) //if the angle is small (in particular, 0) leads to numerical instability.
		return (sign<0)?-fabs(d):fabs(d);
	//otherwise, convert to radians
	a*=(float)M_PI/180;
	//calculate radius of the circle we're tracing (there's some simple geometry to get this)
	float radius=fabs(d/2/sin(a/2));
	//and return the length of the sector we've traced (with the requested sign)
	if(sign<0)
		return -a*radius;
	return a*radius;
}


//this will allow you to solve for the conversion from commands to actual
//which I was planning to then invert for the conversion from desired to command
//but then I realized I could just solve for the inverted for directly
// (never was actually tested btw, and also out of date with rest of this code)
/*
void WalkCalibration::addSample() {
	switch(getType()) {
	case 0: //fs
		addSample(forward,0, firstIn*10);
		addSample(forward,1, secondIn*10);
		addSample(forward,2, 0);
		break;
	case 1: //fr
		addSample(forward,0, arclen(firstIn*10,secondIn, 1));
		addSample(forward,1, 0);
		addSample(forward,2, (old_a>0?secondIn:-secondIn)*M_PI/180);
		break;
	case 2: //sr
		addSample(forward,0, 0);
		addSample(reverse,0, 0);
		addSample(forward,1, arclen(firstIn*10,secondIn,old_y));
		addSample(reverse,1, arclen(firstIn*10,secondIn,old_y));
		addSample(forward,2, (old_a>0?secondIn:-secondIn)*M_PI/180);
		addSample(reverse,2, (old_a>0?secondIn:-secondIn)*M_PI/180);
		break;
	case 3: //br
		addSample(reverse,0, arclen(firstIn*10,secondIn,-1));
		addSample(reverse,1, 0);
		addSample(reverse,2, (old_a>0?secondIn:-secondIn)*M_PI/180);
		break;
	case 4: //bs
		addSample(reverse,0, firstIn*10);
		addSample(reverse,1, secondIn*10);
		addSample(reverse,2, 0);
		break;
	case 5: //r
		addSample(forward,2, (old_a>0?secondIn:-secondIn)*M_PI/180);
		addSample(reverse,2, (old_a>0?secondIn:-secondIn)*M_PI/180);
		break;
	}
	cnts[getType()]++;
}
*/

void WalkCalibration::addSample() {
	float dx=0, dy=0, da=0;
	float dt=(stopTime-startTime)/1000.f;
	switch(curType) {
	case fs:
		if(isPolar) {
			dx=firstIn*std::cos(secondIn*(float)M_PI/180)*10/dt;
			dy=firstIn*std::sin(secondIn*(float)M_PI/180)*10/dt;
		} else {
			dx=firstIn*10/dt;
			dy=secondIn*10/dt;
		}
		addSample(data[curType], dx,dy,da);
		report(0,old_x,dx);
		report(1,old_y,dy);
		break;
	case fr:
		dx=arclen(firstIn*10,secondIn, 1)/dt;
		da=(old_a>0?secondIn:-secondIn)*(float)M_PI/180/dt;
		addSample(data[curType], dx,dy,da);
		report(0,old_x,dx);
		report(2,old_a,da);
		break;
	case sr:
		dy=arclen(firstIn*10,secondIn,old_y)/dt;
		da=(old_a>0?secondIn:-secondIn)*(float)M_PI/180/dt;
		addSample(data[curType], dx,dy,da);
		report(1,old_y,dy);
		report(2,old_a,da);
		break;
	case br:
		dx=arclen(firstIn*10,secondIn,-1)/dt;
		da=(old_a>0?secondIn:-secondIn)*(float)M_PI/180/dt;
		addSample(data[curType], dx,dy,da);
		report(0,old_x,dx);
		report(2,old_a,da);
		break;
	case bs:
		if(isPolar) {
			dx=firstIn*cos(secondIn*(float)M_PI/180)*10/dt;
			dy=firstIn*sin(secondIn*(float)M_PI/180)*10/dt;
		} else {
			dx=firstIn*10/dt;
			dy=secondIn*10/dt;
		}
		addSample(data[curType], dx,dy,da);
		report(0,old_x,dx);
		report(1,old_y,dy);
		break;
	case r:
		da=(old_a>0?secondIn:-secondIn)*(float)M_PI/180/dt;
		addSample(data[curType], dx,dy,da);
		report(2,old_a,da);
		break;
	default:
		serr->printf("unknown type! %d\n",curType);
		return;
	}
	cnts[curType]++;
}

void WalkCalibration::report(unsigned int row, float cmd, float actual) {
	switch(row) {
	case 0:
		sout->printf("Forwrd command=%6.4g  Actual=%6.4g\n",cmd,actual); break;
	case 1:
		sout->printf("Strafe command=%6.4g  Actual=%6.4g\n",cmd,actual); break;
	case 2:
		sout->printf("Rotate command=%6.4g  Actual=%6.4g\n",cmd,actual); break;
	}
}
 
void WalkCalibration::err(const std::string& str) {
		std::vector<std::string> errmsg;
		errmsg.push_back("Error");
		errmsg.push_back(str);
		serr->printf("%s\n",str.c_str());
		Controller::loadGUI("org.tekkotsu.mon.ControllerErr","msg",0,errmsg);
		sndman->playFile(config->controller.error_snd);
		return;
}

void WalkCalibration::addSample(std::vector<float*>& dat, float x, float y, float a) {
	float * d=new float[6];
	dat.push_back(d);
	d[0]=x;
	d[1]=y;
	d[2]=a;
	d[3]=old_x;
	d[4]=old_y;
	d[5]=old_a;

	for(unsigned int i=0; i<6; i++)
		cout << d[i] << ' ';
	cout << endl;
}
	

/*! @file
 * @brief 
 * @author ejt (Creator)
 */

