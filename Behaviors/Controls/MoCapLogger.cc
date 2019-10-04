#include "MoCapLogger.h"

REGISTER_CONTROL(MoCapLogger,"Status Reports");

void MoCapLogger::rename(ControlBase* c, std::ostream& sstream) {
	std::stringstream& ss = dynamic_cast<std::stringstream&>(sstream);
	c->setName(ss.str());
	ss.str("");
}

/*! This section demonstrates the getPosition/getOrientation interface
 *  (with exception handling in case frame is not available in current update)
 *  See dump() below for iteration through all provided frames. */
void MoCapLogger::gotMoCapGUI(const MoCapEvent& mce) {
	try {
		fmat::Column<3> p = mce.getPosition(BaseFrameOffset);
		std::stringstream ss;
		ss.precision(1);
		ss.setf(std::ios::fixed);
		rename(x, ss << "X: " << p[0]);
		rename(y, ss << "Y: " << p[1]);
		rename(z, ss << "Z: " << p[2]);
		try {
			fmat::Quaternion q = mce.getOrientation(BaseFrameOffset);
			if(rotAxis->getStatus()) {
				ss.precision(3);
				rename(r0, ss << "X: " << q.getX());
				rename(r1, ss << "Y: " << q.getY());
				rename(r2, ss << "Z: " << q.getZ());
			} else {
				fmat::Column<3> ypr = q.ypr() * 180/(fmat::fmatReal)M_PI;
				rename(r0, ss << "Heading: " << ypr[0] << "°");
				rename(r1, ss << "Pitch: " << ypr[1] << "°");
				rename(r2, ss << "Roll: " << ypr[2] << "°");
			}
		} catch(const std::exception&) {
			r0->setName("");
			r1->setName("");
			r2->setName("");
		}
	} catch(const std::exception&) {
		x->setName("X: No BaseFrame");
		y->setName("Y: No BaseFrame");
		z->setName("Z: No BaseFrame");
	}
	if(lastRefresh.Age().Value()>=0.5)
	   refresh();
}

void MoCapLogger::gotMoCapConsole(const MoCapEvent& mce) {
	dump(std::cout,mce);
}

void MoCapLogger::gotMoCapFile(const MoCapEvent& mce) {
	dump(file,mce);
}

void MoCapLogger::gotTxtMsgSingle(const TextMsgEvent& txt) {
	if(txt.getText()=="mocap") {
		erouter->addListener(&mocapSingle, EventBase::mocapEGID);
		erouter->addTimer(&mocapSingle, 0, 2000, false);
	}
}

void MoCapLogger::gotMoCapSingle(const EventBase& event) {
	std::cout << '\n'; // probably has the prompt displayed, start a new line
	if(const MoCapEvent* mce = dynamic_cast<const MoCapEvent*>(&event)) {
		dump(std::cout,*mce);
		erouter->remove(&mocapSingle);
	} else {
		std::cerr << "MoCapLogger has not received a MoCapEvent yet, will continue to wait... (check driver configuration?)" << std::endl;
	}
}

/*! Only logs frame which have a position component, checks to see if they happen to also have an orientation */
void MoCapLogger::dump(std::ostream& os, const MoCapEvent& mce) {
	for(MoCapEvent::position_iterator it=mce.positions.begin(); it!=mce.positions.end(); ++it) {
		os << mce.getTimeStamp() << '\t' << outputNames[it->first] << '\t' << it->second[0] << '\t' << it->second[1] << '\t' << it->second[2];
		MoCapEvent::orientation_iterator oit = mce.orientations.find(it->first);
		if(oit!=mce.orientations.end()) {
			if(rotAxis->getStatus()) {
				os << '\t' << oit->second.getX() << '\t' << oit->second.getY() << '\t' << oit->second.getZ();
			} else {
				fmat::Column<3> ypr = oit->second.ypr() * 180/(fmat::fmatReal)M_PI;
				os << '\t' << ypr[0] << '\t' << ypr[1] << '\t' << ypr[2];
			}
		}
		os << '\n';
	}
}

void MoCapLogger::refresh() {
	if(mocapGUI.get()==NULL) {
		mocapGUI.reset(new EventCallbackAs<MoCapEvent>(&MoCapLogger::gotMoCapGUI, *this));
		erouter->addListener(mocapGUI.get(),EventBase::mocapEGID);
	} else {
		lastRefresh.Set();
	}
	
	// has console log status changed?
	if(consoleLog->getStatus()) {
		if(mocapConsole.get()==NULL) {
			mocapConsole.reset(new EventCallbackAs<MoCapEvent>(&MoCapLogger::gotMoCapConsole, *this));
			erouter->addListener(mocapConsole.get(),EventBase::mocapEGID);
		}
	} else {
		mocapConsole.reset();
	}
	
	// has file log status changed? (ignore is triggered on entry to fileLog, this is on return)
	if(fileLog->getLastInput().size()>0) {
		file.open(fileLog->getLastInput().c_str());
		if(file) {
			fileLog->setName("Logging to " + fileLog->getLastInput());
			mocapFile.reset(new EventCallbackAs<MoCapEvent>(&MoCapLogger::gotMoCapFile, *this));
			erouter->addListener(mocapFile.get(),EventBase::mocapEGID);
		} else {
			std::cerr << "Could not open " << fileLog->getLastInput() << std::endl;
		}
		fileLog->clearLastInput();
	}
	ControlBase::refresh();
}

void MoCapLogger::deactivate() {
	mocapGUI.reset();
	ControlBase::deactivate();
}

ControlBase * MoCapLogger::doSelect() {
	ControlBase * c = ControlBase::doSelect();
	if(c==fileLog) {
		file.close();
		mocapFile.reset();
		fileLog->setName("Log To File");
	}
	if(c!=this)
		mocapGUI.reset(); // don't refresh display from background when child active
	return c;
}


/*! @file
 * @brief Implements MoCapLogger, which provides display and logging of mocap data
 * @author ejt (Creator)
 */
