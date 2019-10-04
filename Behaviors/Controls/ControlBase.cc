#include "ControlBase.h"
#include "Motion/MMAccessor.h"
#include "Shared/Config.h"
#ifdef TGT_HAS_LEDS
#  include "Motion/LedMC.h"
#endif
#include "Shared/string_util.h"
#include "Sound/SoundManager.h"
#include "Wireless/Wireless.h"
#include <iomanip>
#include <sstream>

ControlBase * ControlBase::activate(MC_ID display, Socket * gui) {
	display_id=display;
	gui_comm=gui;
	hilightFirst();
	refresh();
	return this;
}

void ControlBase::pause() {
	if(doRewrite) {
		doRewrite=false;
	}
}

void ControlBase::refresh() {
	//Level 1: LEDs
#ifdef TGT_HAS_LEDS
	if(display_id!=invalid_MC_ID) {
		if(hilights.size()>0) {
			MMAccessor<LedMC> display(display_id);
			unsigned int cur=hilights.front();
			if(options.size()<=10)
				display.mc()->displayNumber(cur,LedEngine::onedigit);
			else
				display.mc()->displayNumber(cur,LedEngine::twodigit);
		}
	}
#endif

	//Just do one of the other two levels to avoid over-redundancy
	if(gui_comm==NULL || !wireless->isConnected(gui_comm->sock)) {
#ifndef PLATFORM_APERIOS
		if(!wireless->isConnected(sout->sock))
			return;
#endif
		//Level 2: Console
		const char * const nosel="  ";
		const char * slctd="**";
		if(config->main.use_VT100) {
			slctd="\33[1m**\33[0m";
			if(doRewrite)
				clearMenu();
			sout->printf("\33[1m%s\33[0m:\n",getName().c_str()); //displays name in bold
		} else
			sout->printf("%s:\n",getName().c_str());
		unsigned int digits=0;
		for(unsigned int i=1; i<options.size(); i*=10)
			digits++;
		for(unsigned int i=0; i<options.size(); i++) {
			if(options[i]==NULL)
				for(unsigned int j=0; j<strlen(nosel)+digits+2; j++)
					sout->printf(" ");
			else
				sout->printf("%s%*d%s ",(find(hilights.begin(),hilights.end(),i)!=hilights.end()?slctd:nosel),digits,i,(options[i]->slotsSize()>0?">":"."));
			//cout << (find(hilights.begin(),hilights.end(),i)!=hilights.end()?slctd:nosel) << std::setw(digits) << i << "> ";
			sout->printf("%s\n",getSlotName(i).c_str());
		}
		if(options.size()==0)
			sout->printf("  Empty menu\n");
		doRewrite=true;
	} else {
		//Level 3: GUI

		//		cout << "REFRESHING " << getName() << endl;
		//try to get it all in one packet for better performance
		std::stringstream ss;
		ss << "refresh\n"
			 << getName() << '\n'
			 << options.size() << '\n';
		for(unsigned int i=0; i<options.size(); i++) {
			if(options[i]==NULL)
				ss << "0\n0\n----------\n0\n\n";
			else {
				std::string desc = options[i]->getDescription();
				ss << options[i]->options.size() << '\n'
					 << (binary_search(hilights.begin(),hilights.end(),i)?1:0) << '\n'
					 << options[i]->getName() << '\n'
					 << std::count(desc.begin(),desc.end(),'\n') << '\n'
					 << desc << '\n';
			}
		}
		//		do {
		//cout << "Writing " << ss.str().size() << "...";
		gui_comm->write((const byte*)ss.str().c_str(),ss.str().size());
		//		int cnt=gui_comm->printf("%s",(const byte*)ss.str().c_str());
		//cout << "wrote " << cnt << endl;
		//} while(cnt==-1);
	}
}

void ControlBase::deactivate() {
	hilights.clear();
#ifdef TGT_HAS_LEDS
	if(display_id!=invalid_MC_ID) {
		MMAccessor<LedMC> display(display_id);
		display.mc()->clear();
	}
#endif
	if(doRewrite) {
		if(config->main.use_VT100)
			clearMenu();
		doRewrite=false;
	}
	display_id=invalid_MC_ID;
}

ControlBase* ControlBase::doSelect() {
	//		cout << "ControlBase::doSelect()" << endl;
	//		cout << "cur==" << cur << endl;
	if(hilights.size()==0) {
		sndman->playFile(config->controller.select_snd);
		return this;
	}
	for(unsigned int i=0;i<hilights.size();i++) {
		unsigned int cur=hilights[i];
		if(cur>=options.size() || options[cur]==NULL) {
#ifdef TGT_HAS_LEDS
			if(display_id!=invalid_MC_ID) {
				MMAccessor<LedMC> display(display_id);
				display.mc()->cflash(FaceLEDMask,.5f,100);
			}
#endif
			if(cur>=options.size())
				sout->printf("Invalid choice\n");
			else
				sout->printf("NULL option\n");
			continue;
		}
#ifdef TGT_HAS_LEDS
		if(display_id!=invalid_MC_ID) {
			MMAccessor<LedMC> display(display_id);
			display.mc()->flash(FaceLEDMask,100);
			display.mc()->clear();
		}
#endif
		if(doRewrite) {
			if(config->main.use_VT100)
				clearMenu();
			doRewrite=false;
		}
		sndman->playFile(config->controller.select_snd);
		if(hilights.size()>1) {
			options[cur]->activate(display_id,gui_comm);
			options[cur]->deactivate();
		}
	}
	if(hilights.size()==1)
		return options[hilights.front()];
	return this;
}

ControlBase* ControlBase::doNextItem() {
	//		cout << "ControlBase::doNextItem()" << endl;
	if(options.size()==0)
		return this;
	unsigned int cur=0;
	for(unsigned int i=0; i<hilights.size(); i++)
		if(hilights[i]>=cur)
			cur=(hilights[i]+1)%options.size();
	while(options[cur]==NULL)
		cur=(cur+1)%options.size();
	hilights.clear();
	hilights.push_back(cur);
	sndman->playFile(config->controller.next_snd);
	refresh();
	//		cout << "cur==" << cur << endl;
	return this;
}

ControlBase* ControlBase::doPrevItem() {
	//		cout << "ControlBase::doPrevItem()" << endl;
	if(options.size()==0)
		return this;
	unsigned int cur=options.size()-1;
	for(unsigned int i=hilights.size(); i>0; i--)
		if(hilights[i-1]<=cur)
			cur=(hilights[i-1]+options.size()-1)%options.size();
	while(options[cur]==NULL)
		cur=(cur+options.size()-1)%options.size();
	hilights.clear();
	hilights.push_back(cur);
	sndman->playFile(config->controller.prev_snd);
	refresh();
	//		cout << "cur==" << cur << endl;
	return this;
}

ControlBase * ControlBase::doCancel() {
	sndman->playFile(config->controller.cancel_snd);
	return NULL;
}


ControlBase* ControlBase::doReadStdIn(const std::string& prompt/*=std::string()*/) {
	//Level 1: Local
#ifdef TGT_HAS_LEDS
	if(display_id!=invalid_MC_ID) {
		MMAccessor<LedMC> display(display_id);
		display.mc()->cset(FaceLEDMask,.5f);
	}
#endif
	sndman->playFile(config->controller.read_snd);

	//Just do one of the other two
	if(gui_comm==NULL || !wireless->isConnected(gui_comm->sock)) {
		//Level 2: Console
		if(prompt.size()>0)
			sout->printf("%s\n",prompt.c_str());
		sout->printf("#> ");
		if(!wireless->isConnected(sout->sock)) {
			std::string choice;
			std::cin >> choice;
			std::cout << std::endl;
			return takeInput(choice);
		}
		return this;
	} else {
		//Level 3: GUI
		if(prompt.size()>0) {
			unsigned int lines=std::count(prompt.begin(),prompt.end(),'\n');
			gui_comm->printf("status\n%u\n%s\n",lines,prompt.c_str());
		}
		return this;
	}
}

ControlBase* ControlBase::takeInput(const std::string& str) {
	std::vector<std::string> args;
	std::vector<unsigned int> offsets;
	if(!string_util::parseArgs(str,args,offsets)) {
		serr->printf("ControlBase::takeInput(\"%s\") was not understood.\n",str.c_str());
		refresh();
		return this;
	} else {
		//are there valid arguments?
		if(args.size()==0) {
			refresh();
			return this;
		}
		//let's see if the first arg matches an option name (case sensitive)
		unsigned int choice=-1U;
		bool ambiguous=false;
		for(unsigned int i=0; i<options.size(); i++) {
			if(options[i]!=NULL && options[i]->name == args[0]) {
				if(choice==-1U)
					choice=i;
				else
					ambiguous=true;
			}
		}
		//let's see if the first arg matches an option name (case insensitive)
		if(!ambiguous && choice==-1U) {
			std::string argname=string_util::makeLower(args[0]);
			for(unsigned int i=0; i<options.size(); i++) {
				if(options[i]!=NULL) {
					std::string optname=string_util::makeLower(options[i]->name);
					if(optname.compare(0,argname.size(),argname)==0) {
						if(choice==-1U)
							choice=i;
						else
							ambiguous=true;
					}
				}
			}
		}
		//if we didn't find one already, try treating the arg as an index number
		if(ambiguous || choice==-1U) {
			char* endp=NULL;
			choice=strtol(args[0].c_str(),&endp,10);
			if(endp==NULL || endp==args[0].c_str()) {
				if(config->main.use_VT100) {
					sout->printf("\r\33[1A");
					clearMenu();
					doRewrite=false;
				}
				return invalidInput(str,ambiguous);
			} else if(choice<options.size() && options[choice]!=NULL) {
				ambiguous=false;
			} else {
				if(config->main.use_VT100) {
					sout->printf("\r\33[1A");
					clearMenu();
					doRewrite=false;
				}
				return invalidInput(str,ambiguous);
			}	
		}
		//see what we got...
		if(args.size()>1)
			return options[choice]->takeInput(str.substr(offsets[1]));
		else {
			hilights.clear();
			hilights.push_back(choice);
			return doSelect();
		}
	}
	/*	std::string msg;
	{unsigned int i=0; while(i<str.size() && isspace(str[i])) i++; msg=str.substr(i);}
	if(isdigit(msg[0])) {
		char* endp=NULL;
		unsigned int choice=strtol(msg.c_str(),&endp,10);
		if(endp==NULL) {
			if(config->main.use_VT100) {
				sout->printf("\r\33[1A");
				clearMenu();
				doRewrite=false;
			}
			serr->printf("ControlBase::takeInput(\"%s\") was not understood.\n",str.c_str());
			refresh();
			return this;
		} else if(choice<options.size() && options[choice]!=NULL) {
			hilights.clear();
			hilights.push_back(choice);
			return doSelect();
		} else {
			if(config->main.use_VT100) {
				sout->printf("\r\33[1A");
				clearMenu();
				doRewrite=false;
			}
			sout->printf("%d is not a valid selection\n",choice);
			refresh();
			return this;
		}	
	} else {
		if(config->main.use_VT100) {
			sout->printf("\r\33[1A");
			clearMenu();
			doRewrite=false;
		}
		serr->printf("ControlBase::takeInput(\"%s\") was not understood.\nPlease enter the number of the index - string entry is not supported yet\n",str.c_str());
		refresh();
		return this;		
		}*/
	return this; //should never get here, but gcc 4 thinks we can
}

bool ControlBase::validInput(const std::string& str) {
	unsigned int choice=atoi(str.c_str());
	return (choice<options.size() && options[choice]!=NULL);
}

ControlBase* ControlBase::findSlot(const std::string& slotName) const {
	//let's see if the first arg matches an option name (case sensitive)
	for(unsigned int i=0; i<options.size(); i++) {
		if(options[i]!=NULL && options[i]->name.compare(0,slotName.size(),slotName)==0)
			return options[i];
	}
	//let's see if the first arg matches an option name (case insensitive)
	std::string argname=string_util::makeLower(slotName);
	bool ambiguous=false;
	unsigned int choice=-1U;
	for(unsigned int i=0; i<options.size(); i++) {
		if(options[i]!=NULL) {
			std::string optname=string_util::makeLower(options[i]->name);
			if(optname.compare(0,argname.size(),argname)==0) {
				if(choice==-1U)
					choice=i;
				else
					ambiguous=true;
			}
		}
	}
	//if we didn't find one already, try treating the arg as an index number
	if(ambiguous || choice==-1U) {
		char* endp=NULL;
		choice=strtol(slotName.c_str(),&endp,10);
		if(endp==NULL || endp==slotName.c_str()) {
			return NULL;
		} else if(choice<options.size() && options[choice]!=NULL) {
			ambiguous=false;
		} else {
			return NULL;
		}
	}
	return ambiguous ? NULL : options[choice];
}

std::string ControlBase::getSlotName(unsigned int i) const {
	if(options[i]!=NULL)
		return options[i]->getName();
	else {
		static std::string dashes("----------");
		return dashes;
	}
}


void ControlBase::setSlot(unsigned int i,ControlBase* o) {
	while(options.size()<=i)
		options.push_back(NULL);
	options[i]=o;
}

void ControlBase::pushSlot(ControlBase* o) {
	options.push_back(o);
}

void ControlBase::clearSlots() {
	for(unsigned int i=0; i<options.size(); i++)
		delete options[i];
	options.clear();
	hilights.clear();
}

void ControlBase::setHilights(const std::vector<unsigned int>& hi) {
	float avg=hilightsAvg();
	hilights.clear();
	for(unsigned int i=0; i<hi.size(); i++)
		if(hi[i]<options.size())
			hilights.push_back(hi[i]);
	float newavg=hilightsAvg();
	if(avg!=-1 || newavg!=-1) {
		if(avg<=newavg)
			sndman->playFile(config->controller.next_snd);
		else
			sndman->playFile(config->controller.prev_snd);
	}
	refresh();
}

void ControlBase::hilightFirst() {
	hilights.clear();
	for(unsigned int i=0; i<options.size(); i++)
		if(options[i]!=NULL) {
			hilights.push_back(i);
			return;
		}
}

ControlBase* ControlBase::registerControllerEntry(ControlBase* c, const std::string& menu, int flags/*=ControlBase::DEFAULTS*/) {
	ControlRegistry_t& reg = getControllerEntries();
	bool isnew = reg.find(menu)==reg.end() || reg[menu].find(c->getName())==reg[menu].end();
	reg[menu][c->getName()] = std::make_pair(c,flags);
	if(!isnew)
		std::cerr << "Warning: duplicate registation for " << c->getName() << " in Controller menu " << menu << std::endl;
	return c;
}

void ControlBase::clearMenu() {
	if(config->main.use_VT100) {
		sout->printf("\r\33[%luA",(unsigned long)(options.size()+1)); //moves cursor up to beginning of menu
		sout->printf("\33[J"); //clears to end of screen
	}
}

ControlBase* ControlBase::invalidInput(const std::string& msg, bool ambiguous) {
	if(ambiguous)
		serr->printf("ControlBase(\"%s\")::takeInput(\"%s\") was ambiguous\n",name.c_str(),msg.c_str());
	else
		serr->printf("ControlBase(\"%s\")::takeInput(\"%s\") was not a valid index or option name.\n",name.c_str(),msg.c_str());
	refresh();
	return this;
}

float ControlBase::hilightsAvg() const {
	if(hilights.size()==0)
		return -1;
	unsigned int total=0;
	for(unsigned int i=0; i<hilights.size(); i++)
		total+=hilights[i];
	return (float)total/(float)hilights.size();
}


/*! @file
 * @brief Implements ControlBase from which all items in the control system should inherit
 * @author ejt (Creator)
 */

