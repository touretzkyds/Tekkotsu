//-*-c++-*-
#ifndef INCLUDED_minisim_h_
#define INCLUDED_minisim_h_

#ifdef TK_ENABLE_CONFIG
#  include "Shared/Config.h"
#endif

#ifdef TK_ENABLE_EROUTER
#  include "Events/EventRouter.h"
#endif

#ifdef TK_ENABLE_WIRELESS
#  include "Wireless/Wireless.h"
#  define TK_ENABLE_THREADING
#endif
#ifdef TK_ENABLE_SOUTSERR
#  include "Wireless/Socket.h"
#endif

#ifdef TK_ENABLE_KINEMATICS
#  include <iostream>
#  include "Shared/Config.h"
#  include "Motion/Kinematics.h"
#endif

#ifdef TK_ENABLE_THREADING
#  include "IPC/Thread.h"
#endif

#include <signal.h>

#define TK_cstr(s) TK_str(s)
#define TK_str(s) #s

//! provides initialization and teardown of Tekkotsu globals, provide easy access to framework from command line tools or other executables
/*! See the minisim.h file documentation for a list of macros
 *  to control which framework features will be initialized for use. */
namespace minisim {
	//! call this one time before using framework features
	void initialize();
	//! call this when done using framework features
	void destruct();
	
	//! a class which relies on its constructor/destructor pair to ensure destruct() is called
	/*! This is convenient if your main function has many error returns, or throws exceptions. */
	struct AutoScopeInit {
		AutoScopeInit() { minisim::initialize(); } //!< constructor calls minisim::initialize()
		~AutoScopeInit() { minisim::destruct(); } //!< destructor calls minisim::destructor();
	};
	
#ifdef TK_ENABLE_WIRELESS
	class WirelessThread : public Thread {
	public:
		//! constructor
		WirelessThread() : Thread() {}
		//! destructor -- stop thread
		virtual ~WirelessThread() {
			if(isStarted()) {
				stop();
				join();
			}
		}
		virtual unsigned int runloop() {
			// initial setup done by wireless's own constructor
			wireless->pollTest(NULL); // by passing NULL, we wait indefinitely, so no need to usleep in the Thread code
			wireless->pollProcess();
			wireless->pollSetup(); // reinitialize for next test
			return 0; //no sleep time because pollTest blocks
		}
		virtual Thread& stop() {
			Thread::stop();
			wireless->wakeup();
			return *this;
		}
	} * wireless_thread;
#endif
	
	void initialize() {
		signal(SIGPIPE, SIG_IGN);
		
#ifdef TK_ENABLE_CONFIG
		if(config==NULL) {
			::config = new Config();
			::config->setFileSystemRoot("ms");
			std::string configfile=TK_cstr(TK_ENABLE_CONFIG);
			if(configfile.size()!=0) {
				if(::config->loadFile(configfile.c_str())==0)
					std::cerr << std::endl << " *** ERROR: Could not load configuration file " << configfile << " *** " << std::endl << std::endl;
			} else {
				if(::config->loadFile("config/tekkotsu.xml")==0) {
					if(::config->loadFile("config/tekkotsu.cfg")==0)
						std::cerr << std::endl << " *** ERROR: Could not load configuration file config/tekkotsu.xml *** " << std::endl << std::endl;
					else
						std::cerr << "Successfully imported settings from old-format tekkotsu.cfg" << std::endl;
				}
				if(::config->loadFile("config/sim_ovrd.xml")==0)
					if(::config->loadFile("config/sim_ovrd.cfg")!=0)
						std::cerr << "Successfully imported settings from old-format simulator.cfg" << std::endl;
			}
		}
#endif
		
#ifdef TK_ENABLE_EROUTER
		erouter=new EventRouter;
#endif
		
#ifdef TK_ENABLE_THREADING
		Thread::initMainThread();
#endif

#ifdef TK_ENABLE_WIRELESS
		wireless_thread = new WirelessThread();
		wireless = new Wireless();
#endif
#ifdef TK_ENABLE_SOUTSERR
#  ifdef TK_ENABLE_WIRELESS
		sout=wireless->socket(Socket::SOCK_STREAM,Wireless::WIRELESS_DEF_RECV_SIZE,Wireless::WIRELESS_DEF_SEND_SIZE*12);
		serr=wireless->socket(Socket::SOCK_STREAM,Wireless::WIRELESS_DEF_RECV_SIZE,Wireless::WIRELESS_DEF_SEND_SIZE*4);
		wireless->setDaemon(sout);
		wireless->setDaemon(serr);
#  else
		sout=new Socket(0);
		serr=new Socket(1);
#  endif
		serr->setFlushType(Socket::FLUSH_BLOCKING);
		sout->setTextForward();
		serr->setForward(sout);
#  ifdef TK_ENABLE_WIRELESS
#    ifdef TK_ENABLE_CONFIG
		wireless->listen(sout, if(config!=NULL) ? 10000 : config->main.console_port);
		wireless->listen(serr, if(config!=NULL) ? 10001 : config->main.stderr_port);
#    else
		wireless->listen(sout, 10000 );
		wireless->listen(serr, 10001 );
#    endif
#  endif
#else //no sout/serr
#  ifdef TK_ENABLE_WIRELESS //but if wireless is enabled, might as well define them
		sout=new Socket(-1);
		serr=new Socket(-1);
		serr->setFlushType(Socket::FLUSH_BLOCKING);
		sout->setTextForward();
		serr->setForward(sout);
#  endif
#endif //sout and serr
		
#ifdef TK_ENABLE_KINEMATICS
		if(config==NULL) {
			std::cerr << "ERROR: Kinematics depends on Config -- either define TK_ENABLE_CONFIG,\n"
			            "  or manually create a Config, either way providing values for:\n"
			            "    motion_config::kinematics - location of the kinematics spec file\n"
			            "    motion_config::kinematic_chains - name(s) of chain(s) to load" << std::endl;
			exit(1);
		}
		if(config->motion.kinematics.size()==0 || config->motion.kinematic_chains.size()==0) {
			std::cerr << "ERROR: The specified configuration does not provide values for:\n"
			            "    motion_config::kinematics - location of the kinematics spec file\n"
			            "    motion_config::kinematic_chains - name(s) of chain(s) to load\n"
			            "  These values are required for Kinematics initialization." << std::endl;
			exit(1);
		}
#ifndef TK_ENABLE_SOUTSERR
#  warning If using Kinematics, it is wise to enable sout & serr in case errors are enountered.
#endif
		kine = new Kinematics();
#endif
		
#ifdef TK_ENABLE_WIRELESS
		wireless_thread->start();
#endif
	}

	
	void destruct() {
#ifdef TK_ENABLE_WIRELESS
		wireless_thread->stop().join();
		delete wireless_thread;
		wireless_thread=NULL;
#endif
		
#ifdef TK_ENABLE_KINEMATICS
		delete kine;
		kine=NULL;
#endif
		

#if defined(TK_ENABLE_WIRELESS) || defined(TK_ENABLE_SOUTSERR)
		if(serr!=NULL) {
#  if defined(TK_ENABLE_WIRELESS) && defined(TK_ENABLE_SOUTSERR)
			wireless->setDaemon(serr,false);
			wireless->close(serr);
#  else
			delete serr;
#  endif
			serr=NULL;
		}
		
		if(sout!=NULL) {
#  if defined(TK_ENABLE_WIRELESS) && defined(TK_ENABLE_SOUTSERR)
			wireless->setDaemon(sout,false);
			wireless->close(sout);
#  else
			delete sout;
#  endif
			sout=NULL;
		}

#  ifdef TK_ENABLE_WIRELESS
		delete wireless;
		wireless=NULL;
#  endif
#endif
		
#ifdef TK_ENABLE_EROUTER
		delete erouter;
		erouter=NULL;
#endif
		
#ifdef TK_ENABLE_CONFIG
		delete config;
		config=NULL;
#endif
	}
	
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
