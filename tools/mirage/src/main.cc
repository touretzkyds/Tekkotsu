// the apple-specific implementations of these is in mac/main.mm (objective-c++)
#ifndef __APPLE__

#include "UIController.h"
#include "EnvConfig.h"
#include <cstdlib>
#include <iostream>
#include <cstdlib>
#include <sys/stat.h>
#include <cerrno>

int main(int argc, char** argv) {
	const char* home=std::getenv("HOME");
	std::string configDir = (home==NULL) ? "." : home;
	configDir.append("/.mirage");
	struct stat configDirInfo;
	if(stat(configDir.c_str(),&configDirInfo)!=0) {
		if(errno!=ENOENT) {
			perror(home==NULL?"stat '.mirage' for config":"stat '$HOME/.mirage' for config");
		} else if(mkdir(configDir.c_str(),0755)!=0) {
			perror(home==NULL?"mkdir '.mirage' for config":"mkdir '$HOME/.mirage' for config");
		}
	}

	std::string resourcesDir="Resources";
#ifdef __FreeBSD__
	char proc[]="/proc/curproc/file";
#elif defined(__linux__)
	char proc[]="/proc/self/exe";
#endif
	char selfPath[PATH_MAX];
	int pathLen = readlink(proc, selfPath, sizeof(selfPath)-1);
	if (pathLen < 0) {
		perror("readlink() error looking up executable path");
	} else {
		selfPath[pathLen] = '\0';
		char * pch = strrchr(selfPath,'/');
		if(pch!=NULL) {
			*pch='\0';
			resourcesDir=std::string(selfPath)+"/"+resourcesDir;
		}
	}
	
	ui = new UIController;
	for(int i=1; i<argc; ++i)
		ui->loadFile(argv[i]);
	ui->startup(resourcesDir,configDir,configDir);
	try {
		//Ogre::Root::getSingleton().startRendering();
		while(ui->renderCallback()) {
			usleep(static_cast<useconds_t>(1000000.f/EnvConfig::singleton().fps+.5f));
		}
	} 
	catch(...) {
		ui->shutdown();
		delete ui;
		ui=NULL;
		return EXIT_FAILURE;
	}
	ui->shutdown();
	delete ui;
	ui=NULL;
	return EXIT_SUCCESS;
}

void UIController::exitWithMessage(const std::string& msg) {
	if(msg.size()!=0)
		std::cerr << msg << std::endl;
	ui->shutdown();
	delete ui;
	ui=NULL;
	exit(EXIT_FAILURE);
}

#endif

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
