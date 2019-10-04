#import "OgreController.h"

#include "../UIController.h"
#include "../EnvConfig.h"
#include <CoreFoundation/CFArray.h>
#include <Foundation/NSPathUtilities.h>
#include <cstdlib>
#include <sys/stat.h>
#include <cerrno>

using namespace Ogre;

void UIController::exitWithMessage(const std::string& msg) {
	if(msg.size()!=0) {
		NSString * str = [NSString stringWithUTF8String:msg.c_str()];
		[[NSAlert alertWithMessageText:nil defaultButton:nil alternateButton:nil otherButton:nil informativeTextWithFormat:str] runModal];
	}
	[NSApp terminate:nil];
}

struct FrameRateSync : public plist::PrimitiveListener {
	FrameRateSync(id renderTgt) : plist::PrimitiveListener(), controller(renderTgt), renderTimer(NULL) {
		EnvConfig::singleton().fps.addPrimitiveListener(this);
		plistValueChanged(EnvConfig::singleton().fps);
	}
	
	~FrameRateSync() {
		EnvConfig::singleton().fps.removePrimitiveListener(this);
		if(renderTimer!=nil)
			[renderTimer invalidate];
	}
	
	virtual void plistValueChanged(const plist::PrimitiveBase& pb) {
		if(renderTimer!=nil)
			[renderTimer invalidate];
		float period = 1.f/EnvConfig::singleton().fps;
		renderTimer = [NSTimer scheduledTimerWithTimeInterval:period target:controller selector:@selector(renderFrame) userInfo:NULL repeats:YES];
		[[NSRunLoop currentRunLoop] addTimer:renderTimer forMode:NSEventTrackingRunLoopMode];
	}
	id controller;
	NSTimer *renderTimer;
};
static FrameRateSync * fpsSync=NULL;


@implementation OgreController


- (void)applicationWillFinishLaunching:(NSNotification *)notification
{
	ui = new UIController;
}


- (BOOL)application:(NSApplication*)theApplication openFile:(NSString*)filename
{
	return ui->loadFile([filename UTF8String]) ? YES : NO;
}


- (void)applicationDidFinishLaunching:(NSNotification *)notification
{
	NSArray* arr = NSSearchPathForDirectoriesInDomains(NSLibraryDirectory,NSLibraryDirectory,true);
	std::string configPath,logPath;
	if([arr count]>0) {
		logPath = configPath = [[arr objectAtIndex:0] UTF8String];
		configPath.append("/Preferences");
		logPath.append("/Logs");
	} else {
		const char* home=std::getenv("HOME");
		if(home==NULL) {
			std::cerr << "Could not determine Library directory!  Using $PWD/.mirage" << std::endl;
			configPath=".";
		} else {
			std::cerr << "Could not determine Library directory!  Using $HOME/.mirage" << std::endl;
			configPath=home;
		}
		configPath.append("/.mirage");
		logPath=configPath;
		struct stat configDirInfo;
		if(stat(configPath.c_str(),&configDirInfo)!=0) {
			if(errno!=ENOENT) {
				perror(home==NULL?"stat '.mirage' for config":"stat '$HOME/.mirage' for config");
			} else if(mkdir(configPath.c_str(),0755)!=0) {
				perror(home==NULL?"mkdir '.mirage' for config":"mkdir '$HOME/.mirage' for config");
			}
		}
	}
	std::string resourcePath = [[[NSBundle mainBundle] resourcePath] UTF8String];
	ui->startup(resourcePath,configPath,logPath);
	fpsSync = new FrameRateSync(self);
}


- (void)renderFrame
{
	ui->renderCallback();
}


- (void)applicationWillTerminate:(NSNotification *)aNotification
{
	delete fpsSync;
	fpsSync=NULL;
	ui->shutdown();
}

@end
