#import <Cocoa/Cocoa.h>
#include <string>
#include <vector>

// most of the real work is done in OgreController.mm, called by NSApplicationMain

int main(int argc, const char *argv[])
{
	// any user specified command-line arguments will be processed by NSApp and sent to OgreController's application:openFile:
	int status = NSApplicationMain(argc, argv);
	// nothing past here ever actually gets executed... see OgreController::applicationWillTerminate
	return status;
}

// this is used for some Mac-specific setup in UIController
void get_mac_dirs(std::vector<std::string>& paths) {
	NSAutoreleasePool *pool =  [[NSAutoreleasePool alloc] init];
	
	paths.push_back(std::string([NSHomeDirectory() UTF8String]) + "/.mirage"); // Linux-style directory too for consistency
	
	// also lookup the "Application Support" directory locations.
	NSArray * arr = NSSearchPathForDirectoriesInDomains(NSApplicationSupportDirectory,NSAllDomainsMask,true);
	std::string appName = [[[NSBundle mainBundle] objectForInfoDictionaryKey:@"CFBundleName"] UTF8String];
	for (size_t i=0; i<[arr count]; ++i) {
		paths.push_back(std::string([[arr objectAtIndex:i] UTF8String]) + "/" + appName);
	}
	
	[pool release];
}
