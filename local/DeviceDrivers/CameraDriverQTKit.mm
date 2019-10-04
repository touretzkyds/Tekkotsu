#if defined(__APPLE__)

#include <AvailabilityMacros.h>
#ifdef MAC_OS_X_VERSION_10_6

#include "CameraDriverQTKit.h"

#import <QTKit/QTkit.h>

using namespace std; 

const std::string CameraDriverQTKit::autoRegisterCameraDriver = DeviceDriver::getRegistry().registerType<CameraDriverQTKit>("Camera");

void CameraDriverQTKit::updateCameraList() {
	NSAutoreleasePool	 *autoreleasepool = [[NSAutoreleasePool alloc] init];
	NSEnumerator *itr = [[QTCaptureDevice inputDevices] objectEnumerator];
	while (QTCaptureDevice* dev = [itr nextObject]) {
		if([dev isConnected] && ![dev isInUseByAnotherApplication] && ( [dev hasMediaType:QTMediaTypeVideo] || [dev hasMediaType:QTMediaTypeMuxed] )) {
			string name = [[dev localizedDisplayName] UTF8String];
			if(name.find("Built-in ")==0)
				name.erase(0,strlen("Built-in "));
			setEntry(name,new CameraSourceQTKit(name,dev));
		}
	}
	[autoreleasepool release];
}

#endif // 10.6 or later
#endif // Apple platform

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 *
 * $Author: ejt $
 * $Name:  $
 * $Revision: 1.1 $
 * $State: Exp $
 * $Date: 2010/05/06 20:51:47 $
 */
