#if defined(__APPLE__) && !defined(__x86_64__)

#include <AvailabilityMacros.h>
#ifndef MAC_OS_X_VERSION_10_6

#include "CameraDriverQTSG.h"
#include "Shared/get_time.h"
#include "Shared/debuget.h"

#include <sstream>

using namespace std; 

const std::string CameraDriverQTSG::autoRegisterCameraDriver = DeviceDriver::getRegistry().registerType<CameraDriverQTSG>("Camera");

void CameraDriverQTSG::dumpLiteral(OSType t) {
	union {
		OSType v;
		char s[4];
	} x;
	x.v=t;
	cout << x.s[3] << x.s[2] << x.s[1] << x.s[0];
}

void CameraDriverQTSG::updateCameraList() {
	if(!checkQTThreadInit()) {
		cerr << "CameraDriver: Couldn't initialize QuickTime" << endl;
		return;
	}
	
	// open the sequence grabber, assuming there's only ever one component of this type listed
	SeqGrabComponent sg = OpenDefaultComponent(SeqGrabComponentType, 0);
	if(sg==NULL) {
		cerr << "CameraDriver: Couldn't open sequence grabber" << endl;
		return;
	}
	
	OSErr err=noErr;
	SGChannel sgChan=NULL; // temporary channel just to get device list
	SGDeviceList devList = NULL; // list of capture devices
	try {
		// initialize the default sequence grabber component
		err = SGInitialize(sg);
		if(err!=noErr) throw "SGInitialize";
		
		err = SGNewChannel(sg, VideoMediaType, &sgChan);
		if(err!=noErr) throw "SGNewChannel";
		
		//err = SGSetChannelBounds(sgChan, &bounds);
		//if(err!=noErr) throw "SGSetChannelBounds";
		
		err = SGSetChannelUsage(sgChan, seqGrabRecord | seqGrabLowLatencyCapture /* | seqGrabPreview | seqGrabAlwaysUseTimeBase */);
		if(err!=noErr) throw "SGSetChannelUsage";
		
		// just for debugging info
		/*CodecNameSpecListPtr codecs;
		err = GetCodecNameList(&codecs,0);
		if(err!=noErr) cerr << "Could not get codec list" << endl;
		else {
			cout << "Codec names: " << endl;
			for(int i=0; i<codecs->count; ++i) {
				cout << '\t' << codecs->list[i].codec << ' ';
				dumpLiteral(codecs->list[i].cType);
				cout << ' ' << p2c(codecs->list[i].typeName) << endl;
			}
			DisposeCodecNameList(codecs);
		}*/
		
		
		// thanks Harald ( hxr AT users sourceforge net ) for 'wacaw' source to demonstrate
		// how to get the device list via sequence grabber...
		err = SGGetChannelDeviceList(sgChan, sgDeviceListDontCheckAvailability | sgDeviceListIncludeInputs, &devList);
		if(err!=noErr) throw "SGGetChannelDeviceList";
		
		// we got our list, close the channel so whatever default device it grabbed is available for the CameraSourceQTSG
		// (we can open SequenceGrabber component multiple times, but each channel/device only open from one place...)
		if(sgChan!=NULL)
			SGDisposeChannel(sg,sgChan);
		sgChan=NULL;
		
		map<string,unsigned int> nameCnt;
		SGDeviceListRecord& list = **devList;
		
		storage_t olddict; // need to backup old dictionary so we can delete unused entries
		dict.swap(olddict);
		storage_t::const_iterator it = olddict.find(".type");
		if(it!=olddict.end())
			dict.insert(*it);
		myRef.clear(); // we'll rebuild these in the following loop
		comments.clear();
		
		for(int i = 0; i<list.count; i++) {
			string devName = p2c(list.entry[i].name);
			//cout << "Device: " << devName << ' ' << (list.entry[i].flags & sgDeviceNameFlagDeviceUnavailable) << ' ' << (list.entry[i].flags & sgDeviceNameFlagShowInputsAsDevices) << endl;
			
			SGDeviceInputList inputList = list.entry[i].inputs;
			if(inputList != NULL) {
				SGDeviceInputListRecord& inputs = **inputList;
				//if(inputs.count==0)
					//cout << "    (no inputs)" << endl;
				//cout << "    There are " << inputs.count << " inputs for this device." << endl;
				//cout << "    The current selection is " << inputs.selectedIndex << endl;
				for (int j = 0; j < inputs.count; j++) {
					string inputName = p2c(inputs.entry[j].name);
					//cout << "    Input: " << inputName << ' ' << (inputs.entry[j].flags & sgDeviceInputNameFlagInputUnavailable) << endl;
					// heuristic hack -- take the last word which starts with an alphabet character as the "short" input name
					unsigned int x;
					for(x=inputName.size(); x>0; --x)
						if(isspace(inputName[x-1]) && x<inputName.size() && isalpha(inputName[x]))
							break;
					string name;
					while(x<inputName.size() && !isspace(inputName[x]))
						name+=inputName[x++];
					if(nameCnt[name]++ > 0) {
						stringstream uniqname;
						uniqname << name << "-" << nameCnt[name];
						name = uniqname.str();
					}
					//cout << "    I shall call you '" << name << "'" << endl;
					
					bool found; // go through all of the current sources looking for this input
					for(it=olddict.begin(); it!=olddict.end(); ++it) {
						if(it->first==".type")
							continue;
						CameraSourceQTSG& ds = dynamic_cast<CameraSourceQTSG&>(*it->second);
						// have to match the full device and input names, not the "short" name!
						if(ds.getDeviceName()==devName && ds.getInputName()==inputName) {
							found=true; // we already have an entry for this input -- reuse that instance instead of making a new one
							myRef.insert(dict[name] = &ds);
							break;
						}
					}
					if(!found) { // didn't find a pre-existing CameraSourceQTSG for this input
						// make a new one, store it in the dictionary, and add it to myRef:
						try {
							CameraSourceQTSG * cam = new CameraSourceQTSG(sg, name, devName, inputName, j);
							myRef.insert(dict[name] = cam);
						} catch(const pair<OSErr,const char *>& msg) {
							cerr << "CameraDriver registering: " << inputName << endl;
							cerr << "   on device: " << devName << endl;
							cerr << "     call to: " << msg.second << " returned error " << msg.first << " (attempting to continue...)" << endl;
							continue;
						}
					}
					comments[name]="Device Name: "+devName+"\nInput Name: "+inputName;
				}
			}
			
			// now go through entries of olddict
			// anything that isn't in dict nor myRef should be deleted
			// (entries may be "renamed" if bus topology changes, so might have
			// old entries not deleted because it's in use under a new name)
			for(it=olddict.begin(); it!=olddict.end(); ++it) {
				if(it->first==".type")
					continue;
				if(dict.find(it->first)==dict.end()) {
					if(myRef.find(it->second)==myRef.end())
						delete it->second;
				} else {
					// we shouldn't have any entries which are found in dict (i.e. in use) but not myRef
					ASSERT(myRef.find(it->second)!=myRef.end(), "Entry in use with unreferenced value...???");
				}
			}
		}
		
	} catch(const char* call) {
		cerr << "CameraDriver: " << call << " returned error " << err << endl;
	} catch(const pair<OSErr,const char *>& msg) {
		cerr << "CameraDriver: " << msg.second << " returned error " << msg.first << endl;
	}
	if(devList!=NULL)
		SGDisposeDeviceList(sg, devList);
	devList=NULL;
	if(sgChan!=NULL)
		SGDisposeChannel(sg,sgChan);
	sgChan=NULL;
	if(sg!=NULL)
		CloseComponent(sg);
	sg=NULL;
}

#endif // pre-10.6
#endif // Apple 32 bit

/*! @file
 * @brief Implements CameraDriverQTSG, which provides camera capture through QuickTime and the Sequence Grabber, now deprecated.  See the alternative CameraDriverQTKit implementation.
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
