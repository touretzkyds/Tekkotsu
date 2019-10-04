#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_CAMERA) || defined(TGT_HAS_WEBCAM)

#include "RegionCam.h"
#include "Wireless/Wireless.h"
#include "Events/EventRouter.h"
#include "Events/FilterBankEvent.h"
#include "Behaviors/Controller.h"
#include "Shared/ProjectInterface.h"
#include "Vision/SegmentedColorGenerator.h"
#include "Vision/RegionGenerator.h"
#include "Shared/debuget.h"

REGISTER_BEHAVIOR_MENU_OPT(RegionCam,"TekkotsuMon",BEH_NONEXCLUSIVE);

RegionCam::RegionCam()
	: BehaviorBase("RegionCam"), visRegion(NULL), packet(NULL), cur(NULL), avail(0), max_buf(0)
{
}

void
RegionCam::doStart() {
	BehaviorBase::doStart();
	setupServer();
	erouter->addListener(this,EventBase::visRegionEGID,ProjectInterface::visRegionSID,EventBase::deactivateETID);
}

void
RegionCam::doStop() {
	erouter->removeListener(this);
	closeServer();
	BehaviorBase::doStop();
}

void
RegionCam::doEvent() {
	if(!wireless->isConnected(visRegion->sock))
		return;
	if( (config->vision.regioncam.transport==0 && visRegion->getTransport()==Socket::SOCK_STREAM)
		 || (config->vision.regioncam.transport==1 && visRegion->getTransport()==Socket::SOCK_DGRAM) ) {
		closeServer();
		setupServer();
		return;
	}
	const FilterBankEvent* fbke=dynamic_cast<const FilterBankEvent*>(event);
	ASSERTRET(fbke!=NULL,"unexpected event");
#if DEBUG
	bool succ=writeRegions(*fbke);
	ASSERTRET(succ,"serialization failed");
#else
	writeRegions(*fbke);
#endif
}

void
RegionCam::closeServer() {
	Controller::closeGUI("RegionVisionGUI");
	
	// this could be considered a bug in our wireless - if we don't setDaemon(...,false)
	// it will try to listen again even though we explicitly closed the server socket...
	wireless->setDaemon(visRegion,false);
	wireless->close(visRegion->sock);
}

void
RegionCam::setupServer() {
	std::vector<std::string> args;
	args.push_back("reg"); 
	char port[50];
	snprintf(port,50,"%d",*config->vision.regioncam.port);
	args.push_back(port);
	if(config->vision.regioncam.transport==0) {
		max_buf=UDP_WIRELESS_BUFFER_SIZE;
		visRegion=wireless->socket(Socket::SOCK_DGRAM, 1024, max_buf);
		args.push_back("udp");
	} else if(config->vision.regioncam.transport==1) {
		max_buf=TCP_WIRELESS_BUFFER_SIZE;
		visRegion=wireless->socket(Socket::SOCK_STREAM, 1024, max_buf);
		wireless->setDaemon(visRegion,true);
		args.push_back("tcp");
	} else {
		serr->printf("ERROR: Invalid Config::vision.region_transport: %d\n",*config->vision.regioncam.transport);
		return;
	}
	wireless->listen(visRegion,config->vision.regioncam.port);
	Controller::loadGUI("org.tekkotsu.mon.VisionGUI","RegionVisionGUI",config->vision.regioncam.port,args);
}

bool
RegionCam::openPacket(FilterBankGenerator& fbkgen, unsigned int time, unsigned int layer) {
	if(packet!=NULL)
		return false;

	avail=max_buf-1; //not sure why -1, but Alok had it, so i will too
	ASSERT(cur==NULL,"cur non-NULL");
	cur=NULL;
	char * buf=packet=(char*)visRegion->getWriteBuffer(avail);
	ASSERT(packet!=NULL,"could not get buffer");
	if(packet==NULL)
		return false;
	
	if(!LoadSave::encodeInc("TekkotsuImage",buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(Config::vision_config::ENCODE_SINGLE_CHANNEL,buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(Config::vision_config::SegCamConfig::COMPRESS_RLE,buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;

	if(!LoadSave::encodeInc(fbkgen.getWidth(layer),buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(fbkgen.getHeight(layer),buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(time,buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(fbkgen.getFrameNumber(),buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;

	cur=buf;
	return true;
}

bool
RegionCam::writeRegions(const FilterBankEvent& e) {
	FilterBankGenerator& fbkgen=*e.getSource();

	unsigned int layer=fbkgen.getNumLayers()-1-config->vision.regioncam.skip;
	openPacket(fbkgen,e.getTimeStamp(),layer);
	ASSERTRETVAL(cur!=NULL,"header failed",false);
	
	RegionGenerator * regGen = dynamic_cast<RegionGenerator*>(&fbkgen);
	ASSERTRETVAL(regGen!=NULL,"fbkgen isn't an RegionGenerator",false);

	regGen->selectSaveImage(layer,config->vision.segcam.channel);
	if(!LoadSave::checkInc(regGen->saveBuffer(cur,avail),cur,avail,"save region image failed")) return false;
	
	const SegmentedColorGenerator * seg = dynamic_cast<const SegmentedColorGenerator*>((regGen->getSourceGenerator())->getSourceGenerator()); //Get the source of his source (the SegmentedColorGenerator)
	ASSERTRETVAL(seg!=NULL,"The source of RegionGenerator's source is not a SegmentedColorGenerator - how do i know what the colors are?",false);
	if(!seg->encodeColorsInc(cur,avail)) return false;

	closePacket();

	return true;
}


void
RegionCam::closePacket() {
	if(packet==NULL)
		return;
	visRegion->write(cur-packet);
	packet=cur=NULL;
	avail=0;
}

#endif

/*! @file
 * @brief Implements RegionCam, which forwards the regions from RegionGenerator over wireless
 * @author harm & niels (Creators)
 */

