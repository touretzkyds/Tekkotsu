#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_CAMERA) || defined(TGT_HAS_WEBCAM)

#include "SegCam.h"
#include "Wireless/Wireless.h"
#include "Events/EventRouter.h"
#include "Events/FilterBankEvent.h"
#include "Behaviors/Controller.h"
#include "Shared/ProjectInterface.h"
#include "Vision/SegmentedColorGenerator.h"
#include "Vision/RLEGenerator.h"
#include "Shared/debuget.h"

REGISTER_BEHAVIOR_MENU_OPT(SegCam,"TekkotsuMon",BEH_NONEXCLUSIVE);

SegCam* SegCam::theOne=NULL;

SegCam::SegCam()
	: CameraStreamBehavior("SegCam",visRLE), visRLE(NULL), packet(NULL), cur(NULL), avail(0), max_buf(0), lastProcessedTime(0)
{
	ASSERT(theOne==NULL,"there was already a SegCam running!");
	theOne=this;
}

void
SegCam::doStart() {
	BehaviorBase::doStart();
	setupServer();
	erouter->addListener(this,EventBase::visSegmentEGID,ProjectInterface::visSegmentSID,EventBase::deactivateETID);
	erouter->addListener(this,EventBase::visRLEEGID,ProjectInterface::visRLESID,EventBase::deactivateETID);
}

void
SegCam::doStop() {
	erouter->removeListener(this);
	closeServer();
	BehaviorBase::doStop();
}

void
SegCam::doEvent() {
	if(!wireless->isConnected(visRLE->sock))
		return;
	if( (config->vision.segcam.transport==0 && visRLE->getTransport()==Socket::SOCK_STREAM)
		 || (config->vision.segcam.transport==1 && visRLE->getTransport()==Socket::SOCK_DGRAM) ) {
		closeServer();
		setupServer();
		return;
	}
	try {
		const FilterBankEvent* fbke=dynamic_cast<const FilterBankEvent*>(event);
		if(fbke==NULL) {
			CameraStreamBehavior::doEvent();
			return;
		}
		if ((get_time() - lastProcessedTime) < config->vision.segcam.interval) // not enough time has gone by
			return;
		if(config->vision.segcam.compression==Config::vision_config::SegCamConfig::COMPRESS_NONE && event->getGeneratorID()==EventBase::visSegmentEGID) {
			if(!writeSeg(*fbke)) {
				if(packet!=NULL) {
					cur=packet;
					closePacket();
				}
				//error message should already have been reported
				//ASSERTRET(succ,"serialization failed");
			}
		}
		if(config->vision.segcam.compression==Config::vision_config::SegCamConfig::COMPRESS_RLE && event->getGeneratorID()==EventBase::visRLEEGID) {
			if(!writeRLE(*fbke)) {
				if(packet!=NULL) {
					cur=packet;
					closePacket();
				}
				//error message should already have been reported
				//ASSERTRET(succ,"serialization failed");
			}
		}
	} catch(...) {
		if(packet!=NULL) {
			cur=packet;
			closePacket();
		}
		// typically this is a per-frame recurring error, so let's just stop now
		serr->printf("%s: exception generated during image serialization, stopping stream.\n",getName().c_str());
		stop();
		throw;
	}
}

void
SegCam::closeServer() {
	if(wireless->isConnected(visRLE->sock))
		sendCloseConnectionPacket();
	Controller::closeGUI("SegVisionGUI");
	
	// this could be considered a bug in our wireless - if we don't setDaemon(...,false)
	// it will try to listen again even though we explicitly closed the server socket...
	wireless->setDaemon(visRLE,false);
	wireless->close(visRLE->sock);
}

void
SegCam::setupServer() {
	std::vector<std::string> args;
	args.push_back("rle");
	char port[50];
	snprintf(port,50,"%d",*config->vision.segcam.port);
	args.push_back(port);
	if(config->vision.segcam.transport==0) {
		max_buf=UDP_WIRELESS_BUFFER_SIZE;
		visRLE=wireless->socket(Socket::SOCK_DGRAM, 1024, max_buf);
		args.push_back("udp");
	} else if(config->vision.segcam.transport==1) {
		max_buf=TCP_WIRELESS_BUFFER_SIZE;
		visRLE=wireless->socket(Socket::SOCK_STREAM, 1024, max_buf);
		args.push_back("tcp");
	} else {
		serr->printf("ERROR: Invalid Config::vision.segcam.transport: %d\n",*config->vision.segcam.transport);
		return;
	}
	wireless->setDaemon(visRLE,true);
	wireless->setReceiver(visRLE,networkCallback);
	wireless->listen(visRLE,config->vision.segcam.port);
	
	Controller::loadGUI("org.tekkotsu.mon.VisionGUI","SegVisionGUI",config->vision.segcam.port,args);
}

bool
SegCam::openPacket(FilterBankGenerator& fbkgen, unsigned int time, unsigned int layer) {
	if(packet!=NULL)
		return false;

	avail=max_buf-1; //not sure why -1, but Alok had it, so i will too
	ASSERT(cur==NULL,"cur non-NULL");
	cur=NULL;
	char * buf=packet=(char*)visRLE->getWriteBuffer(avail);
	ASSERT(packet!=NULL,"dropped frame, network bandwidth is saturated (reduce frame rate or size)");
	if(packet==NULL)
		return false;
	
	if(!LoadSave::encodeInc("TekkotsuImage",buf,avail,"ran out of space %s:%u\n",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(Config::vision_config::ENCODE_SINGLE_CHANNEL,buf,avail,"ran out of space %s:%u\n",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(Config::vision_config::SegCamConfig::COMPRESS_RLE,buf,avail,"ran out of space %s:%u\n",__FILE__,__LINE__)) return false;;

	if(!LoadSave::encodeInc(fbkgen.getWidth(layer),buf,avail,"ran out of space %s:%u\n",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(fbkgen.getHeight(layer),buf,avail,"ran out of space %s:%u\n",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(time,buf,avail,"ran out of space %s:%u\n",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(fbkgen.getFrameNumber(),buf,avail,"ran out of space %s:%u\n",__FILE__,__LINE__)) return false;;

	cur=buf;
	return true;
}

bool
SegCam::writeRLE(const FilterBankEvent& e) {
	FilterBankGenerator& fbkgen=*e.getSource();

	unsigned int layer=fbkgen.getNumLayers()-1-config->vision.segcam.skip;
	openPacket(fbkgen,e.getTimeStamp(),layer);
	if(cur==NULL) //error should have been displayed by openPacket
		return false;
	
	RLEGenerator * rle = dynamic_cast<RLEGenerator*>(&fbkgen);
	ASSERTRETVAL(rle!=NULL,"fbkgen isn't an RLEGenerator",false);

	rle->selectSaveImage(layer,config->vision.segcam.channel);
	if(!LoadSave::checkInc(rle->saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.segcam.transport to TCP and reopen seg cam")) return false;
	
	// send out the color map ourselves (since RLE compression doesn't have a concept of color)
	const SegmentedColorGenerator * seg = dynamic_cast<const SegmentedColorGenerator*>(rle->getSourceGenerator());
	ASSERTRETVAL(seg!=NULL,"RLE's source is not a SegmentedColorGenerator - how do i know what the colors are?",false);
	if(!seg->encodeColorsInc(cur,avail)) return false;

	closePacket();

	return true;
}

bool
SegCam::writeSeg(const FilterBankEvent& e) {
	FilterBankGenerator& fbkgen=*e.getSource();

	unsigned int layer=fbkgen.getNumLayers()-1-config->vision.segcam.skip;
	openPacket(fbkgen,e.getTimeStamp(),layer);
	if(cur==NULL) //error should have been displayed by openPacket
		return false;
	
	fbkgen.selectSaveImage(layer,config->vision.segcam.channel);
	if(!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.segcam.transport to TCP and reopen seg cam")) return false;
	
	closePacket();

	return true;
}

void
SegCam::closePacket() {
	if(packet==NULL)
		return;
	visRLE->write(cur-packet);
	packet=cur=NULL;
	avail=0;
	lastProcessedTime = get_time();
}

bool
SegCam::sendCloseConnectionPacket() {
	char msg[]="CloseConnection";
	unsigned int len=strlen(msg)+LoadSave::stringpad;
	char * buf = (char*)visRLE->getWriteBuffer(len);
	if(buf==NULL) {
		std::cerr << "Could not get buffer for closing packet" << std::endl;
		return false;
	}
	unsigned int used=LoadSave::encode(msg,buf,len);
	if(used==0)
		std::cerr << "Could not write close packet" << std::endl;
	visRLE->write(used);
	return true;
}

#endif

/*! @file
 * @brief Implements SegCam, which forwards segmented images from camera over wireless
 * @author ejt (Creator)
 */

