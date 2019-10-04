#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_CAMERA) || defined(TGT_HAS_WEBCAM)

#include "DepthCam.h"
#include "Wireless/Wireless.h"
#include "Events/EventRouter.h"
#include "Vision/RawCameraGenerator.h"
#include "Vision/JPEGGenerator.h"
#include "Events/FilterBankEvent.h"
#include "Behaviors/Controller.h"
#include "Shared/ProjectInterface.h"
#include "Shared/debuget.h"

REGISTER_BEHAVIOR_MENU_OPT(DepthCam,"TekkotsuMon",BEH_NONEXCLUSIVE);

DepthCam* DepthCam::theOne=NULL;

DepthCam::DepthCam()
	: CameraStreamBehavior("DepthCam",visDepth), visDepth(NULL), packet(NULL), cur(NULL), avail(0), max_buf(0), lastProcessedTime(0)
{
	ASSERT(theOne==NULL,"there was already a DepthCam running!");
	theOne=this;
}

void
DepthCam::doStart() {
	BehaviorBase::doStart();
	setupServer();
	erouter->addListener(this,EventBase::visRawDepthEGID, ProjectInterface::visRawDepthSID, EventBase::deactivateETID);
	// *** does this stuff below work for depth?  or is it hard-coded to the rgb image?
	erouter->addListener(this,EventBase::visJPEGEGID,ProjectInterface::visColorJPEGSID,EventBase::deactivateETID);
	erouter->addListener(this,EventBase::visJPEGEGID,ProjectInterface::visGrayscaleJPEGSID,EventBase::deactivateETID);
}

void
DepthCam::doStop() {
	erouter->removeListener(this);
	closeServer();
	BehaviorBase::doStop();
}

void
DepthCam::doEvent() {
	if(!wireless->isConnected(visDepth->sock))
		return;
	if((config->vision.depthcam.transport==0 && visDepth->getTransport()==Socket::SOCK_STREAM)
	   || (config->vision.depthcam.transport==1 && visDepth->getTransport()==Socket::SOCK_DGRAM)) {
		//reset the socket
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
		if((get_time() - lastProcessedTime) < config->vision.depthcam.interval) {// not enough time has gone by
			return;
		}
		/* // turning these off enables individual channel compression
			if(config->vision.depthcam.compression==Config::vision_config::COMPRESS_NONE && event->getGeneratorID()!=EventBase::visRawCameraEGID)
			return;
			if(config->vision.depthcam.compression==Config::vision_config::COMPRESS_JPEG && event->getGeneratorID()!=EventBase::visJPEGEGID)
			return; */
	
		if(!writeDepth(*fbke)) {
		        if(packet) {
		                cur=packet;
		                closePacket();
			}
		}
	} catch(...) {
		if(packet) { //packet was opened, need to close it
			cur=packet; // don't send anything
			closePacket();
		}
		// typically this is a per-frame recurring error, so let's just stop now
		serr->printf("%s: exception generated during image serialization, stopping stream.\n",getName().c_str());
		stop();
		throw;
	}
}

void
DepthCam::closeServer() {
	if(wireless->isConnected(visDepth->sock))
		sendCloseConnectionPacket();
	Controller::closeGUI("DepthVisionGUI");
	
	// this could be considered a bug in our wireless - if we don't setDaemon(...,false)
	// it will try to listen again even though we explicitly closed the server socket...
	wireless->setDaemon(visDepth,false);
	wireless->close(visDepth->sock);
}

void
DepthCam::setupServer() {
	std::vector<std::string> args;
	args.push_back("depth");
	char port[50];
	snprintf(port,50,"%d",*config->vision.depthcam.port);
	args.push_back(port);
	if(config->vision.depthcam.transport==0) {
		max_buf=UDP_WIRELESS_BUFFER_SIZE;
		visDepth=wireless->socket(Socket::SOCK_DGRAM, 1024, max_buf);
		args.push_back("udp");
	} else if(config->vision.depthcam.transport==1) {
		max_buf=TCP_WIRELESS_BUFFER_SIZE;
		visDepth=wireless->socket(Socket::SOCK_STREAM, 1024, max_buf);
		args.push_back("tcp");
	} else {
		serr->printf("ERROR: Invalid Config::vision.depthcam.transport: %d\n",*config->vision.depthcam.transport);
		return;
	}
	wireless->setDaemon(visDepth,true);
	wireless->setReceiver(visDepth,networkCallback);
	wireless->listen(visDepth,config->vision.depthcam.port);
	Controller::loadGUI("org.tekkotsu.mon.VisionGUI","DepthVisionGUI",*config->vision.depthcam.port,args);
}
bool
DepthCam::openPacket(FilterBankGenerator& fbkgen, unsigned int time, unsigned int layer) {
	if(packet!=NULL)
		return false;

	avail=max_buf-1; //not sure why -1, but Alok had it, so i will too
	ASSERT(cur==NULL,"cur non-NULL");
	cur=NULL;
	char * buf=packet=(char*)visDepth->getWriteBuffer(avail);
	ASSERT(packet!=NULL,"dropped frame, network bandwidth is saturated (reduce frame rate or size)");
	if(packet==NULL)
		return false;
	
	if(!LoadSave::encodeInc("TekkotsuImage",buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(Config::vision_config::ENCODE_DEPTH,buf,avail,"ran out of space %s:%u\n",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(*config->vision.depthcam.compression,buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(fbkgen.getWidth(layer),buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(fbkgen.getHeight(layer),buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(time,buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
	if(!LoadSave::encodeInc(fbkgen.getFrameNumber(),buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;

	cur=buf;
	return true;
}
bool
DepthCam::writeDepth(const FilterBankEvent& e) {
	FilterBankGenerator& fbkgen=*e.getSource();

	unsigned int y0_layer=fbkgen.getNumLayers()-1-config->vision.depthcam.y0_skip;
	unsigned int y1_layer=fbkgen.getNumLayers()-1-config->vision.depthcam.y1_skip;
	unsigned int big_layer=y0_layer;
	unsigned int small_layer=y1_layer;
	if(y0_layer<y1_layer) { 
		big_layer=y1_layer;
		small_layer=y0_layer;
	}
	if(const JPEGGenerator* jgen=dynamic_cast<const JPEGGenerator*>(&fbkgen)) {
		if(config->vision.depthcam.compression!=Config::vision_config::DepthCamConfig::COMPRESS_JPEG)
			return true;
		if(jgen->getCurrentSourceFormat()==JPEGGenerator::SRC_COLOR && big_layer-small_layer<2) {
			openPacket(fbkgen,e.getTimeStamp(),big_layer);
			if(cur==NULL) //error should have been displayed by openPacket
				return false;
			
			fbkgen.selectSaveImage(big_layer,0);
			if(!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.depthcam.transport to TCP and reopen depth cam")) return false;
			
			closePacket();
		} else if(jgen->getCurrentSourceFormat()==JPEGGenerator::SRC_GRAYSCALE && big_layer-small_layer>=2) {
			bool opened=openPacket(fbkgen,e.getTimeStamp(),big_layer);
			if(cur==NULL) //error should have been displayed by openPacket
				return false;
			
			if(big_layer==y0_layer) {
				fbkgen.selectSaveImage(y0_layer,RawCameraGenerator::CHAN_Y);
				if(!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.depthcam.transport to TCP and reopen depth cam")) return false;
			} else {
				fbkgen.selectSaveImage(y1_layer,RawCameraGenerator::CHAN_U);
				if(!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.depthcam.transport to TCP and reopen depth cam")) return false;
			}
			
			if(!opened)
				closePacket();
		}
	} else { // not a JPEG generator
		bool opened=false;
		
		if(config->vision.depthcam.compression==Config::vision_config::DepthCamConfig::COMPRESS_NONE || (big_layer-small_layer>=2 && big_layer==y1_layer)) {
			opened=openPacket(fbkgen,e.getTimeStamp(),big_layer);
			if(cur==NULL) //error should have been displayed by openPacket
				return false;
			fbkgen.selectSaveImage(y0_layer,RawCameraGenerator::CHAN_Y);
			if(!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.depthcam.transport to TCP and reopen depth cam")) return false;
		}
		
		if(config->vision.depthcam.compression==Config::vision_config::DepthCamConfig::COMPRESS_NONE || (big_layer-small_layer>=2 && big_layer==y0_layer)) {
			opened=openPacket(fbkgen,e.getTimeStamp(),big_layer);
			if(cur==NULL) //error should have been displayed by openPacket
				return false;
			fbkgen.selectSaveImage(y1_layer,RawCameraGenerator::CHAN_U);
			if(!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.depthcam.transport to TCP and reopen depth cam")) return false;
		}
		
		if(config->vision.depthcam.compression==Config::vision_config::DepthCamConfig::COMPRESS_NONE || !opened)
			closePacket();
	}
	
	return true;
}

void DepthCam::closePacket() {
	if(packet==NULL)
		return;
	visDepth->write(cur-packet);
	packet=cur=NULL;
	avail=0;
	lastProcessedTime = get_time();
}

bool DepthCam::sendCloseConnectionPacket() {
	char msg[]="CloseConnection";
	unsigned int len=strlen(msg)+LoadSave::stringpad;
	char * buf = (char*)visDepth->getWriteBuffer(len);
	if(buf==NULL) {
		std::cerr << "Could not get buffer for closing packet" << std::endl;
		return false;
	}
	unsigned int used=LoadSave::encode(msg,buf,len);
	if(used==0)
		std::cerr << "Could not write close packet" << std::endl;
	visDepth->write(used);
	return true;
}

#endif

/*! @file
 * @brief Implements DepthCam, which forwards images from camera over wireless
 * @author ejt (Creator)
 */
