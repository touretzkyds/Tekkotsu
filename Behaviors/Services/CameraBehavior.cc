#include "CameraBehavior.h"
#include "Events/EventRouter.h"
#include "Events/TextMsgEvent.h"
#include "Shared/RobotInfo.h"
#include "Wireless/Socket.h"
#include "Shared/WorldState.h"
#include "Sound/SoundManager.h"
#include "Shared/Config.h"
#include "Shared/ProjectInterface.h"
#include "Motion/MMAccessor.h"
#include "IPC/SharedObject.h"

#include "Vision/FilterBankGenerator.h"
#include "Vision/RawCameraGenerator.h"
#include "Vision/InterleavedYUVGenerator.h"
#include "Vision/JPEGGenerator.h"
#include "Vision/PNGGenerator.h"

#include "Shared/RobotInfo.h"
#ifdef TGT_HAS_LEDS
#  include "Motion/LedMC.h"
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>

REGISTER_BEHAVIOR_MENU_OPT(CameraBehavior,"Background Behaviors",BEH_NONEXCLUSIVE);

void
CameraBehavior::doStart() {
	BehaviorBase::doStart();
	if(capabilities.findButtonOffset("HeadBut")!=-1U) // if there is a head button, use it for the trigger
		camera_click.setSourceID(capabilities.getButtonOffset("HeadBut"));
	// (otherwise, just stick with the default button 0 as set in camera_click constructor)
	initIndex();
	sndman->loadFile("camera.wav");
#ifdef TGT_HAS_LEDS
	ledID=motman->addPersistentMotion(SharedObject<LedMC>());
#endif
	erouter->addListener(this,camera_click);
	erouter->addListener(this,EventBase::textmsgEGID);
}

void CameraBehavior::doStop() {
	erouter->removeListener(this);
	sndman->releaseFile("camera.wav");
	motman->removeMotion(ledID);
	BehaviorBase::doStop();
}

	
/*! The format used depends on the current config settings.  If JPEG
 *  is the current choice, then a JPEG file will be written.
 *  Otherwise, RawCameraGenerator::saveFile() will be called.
 */
void
CameraBehavior::doEvent() {
	if(event->getGeneratorID()==EventBase::textmsgEGID) {
		const TextMsgEvent * txt=dynamic_cast<const TextMsgEvent*>(event);
		if(txt==NULL || txt->getText()!="camera")
			return;
	} else if(event->shorterThan(camera_click))
		return;

	{
#if defined(TGT_ERS7) || defined(TGT_ERS210) || defined(TGT_ERS220) || defined(TGT_ERS2xx)
		MMAccessor<LedMC> leds(ledID);
		leds->cset(FaceLEDMask,2.0f/3.0f);
		leds->set(TopBrLEDMask,1);
#elif defined(TGT_HAS_LEDS)
		MMAccessor<LedMC> leds(ledID);
		leds->set(AllLEDMask,1);
#endif
	}

	if(config->vision.rawcam.compression==Config::vision_config::RawCamConfig::COMPRESS_NONE) {
		//this is our own odd little format, would be nice to save a TIFF or something instead

		// open file
		FILE * f=openNextFile(".raw");
		if(f==NULL) //error message already displayed in openNextFile()
			return;

		//! write actual image data
		if(config->vision.rawcam.encoding==Config::vision_config::ENCODE_COLOR) {
			FilterBankGenerator * gen=ProjectInterface::defInterleavedYUVGenerator; // just an alias for readability
			gen->selectSaveImage(ProjectInterface::doubleLayer,InterleavedYUVGenerator::CHAN_YUV);
			unsigned int len=gen->saveFileStream(f);
			if(len==0) {
				serr->printf("Error saving file\n");
				sndman->playFile(config->controller.error_snd);
				return;
			}
		} else if(config->vision.rawcam.encoding==Config::vision_config::ENCODE_SINGLE_CHANNEL) {
			FilterBankGenerator * gen=ProjectInterface::defRawCameraGenerator; // just an alias for readability
			gen->selectSaveImage(ProjectInterface::doubleLayer,config->vision.rawcam.channel);
			unsigned int len=gen->saveFileStream(f);
			if(len==0) {
				serr->printf("Error saving file\n");
				sndman->playFile(config->controller.error_snd);
				return;
			}
		}
		
		// close file
		fclose(f);

	} else if(config->vision.rawcam.compression==Config::vision_config::RawCamConfig::COMPRESS_JPEG) {
		//save a JPEG image
		JPEGGenerator * jpeg=NULL; // we'll set this to pick between the color jpeg or a single channel grayscale jpeg
		unsigned int chan=0; // and this will hold the channel to use out of that jpeg generator
		if(config->vision.rawcam.encoding==Config::vision_config::ENCODE_COLOR)
			jpeg=dynamic_cast<JPEGGenerator*>(ProjectInterface::defColorJPEGGenerator);
		else if(config->vision.rawcam.encoding==Config::vision_config::ENCODE_SINGLE_CHANNEL) {
			jpeg=dynamic_cast<JPEGGenerator*>(ProjectInterface::defGrayscaleJPEGGenerator);
			chan=config->vision.rawcam.channel;
		}
		if(jpeg!=NULL) {
			unsigned int tmp_q=jpeg->getQuality(); //temporary storage so we can reset the default
			jpeg->setQuality(92);
			
			// open file
			FILE * f=openNextFile(".jpg");
			if(f==NULL) //error message already displayed in openNextFile()
				return;

			//! write actual image data
			unsigned char * imgbuf=jpeg->getImage(ProjectInterface::doubleLayer,chan);
			unsigned int writ=fwrite(imgbuf,jpeg->getImageSize(ProjectInterface::doubleLayer,chan),1,f);
			if(writ==0) {
				serr->printf("Error saving file\n");
				sndman->playFile(config->controller.error_snd);
				return;
			}

			// close file
			fclose(f);
			
			jpeg->setQuality(tmp_q);
		}

	} else if(config->vision.rawcam.compression==Config::vision_config::RawCamConfig::COMPRESS_PNG) {
		//save a PNG image
		PNGGenerator * png=NULL; // we'll set this to pick between the color png or a single channel grayscale png
		unsigned int chan=0; // and this will hold the channel to use out of that png generator
		if(config->vision.rawcam.encoding==Config::vision_config::ENCODE_COLOR)
			png=dynamic_cast<PNGGenerator*>(ProjectInterface::defColorPNGGenerator);
		else if(config->vision.rawcam.encoding==Config::vision_config::ENCODE_SINGLE_CHANNEL) {
			png=dynamic_cast<PNGGenerator*>(ProjectInterface::defGrayscalePNGGenerator);
			chan=config->vision.rawcam.channel;
		}
		if(png!=NULL) {
			// open file
			FILE * f=openNextFile(".png");
			if(f==NULL) //error message already displayed in openNextFile()
				return;
			
			//! write actual image data
			unsigned char * imgbuf=png->getImage(ProjectInterface::doubleLayer,chan);
			unsigned int writ=fwrite(imgbuf,png->getImageSize(ProjectInterface::doubleLayer,chan),1,f);
			if(writ==0) {
				serr->printf("Error saving file\n");
				sndman->playFile(config->controller.error_snd);
				return;
			}
			
			// close file
			fclose(f);
		}
	}
	
	{
#if defined(TGT_ERS7) || defined(TGT_ERS210) || defined(TGT_ERS220) || defined(TGT_ERS2xx)
		MMAccessor<LedMC> leds(ledID);
		leds->clear();
		leds->flash(TopBrLEDMask,700);
		leds->flash(TopLLEDMask|TopRLEDMask,500);
		leds->flash(MidLLEDMask|MidRLEDMask,300);
		leds->flash(BotLLEDMask|BotRLEDMask,100);
#elif defined(TGT_HAS_LEDS)
		MMAccessor<LedMC> leds(ledID);
		leds->clear();
		if(NumLEDs>3)
			leds->flash(1<<3,700);
		if(NumLEDs>2)
			leds->flash(1<<2,500);
		if(NumLEDs>1)
			leds->flash(1<<1,300);
		if(NumLEDs>0)
			leds->flash(1<<0,100);
#endif
	}

	sout->printf("done\n");
}

FILE *
CameraBehavior::openNextFile(const std::string& ext) {
	FILE * f=fopen(getNextName(ext).c_str(),"w+");
	if(f==NULL) {
		serr->printf("Error opening file\n");
		sndman->playFile(config->controller.error_snd);
		return NULL;
	}
	sndman->playFile("camera.wav");
	return f;
}


std::string
CameraBehavior::getNextName(const std::string& ext) {
	char tmp[100];
	snprintf(tmp,100,"data/img%05d%s",index++,ext.c_str());
	std::string ans=config->portPath(tmp);
	sout->printf("Saving `%s'...",ans.c_str());
	return ans;
}

void
CameraBehavior::initIndex() {
	std::string path=config->portPath("data/");
	DIR* dir=opendir(path.c_str());
	if(dir==NULL) {
		serr->printf("bad path: `%s'\n",path.c_str());
		return;
	}
	struct dirent * ent=readdir(dir);
	while(ent!=NULL) {
		struct stat s;
		std::string fullpath=path+ent->d_name;
		int err=stat(fullpath.c_str(),&s);
		if(err!=0) {
			serr->printf("File disappeared: %s\n",fullpath.c_str());
			return;
		}
		if((s.st_mode&S_IFDIR)==0 && strncasecmp(ent->d_name,"IMG",3)==0) {
			unsigned int cur=atoi(&ent->d_name[3]);
			if(cur>index)
				index=cur;
		}
		ent=readdir(dir);
	}
	closedir(dir);
	index++; //set index to next unused
	sout->printf("The next saved image will go to %simg%05d\n",path.c_str(),index);
}

/*! @file
 * @brief Implements CameraBehavior, for taking pictures
 * @author ejt (Creator)
 */

