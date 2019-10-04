#include <sstream>

#include "Vision/RawCameraGenerator.h"
#include "Vision/RLEGenerator.h"
#include "Vision/RegionGenerator.h"
#include "Vision/SegmentedColorGenerator.h"

#include "DualCoding/SketchData.h"
#include "DualCoding/ShapeBlob.h"
#include "DualCoding/Sketch.h"

#include "Vision/VisualOdometry/VisualOdometry.h"
#include "Crew/Lookout.h"
#include "Crew/MapBuilder.h"
#ifdef TGT_HAS_WALK
#  include "Crew/Pilot.h"
#endif
#if (defined(TGT_HAS_ARMS) || !defined(STRICT_TGT_MODEL)) && !defined(TGT_IS_AIBO)
#  include "Crew/Grasper.h"
#endif
#include "Localization/ShapeBasedParticleFilter.h"

#include "ViewerConnection.h"  // for port numbers and buffer sizes
#include "VRmixin.h"

#include "Behaviors/Controller.h"

using namespace std;

namespace DualCoding {

//----------------------------------------------------------------

VRmixin* VRmixin::theOne=NULL;

//! static function allows us to specify intialization order because static within a function isn't created until the function is called
template<ReferenceFrameType_t _refFrameType,int const init_id, size_t const _width, size_t const _height>
static SketchSpace& createStaticSkS(const std::string& _name) {
  static SketchSpace SkS(_name,_refFrameType,init_id,_width,_height);
  //  cout << _name << " space is constructed\n";
  return SkS;
}

SketchSpace& VRmixin::getCamSkS() {
  return createStaticSkS<camcentric,10000,CameraResolutionX,CameraResolutionY>("cam");
}
SketchSpace& VRmixin::getLocalSkS() {
  return createStaticSkS<egocentric,20000,WORLD_WIDTH,WORLD_HEIGHT>("local"); 
}
SketchSpace& VRmixin::getWorldSkS() {
  return createStaticSkS<allocentric,30000,WORLD_WIDTH,WORLD_HEIGHT>("world");
}
ShapeSpace& VRmixin::getGroundShS() {
  static ShapeSpace ShS(&VRmixin::getCamSkS(),90000,"ground",egocentric);
  return ShS;
}

SketchSpace& VRmixin::camSkS=VRmixin::getCamSkS();
ShapeSpace& VRmixin::camShS=VRmixin::getCamSkS().getDualSpace();

ShapeSpace& VRmixin::groundShS=VRmixin::getGroundShS();

SketchSpace& VRmixin::localSkS=VRmixin::getLocalSkS();
ShapeSpace& VRmixin::localShS=VRmixin::getLocalSkS().getDualSpace();

SketchSpace& VRmixin::worldSkS=VRmixin::getWorldSkS();
ShapeSpace& VRmixin::worldShS=VRmixin::getWorldSkS().getDualSpace();

Shape<AgentData> VRmixin::theAgent;

Socket *VRmixin::camDialogSock=NULL;
Socket *VRmixin::camSketchSock=NULL;
Socket *VRmixin::localDialogSock=NULL;
Socket *VRmixin::localSketchSock=NULL;
Socket *VRmixin::worldDialogSock=NULL;
Socket *VRmixin::worldSketchSock=NULL;

MapBuilder* VRmixin::mapBuilder = NULL;
Lookout* VRmixin::lookout = NULL;
#ifdef TGT_HAS_WALK
Pilot* VRmixin::pilot = NULL;
#endif
#if (defined(TGT_HAS_ARMS) || !defined(STRICT_TGT_MODEL)) && !defined(TGT_IS_AIBO)
Grasper* VRmixin::grasper = NULL;
#endif

VisualOdometry* VRmixin::imageOdometry = NULL;

ShapeBasedParticleFilter* VRmixin::particleFilter = NULL; // will be set by startCrew

unsigned int VRmixin::instanceCount = 0;
unsigned int VRmixin::crewCount = 0;

bool VRmixin::isWalkingFlag = false;


std::vector<ShapeRoot> VRmixin::drawShapes;

VRmixin::VRmixin() {
  if ( instanceCount++ == 0 ) {
    // only want to do the following once
    cout << "Initializing VRmixin statics" << endl;
    SketchRoot::reset();
    if (theOne != NULL) {
      if ( ! mapBuilder->isRetained() )
				cerr << "VRmixin statics already constructed!?!?!" << endl;
      return;
    }
    theOne=this;
	
    camSkS.requireIdx();
    localSkS.requireIdx();
    worldSkS.requireIdx();
		
    camDialogSock=wireless->socket(Socket::SOCK_STREAM, 1024, DIALOG_BUFFER_SIZE);
    camSketchSock=wireless->socket(Socket::SOCK_STREAM, 1024, SKETCH_BUFFER_SIZE);
    worldDialogSock=wireless->socket(Socket::SOCK_STREAM, 1024, DIALOG_BUFFER_SIZE);
    worldSketchSock=wireless->socket(Socket::SOCK_STREAM, 1024, SKETCH_BUFFER_SIZE);
    localDialogSock=wireless->socket(Socket::SOCK_STREAM, 1024, DIALOG_BUFFER_SIZE);
    localSketchSock=wireless->socket(Socket::SOCK_STREAM, 1024, SKETCH_BUFFER_SIZE);
		
    wireless->setReceiver(camDialogSock->sock, &camDialogSockCallback);
    wireless->setReceiver(worldDialogSock->sock, &worldDialogSockCallback);
    wireless->setReceiver(localDialogSock->sock, &localDialogSockCallback);
		
    wireless->setDaemon(camDialogSock,   true);
    wireless->setDaemon(camSketchSock,      true);
    wireless->setDaemon(worldDialogSock, true);
    wireless->setDaemon(worldSketchSock,    true);
    wireless->setDaemon(localDialogSock, true);
    wireless->setDaemon(localSketchSock,    true);
		
    wireless->listen(camDialogSock,   CAM_DIALOG_PORT);
    wireless->listen(camSketchSock,      CAM_SKETCH_PORT);
    wireless->listen(worldDialogSock, WORLD_DIALOG_PORT);
    wireless->listen(worldSketchSock,    WORLD_SKETCH_PORT);
    wireless->listen(localDialogSock, LOCAL_DIALOG_PORT);
    wireless->listen(localSketchSock,    LOCAL_SKETCH_PORT);
		
    camSkS.viewer->setDialogSocket(camDialogSock,     CAM_DIALOG_PORT);
    camSkS.viewer->setSketchSocket(camSketchSock,           CAM_SKETCH_PORT);
    localSkS.viewer->setDialogSocket(localDialogSock, LOCAL_DIALOG_PORT);
    localSkS.viewer->setSketchSocket(localSketchSock,       LOCAL_SKETCH_PORT);
    worldSkS.viewer->setDialogSocket(worldDialogSock, WORLD_DIALOG_PORT);
    worldSkS.viewer->setSketchSocket(worldSketchSock,       WORLD_SKETCH_PORT);

    theAgent = Shape<AgentData>(worldShS);
    theAgent->setColor(rgb(0,0,255));
    theAgent->setObstacle(false);
  }

}

VRmixin::~VRmixin() {
  if ( --instanceCount == 0 ) {
    // if ( mapBuilder->isRetained() ) return;
    cout << "Destructing VRmixin statics" << endl;
    if (theOne == NULL) {
      cerr << "VRmixin statics already destructed!?!?!" << endl;
      return;
    }
    theOne=NULL;
    
    wireless->setDaemon(camDialogSock,  false);
    wireless->setDaemon(camSketchSock,     false);
    wireless->setDaemon(localDialogSock,false);
    wireless->setDaemon(localSketchSock,   false);
    wireless->setDaemon(worldDialogSock,false);
    wireless->setDaemon(worldSketchSock,   false);
    
    wireless->close(camSketchSock->sock);
    wireless->close(camDialogSock->sock);
    wireless->close(localSketchSock->sock);
    wireless->close(localDialogSock->sock);
    wireless->close(worldSketchSock->sock);
    wireless->close(worldDialogSock->sock);
		
    // clear each ShapeSpace first because it may contain rendering links to the SketchSpace
    drawShapes.clear();

    camShS.clear();
    camSkS.bumpRefreshCounter(); // release visible sketches
    camSkS.clear();
    
    localShS.clear();
    localSkS.bumpRefreshCounter(); // release visible sketches
    localSkS.clear();
    
    theAgent = Shape<AgentData>();
    worldShS.clear();
    worldSkS.bumpRefreshCounter(); // release visible sketches
    worldSkS.clear();
		
    camSkS.freeIndexes();
    localSkS.freeIndexes();
    worldSkS.freeIndexes();

    SketchRoot::reset();
  }
}

void VRmixin::startCrew() {
  if ( crewCount++ == 0 ) {
    cout << "Starting crew." << endl;
    mapBuilder = new MapBuilder;
    lookout = new Lookout;
#ifdef TGT_HAS_WALK
    pilot = new Pilot;
#endif
#if (defined(TGT_HAS_ARMS) || !defined(STRICT_TGT_MODEL)) && !defined(TGT_IS_AIBO)
    grasper = new Grasper;
#endif
    if ( particleFilter == NULL )
      particleFilter = new ShapeBasedParticleFilter(camShS,localShS,worldShS);

    // if(imageOdometry == NULL)
    //   imageOdometry = new CurrVisualOdometry();

    mapBuilder->start();
    lookout->start();
#ifdef TGT_HAS_WALK
    pilot->start();
#endif
#if (defined(TGT_HAS_ARMS) || !defined(STRICT_TGT_MODEL)) && !defined(TGT_IS_AIBO)
    grasper->start();
#endif
  }
}

void VRmixin::stopCrew() {
  if ( --crewCount == 0 ) {
    cout << "Stopping crew." << endl;
    // reference counting should delete these:
#if (defined(TGT_HAS_ARMS) || !defined(STRICT_TGT_MODEL)) && !defined(TGT_IS_AIBO)
    grasper->stop();
#endif
#ifdef TGT_HAS_WALK
    pilot->stop();
#endif
    lookout->stop();
    mapBuilder->stop();
    SketchRoot::reset();
    delete particleFilter;
    particleFilter = NULL;
    delete imageOdometry;
    imageOdometry = NULL;
  }
}
	
void VRmixin::requireCrew(const std::string &memberName) {
		if ( crewCount == 0 )
			std::cerr << "\n*** ERROR: " << memberName
								<< " invoked outside of VisualRoutinesStateNode / VRmixin ***" << std::endl;
}

#ifdef TGT_HAS_CAMERA
void VRmixin::projectToGround() {
	projectToGround(kine->linkToBase(CameraFrameOffset));
}
#endif

void VRmixin::projectToGround(const fmat::Transform& camToBase) {
	projectToGround(camToBase, kine->calculateGroundPlane());
}

void VRmixin::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {
  groundShS.clear();
  groundShS.importShapes(camShS.allShapes());
  vector<ShapeRoot> &groundShapes_vec = groundShS.allShapes();
  for(size_t i = 0; i < groundShapes_vec.size(); i++)
    groundShapes_vec[i]->projectToGround(camToBase, groundplane);
}

void VRmixin::refreshSketchWorld() {
  if ( Controller::theOneController != NULL)
    Controller::theOneController->refreshSketchWorld();
}

void VRmixin::refreshSketchLocal() {
  if ( Controller::theOneController != NULL)
    Controller::theOneController->refreshSketchLocal();
}

void VRmixin::refreshSketchCamera() {
  if ( Controller::theOneController != NULL)
    Controller::theOneController->refreshSketchCamera();
}

void VRmixin::autoRefreshSketchWorld() {
  if (autoRefreshWorldAllowed)
    refreshSketchWorld();
}

void VRmixin::autoRefreshSketchLocal() {
  if (autoRefreshLocalAllowed)
    refreshSketchLocal();
}

void VRmixin::autoRefreshSketchCamera() {
  if (autoRefreshCameraAllowed)
    refreshSketchCamera();
}

int VRmixin::camDialogSockCallback(char *buf, int bytes) {
  static std::string incomplete;
  dialogCallback(buf, bytes, incomplete, theOne->camSkS, theOne->camShS);
  return 0;
}

int VRmixin::localDialogSockCallback(char *buf, int bytes) {
  static std::string incomplete;
  dialogCallback(buf, bytes, incomplete, theOne->localSkS, theOne->localShS);
  return 0;
}

int VRmixin::worldDialogSockCallback(char *buf, int bytes) {
  static std::string incomplete;
  dialogCallback(buf, bytes, incomplete, theOne->worldSkS, theOne->worldShS);
  return 0;
}

void VRmixin::dialogCallback(char* buf, int bytes, std::string& incomplete,
			     SketchSpace& SkS, ShapeSpace& ShS) {
  std::string s(buf,bytes);
  // std::cout << "***** dialogCallback: '" << s << "'\n";
  while(s.size()>0) {
    size_t endline=s.find('\n');
    if(endline==std::string::npos) {
      incomplete+=s;
      return;
    }
    else {
      incomplete+=s.substr(0,endline);
      theOne->processSketchRequest(incomplete,SkS,ShS);
      incomplete.erase();
      s=s.substr(endline+1);
    }
  }
  return;
}

bool VRmixin::encodeSketch(const SketchDataRoot& image)
{
  unsigned int avail = SKETCH_BUFFER_SIZE-1;
  Socket* sketchSock = image.getSpace().viewer->getSketchSocket();
  char* buf=(char*)sketchSock->getWriteBuffer(avail);
  ASSERTRETVAL(buf!=NULL,"could not get buffer",false);
  unsigned int used = image.saveBuffer(buf, avail);
  sketchSock->write(used);
  return true;
}

//! Import a color-segmented image as a Sketch<uchar>
Sketch<uchar> VRmixin::sketchFromSeg() {
  Sketch<uchar> cam(camSkS, "camimage");
  cam->setColorMap(segMap);
  size_t const npixels = cam->getNumPixels();
  cmap_t* seg_image = ProjectInterface::defSegmentedColorGenerator->getImage(CAM_LAYER,CAM_CHANNEL);
  memcpy(cam->getRawPixels(), seg_image, npixels);
  return cam;
}

//! Import channel n as a Sketch<uchar>
Sketch<uchar> VRmixin::sketchFromChannel(const RawCameraGenerator::channel_id_t chan) {
  const RawCameraGenerator::channel_id_t the_chan =
    (chan >= 0 && chan < RawCameraGenerator::NUM_CHANNELS) ? chan : RawCameraGenerator::CHAN_Y;
  Sketch<uchar> cam(camSkS,"sketchFromChannel");
  cam->setColorMap(grayMap);
  uchar* campixels = cam->getRawPixels();
  int const incr = ProjectInterface::defRawCameraGenerator->getIncrement(CAM_LAYER);
  int const skip = ProjectInterface::defRawCameraGenerator->getSkip(CAM_LAYER);
  uchar* chan_ptr = ProjectInterface::defRawCameraGenerator->getImage(CAM_LAYER,the_chan);
  if(chan_ptr==NULL) {
    memset(campixels,0,cam->getHeight()*cam->getWidth());
  } else {
    chan_ptr -= incr;  // back up by one pixel to prepare for loop
    for (unsigned int row = 0; row < cam->getHeight(); row++) {
      for (unsigned int col = 0; col < cam->getWidth(); col++)
        *campixels++ = *(chan_ptr += incr);
      chan_ptr += skip;
    }
  }
  return cam;
}

Sketch<uchar> VRmixin::sketchFromRawY() {
  return sketchFromChannel(RawCameraGenerator::CHAN_Y);
}
  
Sketch<usint> VRmixin::sketchFromDepth() {
  Sketch<usint> depth(camSkS,"sketchFromDepth");
  depth->setColorMap(jetMapScaled);
  usint* depthpixels = depth->getRawPixels();
  int const incr = ProjectInterface::defRawDepthGenerator->getIncrement(CAM_LAYER);
  int const skip = ProjectInterface::defRawDepthGenerator->getSkip(CAM_LAYER);
  uchar* chanY_ptr = ProjectInterface::defRawDepthGenerator->getImage(CAM_LAYER, RawCameraGenerator::CHAN_Y);
  uchar* chanU_ptr = ProjectInterface::defRawDepthGenerator->getImage(CAM_LAYER, RawCameraGenerator::CHAN_U);
  for (unsigned int row = 0; row < depth->getHeight(); row++) {
    for (unsigned int col = 0; col < depth->getWidth(); col++)
      *depthpixels++ = *(chanY_ptr += incr) | (*(chanU_ptr += incr))<<8;
    chanY_ptr += skip;
    chanU_ptr += skip;
  }
  return depth;
}

Sketch<yuv> VRmixin::sketchFromYUV() {
  Sketch<yuv> cam(camSkS,"sketchFromYUV");
  cam->setColorMap(segMap);
  yuv* campixels = cam->getRawPixels();
  int const incr = ProjectInterface::defRawCameraGenerator->getIncrement(CAM_LAYER);
  int const skip = ProjectInterface::defRawCameraGenerator->getSkip(CAM_LAYER);
  uchar* chanY_ptr = ProjectInterface::defRawCameraGenerator->getImage(CAM_LAYER,RawCameraGenerator::CHAN_Y);
  uchar* chanU_ptr = ProjectInterface::defRawCameraGenerator->getImage(CAM_LAYER,RawCameraGenerator::CHAN_U);
  uchar* chanV_ptr = ProjectInterface::defRawCameraGenerator->getImage(CAM_LAYER,RawCameraGenerator::CHAN_V);
  if ( chanY_ptr == NULL )
    memset(campixels,0,cam->getHeight()*cam->getWidth()*sizeof(rgb));
  else {
    // back up by one pixel to prepare for loop
    chanY_ptr -= incr;
    chanU_ptr -= incr;
    chanV_ptr -= incr;
    for (unsigned int row = 0; row < cam->getHeight(); row++) {
      for (unsigned int col = 0; col < cam->getWidth(); col++)
        *campixels++ = yuv(*(chanY_ptr += incr), *(chanU_ptr += incr), *(chanV_ptr += incr));
      chanY_ptr += skip;
      chanU_ptr += skip;
      chanV_ptr += skip;
    }
  }
  return cam;
}

//! Import the results of the region generator as a vector of Shape<BlobData>
vector<Shape<BlobData> >
VRmixin::getBlobsFromRegionGenerator(const color_index color, 
				     const int minarea,
				     const BlobData::BlobOrientation_t orient, 
				     const coordinate_t height,
				     const int maxblobs) {
  vector<Shape<BlobData> > result;
  const CMVision::run<uchar> *sketch_buffer = reinterpret_cast<const CMVision::run<uchar>*>
    (ProjectInterface::defRLEGenerator->getImage(CAM_LAYER,CAM_CHANNEL));
  const CMVision::color_class_state* ccs = reinterpret_cast<const CMVision::color_class_state*>
    (ProjectInterface::defRegionGenerator->getImage(CAM_LAYER,CAM_CHANNEL));
  //  cout << "Color " << color << " name '" << ccs[color].name 
  //   << "' has " << ccs[color].num << " regions." << endl;
  const rgb rgbvalue = ProjectInterface::getColorRGB(color);
  const CMVision::region* list_head = ccs[color].list;
  for (int i=0; list_head!=NULL && i<maxblobs && list_head->area >= minarea; list_head = list_head->next, i++) {
    BlobData* blobdat = BlobData::new_blob(camShS,*list_head, sketch_buffer, orient, height, rgbvalue);
    result.push_back(Shape<BlobData>(blobdat));
  }
  return result;
}

void VRmixin::processSketchRequest(const std::string &line,
				   SketchSpace& SkS, 
				   ShapeSpace& ShS)
{
  Socket* dialogSock = SkS.viewer->getDialogSocket();
  if(line.compare(0,strlen("size"),"size")==0) {
    dialogSock->printf("size begin\n");
    dialogSock->printf("width %d\nheight %d\n",int(SkS.getWidth()),int(SkS.getHeight()));
    dialogSock->printf("size end\n");
  }
  else if(line.compare(0,strlen("list"),"list")==0) {
    dialogSock->printf("list begin\n");
    SkS.viewer->writeBigString(SkS.getTmatForGUI());	
    SkS.viewer->writeBigString(SkS.getSketchListForGUI());	
    SkS.viewer->writeBigString(ShS.getShapeListForGUI());	
    dialogSock->printf("list end\n");
  } else if(line.compare(0,strlen("colors"),"colors")==0) {
    dialogSock->printf("colors begin\n");
    const CMVision::color_class_state *ccs = ProjectInterface::defSegmentedColorGenerator->getColors();
    unsigned int i;
    for (i = 0; i < ProjectInterface::defSegmentedColorGenerator->getNumColors(); i++) {
        char color_buffer[512];
        sprintf(color_buffer,"%d,%d,%d,%s\n",ccs[i].color.red,ccs[i].color.green,ccs[i].color.blue,ccs[i].name);
        SkS.viewer->writeBigString(color_buffer);
    }
    dialogSock->printf("colors end\n");
  } else if(line.compare(0,strlen("get"),"get")==0) {
    std::string tempstring = line.substr(strlen("get"),
					 line.length()-strlen("get"));
    std::istringstream ist(tempstring);
    int requested_id = -1;
    ist >> requested_id;
    dialogSock->printf("get read:%d\n",requested_id);
    SketchDataRoot* sketchptr=(SkS.retrieveSketch(requested_id));
    if(sketchptr != NULL)
      encodeSketch(*sketchptr);
    dialogSock->printf("get end\n");
  } else {
    dialogSock->printf("Invalid command\n");
  }
}

bool VRmixin::autoRefreshWorldAllowed = true;
bool VRmixin::autoRefreshLocalAllowed = true;
bool VRmixin::autoRefreshCameraAllowed = true;

Point VRmixin::robotObstaclesPt = Point(0, 0, 0, allocentric);
AngTwoPi VRmixin::robotObstaclesOri = 0;

} // namespace
