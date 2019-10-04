#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_CAMERA) || defined(TGT_HAS_WEBCAM)

#include "Behaviors/Controller.h"
#include "DualCoding/ShapeEllipse.h"
#include "DualCoding/ShapePolygon.h"
#include "Crew/MapBuilder.h"
#include "DualCoding/ShapeLine.h"
#include "DualCoding/Point.h"
#include "DualCoding/PolygonData.h"
#include "DualCoding/SketchSpace.h"
#include "DualCoding/VRmixin.h"
#include "Events/EventRouter.h"
#include "Events/FilterBankEvent.h"
#include "Motion/Kinematics.h"
#include "Shared/ProjectInterface.h"
#include "Shared/debuget.h"
#include "Vision/Graphics.h"
#include "Vision/RawCameraGenerator.h"
#include "Vision/JPEGGenerator.h"
#include "Wireless/Wireless.h"

#include "RawCam.h"

using namespace DualCoding;

REGISTER_BEHAVIOR_MENU_OPT(RawCam,"TekkotsuMon",BEH_NONEXCLUSIVE);

RawCam* RawCam::theOne=NULL;

RawCam::RawCam() : CameraStreamBehavior("RawCam",visRaw),
                   visRaw(NULL), packet(NULL), cur(NULL), avail(0), max_buf(0), lastProcessedTime(0) {
  ASSERT(theOne==NULL,"there was already a RawCam running!");
  theOne=this;
}

void RawCam::doStart() {
  BehaviorBase::doStart();
  setupServer();
  erouter->addListener(this,EventBase::visRawCameraEGID,
                       ProjectInterface::visRawCameraSID,
                       EventBase::deactivateETID);
  erouter->addListener(this,EventBase::visJPEGEGID,
                       ProjectInterface::visColorJPEGSID,
                       EventBase::deactivateETID);
  erouter->addListener(this,EventBase::visJPEGEGID,
                       ProjectInterface::visGrayscaleJPEGSID,
                       EventBase::deactivateETID);
}

void RawCam::doStop() {
  erouter->removeListener(this);
  closeServer();
  BehaviorBase::doStop();
}

void RawCam::doEvent() {
  if ( !wireless->isConnected(visRaw->sock) )
    return;
  if ((config->vision.rawcam.transport==0 && visRaw->getTransport()==Socket::SOCK_STREAM)
     || (config->vision.rawcam.transport==1 && visRaw->getTransport()==Socket::SOCK_DGRAM)) {
    //reset the socket
    closeServer();
    setupServer();
    return;
  }
  try {
    const FilterBankEvent* fbke=dynamic_cast<const FilterBankEvent*>(event);
    if (fbke==NULL) {
      CameraStreamBehavior::doEvent();
      return;
    }
    if ((get_time() - lastProcessedTime) < config->vision.rawcam.interval) {// not enough time has gone by
      return;
    }
    /* // turning these off enables individual channel compression
       if (config->vision.rawcam.compression==Config::vision_config::COMPRESS_NONE &&
       event->getGeneratorID()!=EventBase::visRawCameraEGID)
       return;
       if (config->vision.rawcam.compression==Config::vision_config::COMPRESS_JPEG &&
       event->getGeneratorID()!=EventBase::visJPEGEGID)
       return; */
    // draw shapes into the camera buffer if user requested any
    if ( fbke->getGeneratorID() == EventBase::visRawCameraEGID && ! VRmixin::drawShapes.empty() )
      drawShapesIntoBuffer(*fbke);
    // now write out the camera image
    if (config->vision.rawcam.encoding==Config::vision_config::ENCODE_COLOR) {
      if (!writeColor(*fbke)) {
        if (packet) { //packet was opened, need to close it
          cur=packet; // don't send anything
          closePacket();
        }
        //error message should already be printed in writeColor
        //ASSERTRET(false,"serialization failed");
      }
    } else if (config->vision.rawcam.encoding==Config::vision_config::ENCODE_SINGLE_CHANNEL) {
      //std::cout << "Encode_Single_channel called " << std::endl;
      if (!writeSingleChannel(*fbke)) {
        if (packet) { //packet was opened, need to close it
          cur=packet; // don't send anything
          closePacket();
        }
        //error message should already be printed in writeSingleChannel
        //ASSERTRET(false,"serialization failed");
      }
    }
    else {
      serr->printf("%s: Bad rawcam.encoding setting\n",getName().c_str());
    }
  } catch(...) {
    if (packet) { //packet was opened, need to close it
      cur=packet; // don't send anything
      closePacket();
    }
    // typically this is a per-frame recurring error, so let's just stop now
    serr->printf("%s: exception generated during image serialization, stopping stream.\n",getName().c_str());
    stop();
    throw;
  }
}

unsigned int RawCam::getSourceLayer(unsigned int chan, unsigned int numLayers) {
  if (config->vision.rawcam.encoding==Config::vision_config::ENCODE_SINGLE_CHANNEL) {
    if (config->vision.rawcam.channel!=(int)chan)
      return -1U;
    return numLayers-1-config->vision.rawcam.y_skip;
  }
  // must be full-color
  switch(chan) {
  case RawCameraGenerator::CHAN_Y:
    if (config->vision.rawcam.compression==Config::vision_config::RawCamConfig::COMPRESS_JPEG) {
      if (config->vision.rawcam.y_skip-config->vision.rawcam.uv_skip == 1)
        return numLayers-1-config->vision.rawcam.uv_skip;
    }
    return numLayers-1-config->vision.rawcam.y_skip;
  case RawCameraGenerator::CHAN_U:
  case RawCameraGenerator::CHAN_V:
    if (config->vision.rawcam.compression==Config::vision_config::RawCamConfig::COMPRESS_JPEG) {
      if (config->vision.rawcam.uv_skip-config->vision.rawcam.y_skip == 1)
        return numLayers-1-config->vision.rawcam.y_skip;
    }
    return numLayers-1-config->vision.rawcam.uv_skip;
  default: // other channels, i.e. Y-derivatives
    return -1U;
  }
}

unsigned int RawCam::getSourceYLayer(unsigned int numLayers) {
  return getSourceLayer(RawCameraGenerator::CHAN_Y,numLayers);
}

unsigned int RawCam::getSourceULayer(unsigned int numLayers) {
  return getSourceLayer(RawCameraGenerator::CHAN_U,numLayers);
}

unsigned int RawCam::getSourceVLayer(unsigned int numLayers) {
  return getSourceLayer(RawCameraGenerator::CHAN_V,numLayers);
}

void RawCam::drawShapesIntoBuffer(const FilterBankEvent &fbke) {
  unsigned int layer=fbke.getNumLayers() - 1 - config->vision.rawcam.y_skip;
  unsigned int chanY = RawCameraGenerator::CHAN_Y;
  unsigned int chanU = RawCameraGenerator::CHAN_U;
  unsigned int chanV = RawCameraGenerator::CHAN_V;

  Graphics g(*fbke.getSource(), layer, chanY, chanU, chanV);
  const fmat::Transform baseToCam = kine->baseToLink(CameraFrameOffset);
  const fmat::Transform worldToCam = baseToCam * VRmixin::mapBuilder->worldToLocalMatrix;
  const unsigned int halfWidth = VRmixin::camSkS.getWidth() / 2;
  const unsigned int halfHeight = VRmixin::camSkS.getHeight() / 2;

  for ( std::vector<ShapeRoot>::const_iterator it = VRmixin::drawShapes.begin();
        it != VRmixin::drawShapes.end(); it++ ) {
    if ( ! it->isValid() ) continue;
    g.setColor((*it)->getColor());

    switch ((*it)->getType() ) {

    case pointDataType: {
      const Shape<PointData> &aPoint = ShapeRootTypeConst(*it,PointData);
      Point center(aPoint->getCentroid());
      if ( aPoint->BaseData::getRefFrameType() == allocentric )
        center.applyTransform(worldToCam);
      else if ( aPoint->BaseData::getRefFrameType() == egocentric )
        center.applyTransform(baseToCam);
      int px, py;
      switch ( aPoint->BaseData::getRefFrameType() ) {
      case camcentric:
      case unspecified:
        px = center.coordX();
        py = center.coordY();
        break;
      case egocentric:
      case allocentric:
        if ( center.coordZ() < 0 )
          return;
        float camXnorm, camYnorm;
        config->vision.computePixelCorrected(center.coordX(), center.coordY(), center.coordZ(), 
                                             camXnorm, camYnorm);
        px = int((camXnorm + 1) * halfWidth);
        py = int((camYnorm + 1) * halfHeight);
        break;
      }
      g.drawPoint(px, py);
      g.drawPoint(px+1, py);
      g.drawPoint(px, py+1);
      g.drawPoint(px-1, py);
      g.drawPoint(px, py-1);
      break;
    }

    case lineDataType: {
      const Shape<LineData> &aLine = ShapeRootTypeConst(*it,LineData);
      Point endPt1(aLine->end1Pt());
      Point endPt2(aLine->end2Pt());
      if ( aLine->getRefFrameType() == allocentric ) {
        endPt1.applyTransform(worldToCam);
        endPt2.applyTransform(worldToCam);
      } else if ( aLine->getRefFrameType() == egocentric ) {
        endPt1.applyTransform(baseToCam);
        endPt2.applyTransform(baseToCam);
      }
      int px, py, qx, qy;
      switch ( aLine->getRefFrameType() ) {
      case camcentric:
      case unspecified:
        px = endPt1.coordX();
        py = endPt1.coordY();
        qx = endPt2.coordX();
        qy = endPt2.coordY();
        break;
      case egocentric:
      case allocentric:
        if ( endPt1.coordZ() < 0 || endPt2.coordZ() < 0 )
          return;
        float camXnorm, camYnorm, camXnorm2, camYnorm2;
        config->vision.computePixelCorrected(endPt1.coordX(), endPt1.coordY(), endPt1.coordZ(),
                                             camXnorm, camYnorm);
        config->vision.computePixelCorrected(endPt2.coordX(), endPt2.coordY(), endPt2.coordZ(),
                                             camXnorm2, camYnorm2); 
        px = int((camXnorm + 1) * halfWidth);
        py = int((camYnorm + 1) * halfHeight);
        qx = int((camXnorm2 + 1) * halfWidth);
        qy = int((camYnorm2 + 1) * halfHeight);
        break;
      }
      g.drawLine(px, py, qx, qy);
      g.drawLine(px+1, py, qx+1, qy);  // make line thicker
      g.drawLine(px, py+1, qx, qy+1);  // make line thicker
      break;
    }

    case polygonDataType: {
      const Shape<PolygonData> &aPolygon = ShapeRootTypeConst(*it,PolygonData);
      std::vector<Point> vertices = (aPolygon)->getVertices();
      unsigned int numVertices = vertices.size();
      for (unsigned int i=0; i < numVertices; i++) {
        if ( vertices[i].getRefFrameType() == allocentric )
          vertices[i].applyTransform(worldToCam);
        else if ( vertices[i].getRefFrameType() == egocentric )
          vertices[i].applyTransform(baseToCam);
        if ( vertices[i].coordZ() < 0 )
          return;
      }
      int px[numVertices], py[numVertices];
      for (unsigned int i = 0; i < numVertices; i++) {
        switch ( vertices[i].getRefFrameType() ) {
        case camcentric:
        case unspecified:
          px[i] = vertices[i].coordX();
          py[i] = vertices[i].coordY();
          break;
        case egocentric:
        case allocentric:
          float camXnorm, camYnorm;
          config->vision.computePixelCorrected(vertices[i].coordX(), vertices[i].coordY(), vertices[i].coordZ(),
                                               camXnorm, camYnorm);
          px[i] = int((camXnorm + 1) * halfWidth);
          py[i] = int((camYnorm + 1) * halfHeight);
          break;
        }
      }
      for (unsigned int i = 0; i < numVertices-1; i++) {
        g.drawLine(px[i], py[i], px[i+1], py[i+1]);
        g.drawLine(px[i]+1, py[i], px[i+1]+1, py[i+1]); //make line thicker
        g.drawLine(px[i], py[i]+1, px[i+1], py[i+1]+1); //make line thicker
      }
      break;
    }

    case ellipseDataType: {
      const Shape<EllipseData> &aEllipse = ShapeRootTypeConst(*it,EllipseData);
      // An ellipse is not elliptical in perspective projection.  To
      // account for this, we calculate the four extreme points and
      // project each of them to camera space, then draw the ellipse
      // in four sections.
      float semiMaj = aEllipse->getSemimajor();
      float semiMin = aEllipse->getSemiminor();
      AngPi theta = aEllipse->getOrientation();
      float cosT = cos(theta);
      float sinT = sin(theta);

      Point centerPt = aEllipse->centerPt();
      Point semimajorPt1 = centerPt + Point(semiMaj*cosT, semiMaj*sinT);
      Point semimajorPt2 = centerPt - Point(semiMaj*cosT, semiMaj*sinT);
      Point semiminorPt1 = centerPt + Point(semiMin*sinT, semiMin*cosT);
      Point semiminorPt2 = centerPt - Point(semiMin*sinT, semiMin*cosT);

      if ( aEllipse->getRefFrameType() == allocentric ) {
        centerPt.applyTransform(worldToCam);
        semimajorPt1.applyTransform(worldToCam);
        semimajorPt2.applyTransform(worldToCam);
        semiminorPt1.applyTransform(worldToCam);
        semiminorPt2.applyTransform(worldToCam);
      } else if ( aEllipse->getRefFrameType() == egocentric ) {
        centerPt.applyTransform(baseToCam);
        semimajorPt1.applyTransform(baseToCam);
        semimajorPt2.applyTransform(baseToCam);
        semiminorPt1.applyTransform(baseToCam);
        semiminorPt2.applyTransform(baseToCam);
      }
      int pxi, pyi;
      float major1Len, minor1Len, major2Len, minor2Len, newOrient;
      switch ( aEllipse->getRefFrameType() ) {
      case camcentric:
      case unspecified:
        newOrient = aEllipse->getOrientation();
        pxi = round(centerPt.coordX());
        pyi = round(centerPt.coordY());
        major1Len = major2Len = aEllipse->getSemimajor();
        minor1Len = minor2Len = aEllipse->getSemiminor();
        break;
      case egocentric:
      case allocentric:
        if ( semimajorPt1.coordZ() < 0 || semimajorPt2.coordZ() < 0 ||
             semiminorPt1.coordZ() < 0 || semiminorPt2.coordZ() < 0 )
          return;
      float camXnorm, camYnorm, major1Xnorm, major1Ynorm, major2Xnorm, major2Ynorm, 
        minor1Xnorm, minor1Ynorm, minor2Xnorm, minor2Ynorm;
      config->vision.computePixelCorrected(centerPt.coordX(), centerPt.coordY(), centerPt.coordZ(),
                                           camXnorm, camYnorm);
      config->vision.computePixelCorrected(semimajorPt1.coordX(), semimajorPt1.coordY(), semimajorPt1.coordZ(),
                                           major1Xnorm, major1Ynorm);
      config->vision.computePixelCorrected(semimajorPt2.coordX(), semimajorPt2.coordY(), semimajorPt2.coordZ(),
                                           major2Xnorm, major2Ynorm);
      config->vision.computePixelCorrected(semiminorPt1.coordX(), semiminorPt1.coordY(), semiminorPt1.coordZ(),
                                           minor1Xnorm, minor1Ynorm);
      config->vision.computePixelCorrected(semiminorPt2.coordX(), semiminorPt2.coordY(), semiminorPt2.coordZ(),
                                           minor2Xnorm, minor2Ynorm);
      float px = (camXnorm + 1) * halfWidth;
      float py = (camYnorm + 1) * halfHeight;
      float major1x = (major1Xnorm + 1) * halfWidth;
      float major2x = (major2Xnorm + 1) * halfWidth;
      float major1y = (major1Ynorm + 1) * halfHeight;
      float major2y = (major2Ynorm + 1) * halfHeight;
      float minor1x = (minor1Xnorm + 1) * halfWidth;
      float minor2x = (minor2Xnorm + 1) * halfWidth;
      float minor1y = (minor1Ynorm + 1) * halfHeight;
      float minor2y = (minor2Ynorm + 1) * halfHeight;
      major1Len = sqrt((px-major1x)*(px-major1x) + (py-major1y)*(py-major1y));
      major2Len = sqrt((px-major2x)*(px-major2x) + (py-major2y)*(py-major2y));
      minor1Len = sqrt((px-minor1x)*(px-minor1x) + (py-minor1y)*(py-minor1y));
      minor2Len = sqrt((px-minor2x)*(px-minor2x) + (py-minor2y)*(py-minor2y));
      newOrient = atan2(major1y-py, major1x-px);
      pxi = round(px);
      pyi = round(py);
      break;
      }
      g.drawQuarterEllipse(pxi, pyi, -major1Len, -minor1Len, newOrient);
      g.drawQuarterEllipse(pxi, pyi, major2Len, minor2Len, newOrient);
      g.drawQuarterEllipse(pxi, pyi, major2Len, -minor1Len, newOrient);
      g.drawQuarterEllipse(pxi, pyi, -major1Len, minor2Len, newOrient);
      g.drawQuarterEllipse(pxi+1, pyi, -major1Len, -minor1Len, newOrient);
      g.drawQuarterEllipse(pxi+1, pyi, major2Len, minor2Len, newOrient);
      g.drawQuarterEllipse(pxi+1, pyi, major2Len, -minor1Len, newOrient);
      g.drawQuarterEllipse(pxi+1, pyi, -major1Len, minor2Len, newOrient);
      g.drawQuarterEllipse(pxi, pyi+1, -major1Len, -minor1Len, newOrient);
      g.drawQuarterEllipse(pxi, pyi+1, major2Len, minor2Len, newOrient);
      g.drawQuarterEllipse(pxi, pyi+1, major2Len, -minor1Len, newOrient);
      g.drawQuarterEllipse(pxi, pyi+1, -major1Len, minor2Len, newOrient);
      break;
    }

    default:
      std::cout << "Don't know how to draw " << *it << " into camera buffer." << std::endl;
    }
  }
}

void RawCam::closeServer() {
  if (wireless->isConnected(visRaw->sock))
    sendCloseConnectionPacket();
  Controller::closeGUI("RawVisionGUI");

  // this could be considered a bug in our wireless - if we don't setDaemon(...,false)
  // it will try to listen again even though we explicitly closed the server socket...
  wireless->setDaemon(visRaw,false);
  wireless->close(visRaw->sock);
}

void RawCam::setupServer() {
  std::vector<std::string> args;
  args.push_back("raw");
  char port[50];
  snprintf(port,50,"%d",*config->vision.rawcam.port);
  args.push_back(port);
  if (config->vision.rawcam.transport==0) {
    max_buf=UDP_WIRELESS_BUFFER_SIZE;
    visRaw=wireless->socket(Socket::SOCK_DGRAM, 1024, max_buf);
    args.push_back("udp");
  } else if (config->vision.rawcam.transport==1) {
    max_buf=TCP_WIRELESS_BUFFER_SIZE;
    visRaw=wireless->socket(Socket::SOCK_STREAM, 1024, max_buf);
    args.push_back("tcp");
  } else {
    serr->printf("ERROR: Invalid Config::vision.rawcam.transport: %d\n",*config->vision.rawcam.transport);
    return;
  }
  wireless->setDaemon(visRaw,true);
  wireless->setReceiver(visRaw,networkCallback);
  wireless->listen(visRaw,config->vision.rawcam.port);

  Controller::loadGUI("org.tekkotsu.mon.VisionGUI","RawVisionGUI",*config->vision.rawcam.port,args);
}

bool RawCam::openPacket(FilterBankGenerator& fbkgen, unsigned int time, unsigned int layer) {
  if (packet!=NULL)
    return false;

  avail=max_buf-1; //not sure why -1, but Alok had it, so i will too
  ASSERT(cur==NULL,"cur non-NULL");
  cur=NULL;
  char * buf=packet=(char*)visRaw->getWriteBuffer(avail);
  ASSERT(packet!=NULL,"dropped frame, network bandwidth is saturated (reduce frame rate or size)");
  if (packet==NULL)
    return false;

  if (!LoadSave::encodeInc("TekkotsuImage",buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
  if (!LoadSave::encodeInc(*config->vision.rawcam.encoding,buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
  if (!LoadSave::encodeInc(*config->vision.rawcam.compression,buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
  if (!LoadSave::encodeInc(fbkgen.getWidth(layer),buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
  if (!LoadSave::encodeInc(fbkgen.getHeight(layer),buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
  if (!LoadSave::encodeInc(time,buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
  if (!LoadSave::encodeInc(fbkgen.getFrameNumber(),buf,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;

  cur=buf;
  return true;
}

bool RawCam::writeColor(const FilterBankEvent& e) {
  FilterBankGenerator& fbkgen=*e.getSource();

  unsigned int y_layer=fbkgen.getNumLayers()-1-config->vision.rawcam.y_skip;
  unsigned int uv_layer=fbkgen.getNumLayers()-1-config->vision.rawcam.uv_skip;

  if (config->vision.rawcam.channel==-1) {
    if ((e.getGeneratorID()==EventBase::visRawCameraEGID && config->vision.rawcam.compression==Config::vision_config::RawCamConfig::COMPRESS_NONE)
       || (e.getGeneratorID()==EventBase::visJPEGEGID && config->vision.rawcam.compression==Config::vision_config::RawCamConfig::COMPRESS_JPEG)) {
      if (const JPEGGenerator* jgen=dynamic_cast<const JPEGGenerator*>(&fbkgen))
        if (jgen->getCurrentSourceFormat()==JPEGGenerator::SRC_COLOR)
          return true;
      openPacket(fbkgen,e.getTimeStamp(),uv_layer);
      if (cur==NULL) //error should have been displayed by openPacket
        return false;

      if (!LoadSave::encodeInc("FbkImage",cur,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
      if (!LoadSave::encodeInc(0,cur,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
      if (!LoadSave::encodeInc(0,cur,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
      if (!LoadSave::encodeInc(-1,cur,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
      if (!LoadSave::encodeInc(RawCameraGenerator::CHAN_Y,cur,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;
      if (!LoadSave::encodeInc("blank",cur,avail,"ran out of space %s:%un",__FILE__,__LINE__)) return false;;

      fbkgen.selectSaveImage(uv_layer,RawCameraGenerator::CHAN_U);
      if (!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.rawcam.transport to TCP and reopen raw cam")) return false;

      fbkgen.selectSaveImage(uv_layer,RawCameraGenerator::CHAN_V);
      if (!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.rawcam.transport to TCP and reopen raw cam")) return false;

      closePacket();
    }
    return true;
  }

  unsigned int big_layer=y_layer;
  unsigned int small_layer=uv_layer;
  if (y_layer<uv_layer) {
    big_layer=uv_layer;
    small_layer=y_layer;
  }
  if (const JPEGGenerator* jgen=dynamic_cast<const JPEGGenerator*>(&fbkgen)) {
    if (config->vision.rawcam.compression!=Config::vision_config::RawCamConfig::COMPRESS_JPEG)
      return true;
    if (jgen->getCurrentSourceFormat()==JPEGGenerator::SRC_COLOR && big_layer-small_layer<2) {
      openPacket(fbkgen,e.getTimeStamp(),big_layer);
      if (cur==NULL) //error should have been displayed by openPacket
        return false;

      fbkgen.selectSaveImage(big_layer,0);
      if (!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.rawcam.transport to TCP and reopen raw cam")) return false;

      closePacket();
    } else if (jgen->getCurrentSourceFormat()==JPEGGenerator::SRC_GRAYSCALE && big_layer-small_layer>=2) {
      bool opened=openPacket(fbkgen,e.getTimeStamp(),big_layer);
      if (cur==NULL) //error should have been displayed by openPacket
        return false;

      if (big_layer==y_layer) {
        fbkgen.selectSaveImage(y_layer,RawCameraGenerator::CHAN_Y);
        if (!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.rawcam.transport to TCP and reopen raw cam")) return false;
      } else {
        fbkgen.selectSaveImage(uv_layer,RawCameraGenerator::CHAN_U);
        if (!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.rawcam.transport to TCP and reopen raw cam")) return false;
        fbkgen.selectSaveImage(uv_layer,RawCameraGenerator::CHAN_V);
        if (!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.rawcam.transport to TCP and reopen raw cam")) return false;
      }

      if (!opened)
        closePacket();
    }
  } else {
    bool opened=false;

    if (config->vision.rawcam.compression==Config::vision_config::RawCamConfig::COMPRESS_NONE || (big_layer-small_layer>=2 && big_layer==uv_layer)) {
      opened=openPacket(fbkgen,e.getTimeStamp(),big_layer);
      if (cur==NULL) //error should have been displayed by openPacket
        return false;
      fbkgen.selectSaveImage(y_layer,RawCameraGenerator::CHAN_Y);
      if (!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.rawcam.transport to TCP and reopen raw cam")) return false;
    }

    if (config->vision.rawcam.compression==Config::vision_config::RawCamConfig::COMPRESS_NONE || (big_layer-small_layer>=2 && big_layer==y_layer)) {
      opened=openPacket(fbkgen,e.getTimeStamp(),big_layer);
      if (cur==NULL) //error should have been displayed by openPacket
        return false;
      fbkgen.selectSaveImage(uv_layer,RawCameraGenerator::CHAN_U);
      if (!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.rawcam.transport to TCP and reopen raw cam")) return false;
      fbkgen.selectSaveImage(uv_layer,RawCameraGenerator::CHAN_V);
      if (!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.rawcam.transport to TCP and reopen raw cam")) return false;
    }

    if (config->vision.rawcam.compression==Config::vision_config::RawCamConfig::COMPRESS_NONE || !opened)
      closePacket();
  }

  return true;
}

bool RawCam::writeSingleChannel(const FilterBankEvent& e) {
  FilterBankGenerator& fbkgen=*e.getSource();
  if ( (config->vision.rawcam.compression==Config::vision_config::RawCamConfig::COMPRESS_NONE &&
       e.getGeneratorID()==EventBase::visRawCameraEGID) ||
      (config->vision.rawcam.compression==Config::vision_config::RawCamConfig::COMPRESS_JPEG &&
          e.getGeneratorID()==EventBase::visJPEGEGID) )
    {
      if (const JPEGGenerator * jgen=dynamic_cast<const JPEGGenerator*>(&fbkgen))
        if (jgen->getCurrentSourceFormat()!=JPEGGenerator::SRC_GRAYSCALE)
          return true;
      unsigned int layer=fbkgen.getNumLayers()-1-config->vision.rawcam.y_skip;

      openPacket(fbkgen,e.getTimeStamp(),layer);
      if (cur==NULL) //error should have been displayed by openPacket
        return false;

      fbkgen.selectSaveImage(layer,config->vision.rawcam.channel);
      if (!LoadSave::checkInc(fbkgen.saveBuffer(cur,avail),cur,avail,"image size too large -- may need to set Config::vision.rawcam.transport to TCP and reopen raw cam")) return false;

      closePacket();
    }
  return true;
}

void RawCam::closePacket() {
  if (packet==NULL)
    return;
  visRaw->write(cur-packet);
  packet=cur=NULL;
  avail=0;
  lastProcessedTime = get_time();
}

bool RawCam::sendCloseConnectionPacket() {
  char msg[]="CloseConnection";
  unsigned int len=strlen(msg)+LoadSave::stringpad;
  char * buf = (char*)visRaw->getWriteBuffer(len);
  if (buf==NULL) {
    std::cerr << "Could not get buffer for closing packet" << std::endl;
    return false;
  }
  unsigned int used=LoadSave::encode(msg,buf,len);
  if (used==0)
    std::cerr << "Could not write close packet" << std::endl;
  visRaw->write(used);
  return true;
}

#endif

/*! @file
 * @brief Implements RawCam, which forwards images from camera over wireless
 * @author ejt (Creator)
 */

