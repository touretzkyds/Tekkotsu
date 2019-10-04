//-*-c++-*-
#ifndef INCLUDED_ViewerConnection_h_
#define INCLUDED_ViewerConnection_h_

#include <string>

#include "Shared/ProjectInterface.h"
#include "Wireless/Wireless.h"

namespace DualCoding {

static const unsigned int DIALOG_BUFFER_SIZE=20000;

#if defined(PLATFORM_APERIOS)
static const unsigned int SKETCH_BUFFER_SIZE=70000; // Raised to 120,000 for sketch<usint> in world space; then lowered to 70000 due to memory shortage
#else
static const unsigned int SKETCH_BUFFER_SIZE=(unsigned int)(3.7*(1<<20)); // Raised to 1.5MB for higher resolution (1280x720) uint images
#endif

static const unsigned int CAM_LAYER=ProjectInterface::fullLayer; //!< the full resolution layer in the filter bank
static const unsigned int CAM_CHANNEL=0;  //!< corresponds to appropriate thresholding listed in tekkotsu.xml
static const unsigned int WORLD_WIDTH=225;
static const unsigned int WORLD_HEIGHT=225;

static const unsigned short CAM_DIALOG_PORT   = 5800;
static const unsigned short CAM_SKETCH_PORT      = 5801;
static const unsigned short LOCAL_DIALOG_PORT = 5802;
static const unsigned short LOCAL_SKETCH_PORT    = 5803;
static const unsigned short WORLD_DIALOG_PORT = 5804;
static const unsigned short WORLD_SKETCH_PORT    = 5805;

class ViewerConnection {

 private:
  Socket *dialogSock, *sketchSock;
  int dialogPort, sketchPort;

 public:
  ViewerConnection(void) : dialogSock(NULL), sketchSock(NULL), dialogPort(0), sketchPort(0) {}

  Socket *getDialogSocket() const { return dialogSock; }
  Socket *getSketchSocket() const { return sketchSock; }

  void setDialogSocket(Socket* sock, int const port) { dialogSock = sock; dialogPort = port; }
  void setSketchSocket(Socket* sock, int const port) { sketchSock = sock; sketchPort = port; }

  void writeBigString(std::string const &msg);

 private:
  ViewerConnection(const ViewerConnection&); //!< never call this
  ViewerConnection& operator=(const ViewerConnection&); //!< never call this

};

} // namespace

#endif
