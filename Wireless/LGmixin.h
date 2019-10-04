//-*-c++-*-
#ifndef INCLUDED_LGmixin_h_
#define INCLUDED_LGmixin_h_

#include "Wireless/Wireless.h"
#include "DualCoding/Sketch.h"

//! Mix-in for the BehaviorBase or StateNode class to give access to Looking Glass variables.
class LGmixin {
 protected:
  static unsigned int instanceCount; //!< count of LGmixin instances -- when this hits zero, close the socket
  static Socket *LGsock; //!< socket to talk to Looking Glass server

 public:
  //! Constructor
  LGmixin();

  //! Destructor
  virtual ~LGmixin();

  //! Upload a file from the AIBO to the Looking Glass client
  static void uploadFile(const std::string &filename, bool display=false, bool isImage=false);

  //! Display an HTML file on the Looking Glass
  static void displayHtmlFile(const std::string &remoteFilename);

  //! Display a single image file on the Looking Glass (creates a dummy HTML file)
  static void displayImageFile(const std::string &remoteFilename);

  //! Display HTML string on the Looking Glass (creates a dummy HTML file)
  static void displayHtmlText(const std::string &text);

  //! Upload current camera image to the Looking Glass client as a JPEG file
  static void uploadCameraImage(const std::string &remoteFileName);

  //! For debugging: send an arbitrary command string to the Looking Glass client
  static void sendCommand(const std::string &command);

  //! Upload a sketch as a JPEG file to the Looking Glass client
  static void uploadSketch(const DualCoding::Sketch<DualCoding::uchar> &sketch, const std::string &remoteFilename);

 private:
  //! used so static member functions can access non-static members
  static LGmixin* theOne;

  LGmixin (const LGmixin&);	 //!< never call this
  LGmixin& operator=(const LGmixin&); //!< never call this

  static const unsigned int LGbufferSize = 80000; //!< maximum send buffer size
  static const unsigned int LGport = 10100; //!< port number to listen on

};

#endif
