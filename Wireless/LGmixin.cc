#include "DualCoding/Sketch.h"
#include "Shared/ProjectInterface.h"
#include "Shared/ImageUtil.h"
#include "Vision/JPEGGenerator.h"

#include "LGmixin.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <unistd.h>

using namespace std;
using namespace DualCoding;

unsigned int LGmixin::instanceCount = 0;
Socket* LGmixin::LGsock = NULL;
LGmixin* LGmixin::theOne = NULL;

LGmixin::LGmixin() {
  if ( instanceCount++ > 0 )
    return;
  if (theOne != NULL) {
      cerr << "LGmixin statics already constructed!?!?!" << endl;
      return;
    }
    theOne=this;

    LGsock = wireless->socket(Socket::SOCK_STREAM, 1024, LGbufferSize);
    wireless->setDaemon(LGsock, false);
    wireless->listen(LGsock, LGport);
}

LGmixin::~LGmixin() {
  if ( --instanceCount > 0 )
    return;
  if (theOne == NULL) {
    cerr << "LGmixin statics already destructed!?!?!" << endl;
    return;
  }
  wireless->close(LGsock->sock);
  theOne = NULL;
}

void LGmixin::uploadFile(const std::string &filename, bool display, bool isImage) {
  if ( !wireless->isConnected(LGsock->sock) ) {
    cerr << "LookingGlass not connected." << endl;
    return;
  }

  int in_file = open(filename.c_str(), O_RDONLY);
  if ( in_file < 0 ) {
    cerr << "Error: Unable to open file\n";
    return;
  }
  struct stat s;
  stat(filename.c_str(),&s);
  const std::string remoteFilename = filename.substr(filename.rfind('/')+1);
  LGsock->printf("UPLOAD_BINARY %s %u\n", remoteFilename.c_str(), (unsigned int)s.st_size);
  
	size_t remain = static_cast<size_t>(s.st_size);
  while(remain>0){
    char *buffer = (char*)LGsock->getWriteBuffer(remain);
    if ( buffer==NULL ) {
      cerr << "NULL buffer in LG uploadFile: file too big?" << endl;
      break;
    }
    int read_size = read(in_file, buffer, remain);
    if ( read_size < 0 ){
      cerr << "Error: Read error " << read_size << endl;
			read_size=0; // fall through
    }
		LGsock->write(read_size); // *must* always balance getWriteBuffer() with write()
		remain = (read_size==0) ? 0 : remain-read_size;
  }
  
  if(display) {
    if ( isImage )
      displayImageFile(remoteFilename);
    else
      displayHtmlFile(remoteFilename);
  }

  close(in_file);
}

void LGmixin::displayHtmlFile(const std::string &remoteFilename) {
  LGsock->printf("DISPLAY LookingGlass_Dummy_File\n"); // work-around because renderer can't display same file twice
  LGsock->printf("DISPLAY %s\n",remoteFilename.c_str());
}

void LGmixin::displayImageFile(const std::string &remoteFilename) {
  LGsock->printf((string("UPLOAD_HTML %s.html\n") +
		 string("<html><body><table width=100%% height=100%%><tr><td align=center valign=middle>") +
		  string("<img src=\"%s\"></td></tr></table></body>\n</html>\n")).c_str(),
		 remoteFilename.c_str(), remoteFilename.c_str());
  displayHtmlFile(remoteFilename+".html");
}

void LGmixin::displayHtmlText(const std::string &text) {
  unsigned int const msgSize = text.size();
  string const tempfilename = "temp9999";
  LGsock->printf("UPLOAD_BINARY %s %u\n", tempfilename.c_str(), msgSize);
  char *buffer = (char*)LGsock->getWriteBuffer(msgSize);
	if ( buffer==NULL ) {
		cerr << "NULL buffer in LG displayHtmlText: file too big?" << endl;
		return;
	}
  memcpy(buffer,text.c_str(),msgSize);
  LGsock->write(msgSize);
  displayHtmlFile(tempfilename);
}

void LGmixin::sendCommand(const std::string &command) {
  LGsock->printf("%s\n", command.c_str());
}

void LGmixin::uploadCameraImage(const std::string &remoteFilename) {
    JPEGGenerator *jpeg = ProjectInterface::defColorJPEGGenerator;  
    // call to getImage must come before call to getImageSize
    char *image = (char*)jpeg->getImage(ProjectInterface::fullLayer, 0);
    int const imgSize = jpeg->getImageSize(ProjectInterface::fullLayer, 0);
    LGsock->printf("UPLOAD_BINARY %s %u\n", remoteFilename.c_str(), imgSize);
    char *buffer = (char*)LGsock->getWriteBuffer(imgSize);
    if ( buffer==NULL ) {
      cerr << "NULL buffer in LG uploadCameraImage: file too big?" << endl;
      return;
    }
    memcpy(buffer,image,imgSize);
    LGsock->write(imgSize);
}

void LGmixin::uploadSketch(const DualCoding::Sketch<DualCoding::uchar> &sketch,
			   const std::string &remoteFilename) {
  // get the color table into a format suitable for fast indexing
  int const numColors = ProjectInterface::getNumColors();
  uchar Rbyte[numColors], Gbyte[numColors], Bbyte[numColors];
  for ( int i=0; i<numColors; i++ ) {
    rgb const c = ProjectInterface::getColorRGB(i);
    Rbyte[i] = c.red;
    Gbyte[i] = c.green;
    Bbyte[i] = c.blue;
  }
  // translate indexed color image into RGB image
  size_t const numPixels = sketch->getNumPixels();
  size_t const pixelsize = 3;
  size_t const inbuffsize = numPixels * pixelsize;
  char* inbuff = new char[inbuffsize];
  char* inbuffptr = inbuff;
  for ( size_t i=0; i<numPixels; i++ ) {
    uchar const p = sketch[i];
    *inbuffptr++ = Rbyte[p];
    *inbuffptr++ = Gbyte[p];
    *inbuffptr++ = Bbyte[p];
  }
  char *outbuff = NULL;
  size_t outbuffsize = 0;
  size_t const imgSize =
    image_util::encodeJPEG(inbuff, inbuffsize, sketch->getWidth(), sketch->getHeight(), pixelsize, 
			   outbuff, outbuffsize, pixelsize, 70);
  LGsock->printf("UPLOAD_BINARY %s %lu\n", remoteFilename.c_str(), (unsigned long)imgSize);
  char *lgbuff = (char*)LGsock->getWriteBuffer(imgSize);
  if ( lgbuff==NULL ) {
    cerr << "NULL buffer in LG uploadSketch" << endl;
    return;
  }
  memcpy(lgbuff,outbuff,imgSize);
  LGsock->write(imgSize);
  delete [] outbuff;
  cout << "LG:  outbuffsize = " << outbuffsize << ",  imgSize = " << imgSize << endl;
  delete[] inbuff;
}
