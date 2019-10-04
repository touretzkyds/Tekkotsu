//-*-c++-*-

#include <iostream>
#include <string>
#include <math.h>  // for min

using namespace std;

#include "ViewerConnection.h"

namespace DualCoding {

void ViewerConnection::writeBigString(std::string const &msg) {
  size_t const len = msg.size();
  for (size_t startpos = 0; startpos<len; ) {
    size_t const chunksize = min(len-startpos, (size_t)(DIALOG_BUFFER_SIZE-1));
    string const chunk = msg.substr(startpos,chunksize);
    dialogSock->printf("%s",chunk.c_str());
    startpos += chunksize;
  }
}


} // namespace
