#include "Vision/colors.h"
#include <cstdio>

//! displays an rgb value in the form '[r,g,b]'
std::ostream& operator<<(std::ostream &os, const rgb &rgbval) {
  os << "[" << (unsigned int)rgbval.red
     << "," << (unsigned int)rgbval.green
     << "," << (unsigned int)rgbval.blue
     << "]";
  return os;
}

//! returns @a rgbval in the string form 'r g b'
std::string toString(const rgb &rgbval) {
  char buff[15];
  snprintf(buff,15,"%d %d %d",rgbval.red,rgbval.green,rgbval.blue);
  return buff;
}


//! displays a yuv value in the form '[y,u,v]'
std::ostream& operator<<(std::ostream &os, const yuv &yuvval) {
  os << "yuv[" << (unsigned int)yuvval.y
     << "," << (unsigned int)yuvval.u
     << "," << (unsigned int)yuvval.v
     << "]";
  return os;
}

//! returns @a yuvval in the string form 'y u v'
std::string toString(const yuv &yuvval) {
  char buff[15];
  snprintf(buff,15,"%d %d %d",yuvval.y,yuvval.u,yuvval.v);
  return buff;
}

//! converts rgb to yuv
yuv rgb2yuv(const rgb x) {
  int yval = int((8.0 * x.red / 27.0) + (16.0 * x.green / 27.0) + (x.blue / 9.0));
  int uval = int(128.0 - (4.0 * x.red / 27.0) - (8.0 * x.green / 27.0) + (4.0 * x.blue / 9.0));
  int vval = int((19.0 * x.red / 27.0) - (16.0 * x.green / 27.0) - (x.blue / 9.0) + 128.0);

  if (yval > 255) yval = 255;
  if (yval < 0) yval = 0;
  else if (uval > 255) uval = 255;
  if (vval < 0) vval = 0;
  else if (vval > 255) vval = 255;

  return yuv(yval, uval, vval);
}
