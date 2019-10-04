#ifndef __PGMIMG_H
#define __PGMIMG_H

#include <vector>
#include <fstream>
#include <string>

class keypoint;
class keypointPair;

class PGMImg{
private:
  int   width;
  int   height;
  int   base;
  unsigned char* pixels;
  std::string filename;
		
  void setPixel(int x, int y, unsigned char color);
	
public:
  int   id;
  PGMImg();
  PGMImg(PGMImg* img1, PGMImg* img2, std::vector<keypointPair*>* matches);
		
  ~PGMImg();
		
  int getHeight();
  int getWidth();
		
  void fromFile(std::string imageFilename);
  void toFile(std::string imageFilename);
		
  unsigned char getPixel(int x, int y);
		
  void drawLine(int x1, int y1, int x2, int y2, unsigned char color);
  void drawKeypoint(keypoint* key);
  void drawKeypoints(std::vector<keypoint*> keys);
		
  // Transformations
  void translate(int x, int y, unsigned char color); // increases size if necessary
  void rotate(double s, unsigned char color);
  void doubleSize(double s, unsigned char color);    // doubles the image s times
  void halfSize(double s, unsigned char color);      // halves the image s times
  void cropImage(int startx, int starty, int endx, int endy, unsigned char color);

private:
  PGMImg(const PGMImg&);
  PGMImg& operator=(const PGMImg&);
};

#endif
