//-*-c++-*-
#include <cmath>
#include <vector>
#include <list>

#include "Shared/Measures.h"

#include "SketchSpace.h"
#include "Sketch.h"
#include "Region.h"
#include "visops.h"

namespace DualCoding {

// be careful about changing this value, b/c memset writes bytes
#define NOTCOMPUTED 0

Region::Region(const SketchSpace& _space) : 
  SketchIndices(), space(_space),
  topBound(NOTCOMPUTED), botBound(NOTCOMPUTED), 
  leftBound(NOTCOMPUTED), rightBound(NOTCOMPUTED),
  area(NOTCOMPUTED)
{ 
  memset(&moments, NOTCOMPUTED, sizeof(float)*(MAX_MOMENT+1)*(MAX_MOMENT+1));
  memset(&cmoments, NOTCOMPUTED, sizeof(float)*(MAX_MOMENT+1)*(MAX_MOMENT+1));
}

void Region::recomputeProperties() {
  topBound = NOTCOMPUTED;
  botBound = NOTCOMPUTED; 
  leftBound = NOTCOMPUTED;
  rightBound = NOTCOMPUTED;
  area = NOTCOMPUTED;
  memset(&moments, NOTCOMPUTED, sizeof(float)*(MAX_MOMENT+1)*(MAX_MOMENT+1));
  memset(&cmoments, NOTCOMPUTED, sizeof(float)*(MAX_MOMENT+1)*(MAX_MOMENT+1));
}

std::list<Region> Region::extractRegions(const Sketch<uint>& labels, uint area_thresh)
{
  labels.checkValid();
  const uint* labeldata = labels->getRawPixels();
  size_t length = labels->getNumPixels();
  
  // tally up areas
  std::vector<uint> areas(labels->max()+1, 0); // not having +1 may have caused malloc crash
  for (size_t i = 0; i < length; i++)
    if (labeldata[i] != 0)
      areas[labeldata[i]]++;	
  
  // add unsorted Regions to list
  std::list<Region> regionlist;
  for (uint r = 0; r < (uint)(areas.size()); r++) {
    if (areas[r] >= area_thresh) {
      // construct Region and add to list
      Region cur_reg(labels->getSpace());
      for (size_t i = 0; i < length; i++)
	if (labeldata[i] == r)
	  cur_reg.table.insert(i);
      cur_reg.area = areas[r]; // go ahead and pre-set area
      regionlist.push_back(cur_reg); // actually calls copy constructor
    }
  }
  
  regionlist.sort();
  return regionlist;
}


Region Region::extractRegion(const Sketch<bool>& sketch)
{
  size_t length = sketch->getNumPixels();

  Region cur_reg(sketch->getSpace());

  int area = 0;

  for (size_t i = 0; i< length; i++) {
    if (sketch[i]) {
      area++;
      cur_reg.table.insert(i);
    }
  }

  cur_reg.area = area;

  return cur_reg;
}

bool Region::operator< (const Region& other) const { return (area > other.area); }

int Region::findTopBound() {
  if (topBound != NOTCOMPUTED)  // problem if NOTCOMPUTED == 0
    return topBound;
  else {
    SketchIndices::CI it;
    int top = space.getHeight();
    int width = space.getWidth();
    for (it = table.begin(); it != table.end(); it++)
      if (int((*it)/width) < top)
				top = (*it)/width;
    topBound = top;
    return top;
  }
}

int Region::findBotBound() {
  if (botBound != NOTCOMPUTED)  // problem if NOTCOMPUTED == 0
    return botBound;	
  else {
    SketchIndices::CI it;
    int bot = -1;
    int width = space.getWidth();
    for (it = table.begin(); it != table.end(); it++)
      if (int((*it+1)/width) > bot)
				bot = (*it)/width;
    botBound = bot;
    return bot;
  }
}

int Region::findLeftBound() {
  if (leftBound != NOTCOMPUTED)  // problem if NOTCOMPUTED == 0
    return leftBound;	
  else {
    SketchIndices::CI it;
    int left = 9999;
    int width = space.getWidth();
    for (it = table.begin(); it != table.end(); it++)
      if (int((*it)%width) < left)
				left = (*it)%width;
    leftBound = left;
    return left;
  }
}

int Region::findRightBound() {
  if (rightBound != NOTCOMPUTED)  // problem if NOTCOMPUTED == 0
    return rightBound;	
  else {
    SketchIndices::CI it;
    int right = -1;
    int width = space.getWidth();
    for (it = table.begin(); it != table.end(); it++)
      if (int((*it)%width) > right)
				right = (*it)%width;
    rightBound = right;
    return right;
  }
}

// returns x coordinate of first match Point for given y_coord
int Region::findXcoordFor(const coordinate_t y_coord) {
  const int width = space.getWidth();
  //  const int width = space.getWidth()+1;
  for (SketchIndices::CI it = table.begin(); 
       it != table.end(); it++) 
    if ((*it)/width == y_coord) 
      return (*it)%width;
  //return (*it)%(width-1);
  return -1;
}

// returns y coordinate of first match Point for given x_coord
int Region::findYcoordFor(const coordinate_t x_coord) {
  const int width = space.getWidth();
  for (SketchIndices::CI it = table.begin(); 
       it != table.end(); it++)
    if ((*it)%width == x_coord)
      return (*it)/width;
  return -1;
}

Point Region::findTopBoundPoint() {
  const coordinate_t y_coord = findTopBound();
  return Point(findXcoordFor(y_coord),y_coord);
}
Point Region::findBotBoundPoint() {
  const coordinate_t y_coord = findBotBound();
  return Point(findXcoordFor(y_coord),y_coord);
}
Point Region::findLeftBoundPoint() {
  const coordinate_t x_coord = findLeftBound();
  return Point(x_coord, findYcoordFor(x_coord));
}
Point Region::findRightBoundPoint() {
  const coordinate_t x_coord = findRightBound();
  return Point(x_coord, findYcoordFor(x_coord));
}

bool Region::isContained(const Point& pt, const uint max_dist) {
  const uint x_coord = (uint) pt.coordX();
  const uint y_coord = (uint) pt.coordY();
  const uint width = space.getWidth();
  //  cout << pt << endl;
  for (SketchIndices::CI it = table.begin(); 
       it != table.end(); it++)
    if (((*it)%width <= x_coord+max_dist && (*it)%width >= x_coord-max_dist)
				&& ((*it)/width <= y_coord+max_dist && (*it)/width >= y_coord-max_dist))
      return true;
  return false;
}

Point Region::mostDistantPtFrom(const LineData& ln) {
  float max_dist = 0;
  Point most_dist_pt;
  const int width = space.getWidth();
  //  cout << pt << endl;
  for (SketchIndices::CI it = table.begin(); 
       it != table.end(); it++) {
    if (ln.perpendicularDistanceFrom(Point((*it)%width, (*it)/width)) > max_dist) {
      max_dist = ln.perpendicularDistanceFrom(Point((*it)%width, (*it)/width));
      most_dist_pt.setCoords((*it)%width, (*it)/width);
    }
  }
  return most_dist_pt;
}




// All moment-based equations taken from Prokop & Reeves 1992, "A survey of moment-based techniques for unoccluded object representation and recognition"
float Region::findMoment(size_t p, size_t q) 
{
  // should add in more efficient routines for some low-moments like area
  
  if(moments[p][q] != NOTCOMPUTED) {
    return moments[p][q];
  } else {
    // should pre-calculate powers for rows and columns, as in Flusser (1998)
    int xmin = findLeftBound(), xmax = findRightBound();
    float powp[xmax-xmin+1];
    for (int x = xmin; x <= xmax; x++) {
      if (x == 0) // if don't check for this, risk floating-point exception
	powp[x-xmin] = 1;
      else powp[x-xmin] = std::pow((float)(x), (float)p); 
    }
    int ymin = findTopBound(), ymax = findBotBound();
    float powq[ymax-ymin+1];
    for (int y = ymin; y <= ymax; y++) {
      if (y == 0)
	powq[y-ymin] = 1;
      else powq[y-ymin] = std::pow((float)(y), (float)q); 
    }
    
    float m = 0;
    int xval, yval;
    for (SketchIndices::CI it = table.begin(); it != table.end(); ++it) {
      xval = (*it) % space.getWidth();
      yval = (*it) / space.getWidth();
      m += powp[xval-xmin] * powq[yval-ymin];
    }
    moments[p][q] = m;
    return m;
  }
}

float Region::findCentralMoment(size_t p, size_t q) {
  // should add in more efficient routines for some low-moments like area
  
  if(cmoments[p][q] != NOTCOMPUTED) {
    return cmoments[p][q];
  } else {
    Point centroid = findCentroid(); //cen.first;
    const float cx = centroid.coordX();
    const float cy = centroid.coordY();
    
    // should pre-calculate powers for rows and columns, as in Flusser (1998)
    int xmin = findLeftBound(), xmax = findRightBound();
    float powp[xmax-xmin+1];
    for (int x = xmin; x <= xmax; x++) {
      if ((x-cx)==0) // if don't check for this, risk floating-point exception
        powp[x-xmin] = 1;
      else powp[x-xmin] = std::pow((float)(x-cx), (float)p); 
    }
    int ymin = findTopBound(), ymax = findBotBound();
    float powq[ymax-ymin+1];
    for (int y = ymin; y <= ymax; y++) {
      if ((y-cy)==0)
        powq[y-ymin] = 1;
      else powq[y-ymin] = std::pow((float)(y-cy), (float)q); 
    }
    
    float m = 0;
    int xval, yval;
    for (SketchIndices::CI it = table.begin(); it != table.end(); ++it) {
      xval = (*it) % space.getWidth();
      yval = (*it) / space.getWidth();
      //m += pow(xval,(float)p) * pow(yval,(float)q);
      m += powp[xval-xmin] * powq[yval-ymin];
    }
    
    cmoments[p][q] = m;
    return m;
  }
}

float Region::findNormCentralMoment(size_t p, size_t q) {
  // normalize
  // from Gonzalez & Woods (1992)
  float gamma = (p+q)/2 + 1;
  return(findCentralMoment(p,q) / std::pow(findArea(), gamma));
}

int Region::findArea() {
  if (area != NOTCOMPUTED)
    return area;
  else {
    area = table.size();
    return area;
  }
}

Point Region::findCentroid() {
  findArea();
  return Point(findMoment(1,0)/area, findMoment(0,1)/area);
  //	centroid.first = findMoment(1,0)/findMoment(0,0);
  //	centroid.second = findMoment(0,1)/findMoment(0,0);
  //	return centroid;
  
  /*	if (centroid.first != NOTCOMPUTED) {
	return centroid;
	} else {
	int xsum = 0, ysum = 0;
	typedef SketchIndices::const_iterator CI;
	for (CI i = begin(); i != end(); ++i) {
	xsum += (*i) % space.getWidth();
	ysum += (*i) / space.getWidth();
	}
	centroid.first = xsum/findArea();
	centroid.second = ysum/findArea();
	return centroid;
	}*/
}

AngPi Region::findPrincipalAxisOrientation() {
  return AngPi( 0.5f * std::atan2(2*findCentralMoment(1,1), 
			   findCentralMoment(2,0)-findCentralMoment(0,2)));
}

float Region::findSemiMajorAxisLength() {
  float u20 = findCentralMoment(2,0);
  float u02 = findCentralMoment(0,2);
  float u11 = findCentralMoment(1,1);
  float u00 = findArea(); //  = findCentralMoment(0,0);
  return std::sqrt((2*(u20+u02+std::sqrt((u20-u02)*(u20-u02)+4*u11*u11)))/u00);
}

float Region::findSemiMinorAxisLength() {
  float u20 = findCentralMoment(2,0);
  float u02 = findCentralMoment(0,2);
  float u11 = findCentralMoment(1,1);
  // float u00 = findCentralMoment(0,0);
  float u00 = findArea();
  return std::sqrt((2*(u20+u02-std::sqrt((u20-u02)*(u20-u02)+4*u11*u11)))/u00);
}

Region& Region::operator=(const Region &other) {
  topBound = other.topBound;
  botBound = other.botBound;
  leftBound = other.leftBound;
  rightBound = other.rightBound;
  for (size_t i = 0; i <= MAX_MOMENT; i++)
    for (size_t j = 0; j <= MAX_MOMENT; j++) {
      moments[i][j] = other.moments[i][j];
      cmoments[i][j] = other.cmoments[i][j];
    }
  area = other.area;
  return *this;
}

/* FIX THIS
float Region::findRadius() {
  float u20 = findCentralMoment(2,0);
  float u02 = findCentralMoment(0,2);
  float u00 = findArea();
  return sqrt((2.0*(u20+u02))/u00);
}
*/

} // namespace
