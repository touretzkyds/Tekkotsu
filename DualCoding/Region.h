//-*-c++-*-
#ifndef INCLUDED_Region_h
#define INCLUDED_Region_h

#include <list>

#include "Shared/Measures.h"

#include "Point.h"
#include "LineData.h"
#include "SketchIndices.h"
#include "Sketch.h"

namespace DualCoding {

class SketchSpace;

#define MAX_MOMENT 5

class Region : public SketchIndices {
public:
  Region(const SketchSpace& _space); //!< initializes all properties to be NOTCOMPUTED
  ~Region() {};
  
  //! Returns a list of the different Regions in labels, sorted by area
  //@param labels Each different region in labels should have a different number assigned to it
  //@param area_thresh Minimum allowed area of a region
  static std::list<Region> extractRegions(const Sketch<uint>& labels, uint area_thresh = 10);

  //! Return a single region from a Sketch<bool>
  static Region extractRegion(const Sketch<bool>& sketch);
  
  //! Render a region as a Sketch<bool>
  Sketch<bool> getRendering() const { return SketchIndices::getRendering(const_cast<SketchSpace&>(space)); }

  //! Compares areas of two Regions; really only used to sort Regions
  //! assumes area already calculated
  //! Notes that it actually returns greater-than instead of less-than, so
  //! that lists are sorted with biggest areas first
  bool operator< (const Region& other) const;
  

  //! sets all properties to be NOTCOMPUTED
  //! called after making changes (either addition/deletion) to table
  void recomputeProperties();

  int findTopBound();
  int findBotBound();
  int findLeftBound();
  int findRightBound();

  Point findTopBoundPoint();
  Point findBotBoundPoint();
  Point findLeftBoundPoint();
  Point findRightBoundPoint();
  bool isContained(const Point&, const uint max_dist=0);

  Point mostDistantPtFrom(const LineData&);
  
  //! Calculates the two-dimensional Cartesian moment Mpq 
  float findMoment(size_t p, size_t q);
  float findCentralMoment(size_t p, size_t q);
  //! Finds the area-normalized central moment
  float findNormCentralMoment(size_t p, size_t q);
  
  int findArea();
  Point findCentroid(); //!< returns coords of center of mass
  
  //! Returns the angle of the orientation of the principal axis in radians
  AngPi findPrincipalAxisOrientation();
  
  //! Returns the length of the semi-major (x) axis of the image ellipse
  float findSemiMajorAxisLength();
  
  //! Returns the length of the semi-minor (y) axis of the image ellipse
  float findSemiMinorAxisLength();
  
  float findRadius() { return findSemiMajorAxisLength(); } //FIX THIS

private:
  const SketchSpace& space;
  
  int topBound;
  int botBound;
  int leftBound;
  int rightBound;
  
  float moments[MAX_MOMENT+1][MAX_MOMENT+1];
  float cmoments[MAX_MOMENT+1][MAX_MOMENT+1];
  
  int area;
  
  int findXcoordFor(const coordinate_t y_coord);
  int findYcoordFor(const coordinate_t x_coord);
  
public:
  const SketchSpace& getSpace() const { return space; };
  Region& operator=(const Region &other);
};

} // namespace

#endif
