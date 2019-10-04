//-*-c++-*-
#ifndef _TAPIA3MARKERDATA_H_
#define _TAPIA3MARKERDATA_H_

#include <vector>

#include "DualCoding/DualCoding.h"
#include "DualCoding/ShapeRoot.h"

namespace DualCoding {

class MapBuilderRequest;

//! Marker shapes, described by a single point in space
class Tapia3MarkerData : public MarkerData {
public:
  //! the color on the top of the marker
  rgb topColor;
  //! the color at the middle of the marker
  rgb middleColor;
  //! the color on the bottom of the marker
  rgb bottomColor;

public:
  //! Constructor
  Tapia3MarkerData(ShapeSpace& space, Point& center, const rgb topColor, const rgb middleColor, const rgb bottomColor);
    
  DATASTUFF_H(Tapia3MarkerData);

  //! Print information about this shape.
  virtual void printParams() const;
    
  virtual bool isMatchingMarker(const Shape<MarkerData>& other) const;

  //! Return the string description for this marker (used to display marker specific information)
  virtual std::string getMarkerDescription() const;
  
  //! Extract markers of @a type from @a sketch
  static std::vector<Shape<MarkerData> > extractMarkers(const Sketch<uchar> &sketch, const MapBuilderRequest &req);
  
public:
  //! type of tapia3 markers
  static const MarkerType_t tapia3MarkerType;
  
};

template<> class Shape<Tapia3MarkerData> : public ShapeRoot {
public:
  SHAPESTUFF_H(Tapia3MarkerData);
};

} // namespace

#endif // MARKERDATA_H_
