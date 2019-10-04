//-*-c++-*-
#ifndef _BICOLORMARKERDATA_H_
#define _BICOLORMARKERDATA_H_

#include <vector>

#include "DualCoding/ShapeRoot.h"

namespace DualCoding {

class MapBuilderRequest;

//! Marker shapes, described by a single point in space
class BiColorMarkerData : public MarkerData {
public:
  //! the color on the top of the marker
  rgb topColor;
  //! the color on the bottom of the marker
  rgb bottomColor;

public:
  //! Constructor
  BiColorMarkerData(ShapeSpace& space, const Point& center, const rgb topColor, const rgb bottomColor);
    
  DATASTUFF_H(BiColorMarkerData);

  //! Print information about this shape.
  virtual void printParams() const;
    
  virtual bool isMatchingMarker(const Shape<MarkerData>& other) const;

  //! Return the string description for this marker (used to display marker specific information)
  virtual std::string getMarkerDescription() const;
  
  //! Extract bicolor markers from @a sketch
  static std::vector<Shape<MarkerData> > extractMarkers(const Sketch<uchar> &sketch, const MapBuilderRequest &req);
  
public:
  //! type of bicolor markers
  static const MarkerType_t biColorMarkerType;
  
};

template<> class Shape<BiColorMarkerData> : public ShapeRoot {
public:
  SHAPESTUFF_H(BiColorMarkerData);
};

} // namespace

#endif // MARKERDATA_H_
