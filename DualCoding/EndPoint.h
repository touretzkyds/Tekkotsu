//-*-c++-*-
#ifndef _ENDPOINT_H_
#define _ENDPOINT_H_

#include "Shared/Measures.h"
#include "Point.h"

namespace DualCoding {

class EndPoint : public Point {
private: 
  //! @a valid indicates whether or not this is known to be the final end-point of the line.
  /*! If false, this was the point at which the line went off-camera, not the true end point. */
  bool valid;

  //! @a active indicates whether or not the line is to terminate at this point.  
  /*! If false, the line will continue infinitely as a ray beyond this point. */
  bool active;

  //! @a rendering_valid is set when the line is rendered and cleared if an endpoint is modified
  mutable bool rendering_valid;

  //! How many measurements have gone into the estimate of this point.
  int nsamples;
	
public:
  friend class LineData;
  friend class BoundaryDetector;
  friend class PolygonData;
	
  EndPoint()
    : Point(), valid(true), active(true), rendering_valid(false), nsamples(1) {};

  EndPoint(coordinate_t const &xp, coordinate_t const &yp, coordinate_t const &zp=0) 
    : Point(xp,yp,zp), valid(true), active(true), rendering_valid(false), nsamples(1) {};

  EndPoint(const Point& otherPt)
    : Point(otherPt), valid(true), active(true), rendering_valid(false), nsamples(1) {};

  bool isValid() const { return valid; }
  void setValid(bool _valid) { valid = _valid; }

  bool isActive() const { return active; }
  void setActive(bool _active) { 
    if ( active != _active ) rendering_valid = false;
    active = _active;
  }

  bool isMatchFor(const EndPoint& other, float dist_thresh) const {return distanceFrom(other) <= dist_thresh; }
  bool operator==(const EndPoint& other) const { return isMatchFor(other,5); }


  //! Checks if endpoint comes to close to edge of camera frame and if so,
  //! marks it as invalid.
  void checkValidity(int width, int height, int edge_thresh);

  void updateParams(const EndPoint& other);
  void updateParams(const EndPoint& other, unsigned int num_updates);

private:
  void clearNumUpdates(void) { nsamples=0; };
  void setNumUpdates(long _nsamples) { nsamples = _nsamples; };

public:
  void incrementNumUpdates(void) { nsamples++; };
  void decrementNumUpdates(void) { nsamples--; };
  int numUpdates(void) const { return nsamples; };
};

} // namespace

#endif
