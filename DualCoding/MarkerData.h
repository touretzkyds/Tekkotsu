//-*-c++-*-
#ifndef _MARKERDATA_H_
#define _MARKERDATA_H_

#include <vector>
#include <set>
#include <map>
#include <iostream>
#include <string>

#include "Shared/fmatSpatial.h"

#include "BaseData.h"      // superclass
#include "Point.h"         // Point data member
#include "SketchTypes.h"   // uchar

namespace DualCoding {

  class ShapeRoot;
  template<typename T> class Sketch;

  class MarkerData;
  class MapBuilderRequest;

  typedef std::string MarkerType_t;
  typedef std::vector<Shape<MarkerData> > (*MarkerExtractFn_t)(const Sketch<uchar>&, const MapBuilderRequest&);

  //! Marker shapes, described by a single point in space
  class MarkerData : public BaseData {
  public:
    //! center (centroid) of marker in coordinate frame.  In image frame this is abused to store (pixel_x,pixel_y,distance).
    Point center;
    //! The type of marker this is
    MarkerType_t typeOfMarker;

    static const MarkerType_t unknownMarkerType;

  public:
    //! Constructor
    MarkerData(ShapeSpace& _space, const Point& center, const rgb rgbvalue=rgb());

    static ShapeType_t getStaticType() { return markerDataType; }
    
    MarkerType_t getMarkerType() const { return typeOfMarker; }

    DATASTUFF_H(MarkerData);
    
    friend class Shape<MarkerData>;
    
    //! return the centroid of the shape in point format
    virtual Point getCentroid() const;
    
    //! Print information about this shape.
    virtual void printParams() const;
    
    //! Transformations. (Virtual in BaseData.)
    virtual void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);
    
    //! Project to ground
    virtual void projectToGround(const fmat::Transform& camToBase,
				 const PlaneEquation& groundplane);
    
    //! Update derived properties
    virtual void update_derived_properties();
    
    //! Match blobs based on their parameters.  (Virtual in BaseData.)
    virtual bool isMatchFor(const ShapeRoot& other) const;
    
    //! Update parameters given matching shape (for merging in mapping)
    virtual bool updateParams(const ShapeRoot& other, bool forceUpdate=false);
    
    //! Test for marker with matching features.
    /*!  Unlike isMatchFor, does not consider positions, should just compare things like colors or
         other marker-specific features.
      */
    virtual bool isMatchingMarker(const Shape<MarkerData>& other) const;
	  
    virtual bool localizeByCamera() const {return true;}

    virtual unsigned short getDimension() const { return 3; }

    //! Return the string description for this marker (used to display marker specific information)
    virtual std::string getMarkerDescription() const;

    //! Get the name for a given marker type
    static std::string getMarkerTypeName(MarkerType_t type);

    // Marker dispatch stuff
    static MarkerType_t registerMarkerType(std::string markerTypeName, MarkerExtractFn_t extractor);

    //! Extract markers of @a type from @a sketch
    static std::vector<Shape<MarkerData> > extractMarkers(const Sketch<uchar> &sketch, MarkerType_t type, const MapBuilderRequest &req);

    //! Extract markers of all registered types from @a sketch
    static std::vector<Shape<MarkerData> > extractMarkers(const Sketch<uchar> &sketch, const MapBuilderRequest &req);

    //! Set the z-coordinate (distance from the camera) for a pixel, assuming we know the object's height above the ground plane
    static void calculateCameraDistance(Point &p, const float height);

    //! Return set of all known marker types; used for general-purpose localization code
    static const std::set<MarkerType_t> allMarkerTypes();

  private:
    //! Map for marker extraction dispatch  
    static std::map<MarkerType_t, MarkerExtractFn_t>& getExtractorMap();

    //! Render into a sketch space and return reference. (Private.)
    virtual Sketch<bool>* render() const;
    
    MarkerData& operator=(const MarkerData&); //!< don't call
  };

} // namespace

#endif // MARKERDATA_H_
