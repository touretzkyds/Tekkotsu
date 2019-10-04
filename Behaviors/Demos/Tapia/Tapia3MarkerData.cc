//-*-c++-*-
#include <vector>

#include "DualCoding/ShapeMarker.h"
#include "DualCoding/Sketch.h"

#include "DualCoding/ShapeFuns.h"
#include "DualCoding/BlobData.h"
#include "DualCoding/ShapeBlob.h"

#include "Behaviors/Demos/Tapia/Tapia3MarkerData.h"

using namespace std;

namespace DualCoding {

  const MarkerType_t Tapia3MarkerData::tapia3MarkerType =
    MarkerData::registerMarkerType("tapia3MarkerType",Tapia3MarkerData::extractMarkers);

  Tapia3MarkerData::Tapia3MarkerData(ShapeSpace& _space, Point& _center, rgb _top, rgb _mid, rgb _bottom) :
    MarkerData(_space, _center, _top), topColor(_top), middleColor(_mid), bottomColor(_bottom)
  { typeOfMarker = tapia3MarkerType; }

  DATASTUFF_CC(Tapia3MarkerData);

  void Tapia3MarkerData::printParams() const {
    cout << "Type = " << getTypeName() << "(Tapia3)" << "  ID=" << getId() << "  ParentID=" << getParentId() << endl;
    cout << "  topColor = " << topColor << endl
	 << "  middleColor = " << middleColor << endl
	 << "  bottom color = " << bottomColor << endl
	 << "  center = " << center << endl;
  }

  bool Tapia3MarkerData::isMatchingMarker(const Shape<MarkerData>& other) const {
    // make sure they are the same type
    if (!(typeOfMarker == other->typeOfMarker))
      return false;
  
    // make sure colors are the same
    const Tapia3MarkerData& other_tapia3 = (const Tapia3MarkerData &)(other.getData());
    if (!(topColor == other_tapia3.topColor && bottomColor == other_tapia3.bottomColor))
      return false;  

    return true;
  }

  string Tapia3MarkerData::getMarkerDescription() const {
    stringstream ss;
    ss << "coords=" << getCentroid();
    ss << " topColor=[" << toString(topColor)  << "] middleColor=[" << toString(middleColor)
       << "] bottomColor=[" << toString(bottomColor) << "]";
    return ss.str();
  }

  vector<Shape<MarkerData> > 
  Tapia3MarkerData::extractMarkers(const Sketch<uchar> &sketch, const MapBuilderRequest &req) {
    std::map<MarkerType_t, coordinate_t>::const_iterator it = req.markerHeights.find(tapia3MarkerType);
    coordinate_t markerHeight = (it != req.markerHeights.end()) ? it->second : MapBuilderRequest::defaultMarkerHeight;
    NEW_SHAPEVEC(blobs, BlobData, BlobData::extractBlobs(sketch, 100));
  
    vector<Shape<MarkerData> > markers;
  
    // Find the top and middle blobs first.  If they pass the test for
    // a marker, then look for a bottom blob below them.
    SHAPEVEC_ITERATE(blobs, BlobData, b1) {
      SHAPENEXT_ITERATE(blobs, BlobData, b1, b2) {
	Point c1 = b1->getCentroid();
	Point c2 = b2->getCentroid();
      
	// only look for markers with different top and middle colors
	if (b1->getColor() == b2->getColor())
	  continue;
      
	// areas should be similar
	float ratio = b1->getArea()/b2->getArea();
	if ((ratio < 0.5) || (ratio > 2.0))
	  continue;
      
	float dy = abs(c1.coordY() - c2.coordY());
	float dx = abs(c1.coordX() - c2.coordX());
      
	float width  = ((b1->topRight.coordX() - b1->topLeft.coordX())
			+ (b2->topRight.coordX() - b2->topLeft.coordX()))/2.f;
	float height = ((b1->bottomLeft.coordY() - b1->topLeft.coordY())
			+ (b2->bottomLeft.coordY() - b2->topLeft.coordY()))/2.f;
      
	// should be roughly aligned on top of each other
	if ((dx > 0.5 * width) || (dy > 1.5 * height))
	  continue;

	Shape<BlobData> topBlob = c1.coordY() < c2.coordY() ? b1 : b2;
	Shape<BlobData> middleBlob = c1.coordY() < c2.coordY() ? b2 : b1;
      
	Point center( (c1.coordX()+c2.coordX()) / 2,
		      (topBlob->bottomLeft.coordY() + middleBlob->topLeft.coordY()) / 2);
	calculateCameraDistance(center, markerHeight);

	SHAPEVEC_ITERATE(blobs, BlobData, bottomBlob) {

	  // bottom blob should be a different color than top and middle
	  if ( (bottomBlob->getColor() == topBlob->getColor()) ||
	       (bottomBlob->getColor() == middleBlob->getColor()) )
	    continue;

	  Point c3 = bottomBlob->getCentroid();
	  float dx3 = abs(center.coordX() - c3.coordX());

	  // bottom blob should have similar x position to top two, but height might
	  // not match if camera view is cut off, so just check that it's below their center
	  if ( dx3 > 0.5 * width  || center.coordY() > c3.coordY() )
	    continue;
      
	  Tapia3MarkerData *m = 
	    new Tapia3MarkerData(sketch->getSpace().getDualSpace(),
				 center,
				 topBlob->getColor(),
				 middleBlob->getColor(),
				 bottomBlob->getColor());

	markers.push_back(Shape<MarkerData>(m));
	} END_ITERATE;
      } END_ITERATE;
    } END_ITERATE;

    return markers;
  }

}; // namespace
