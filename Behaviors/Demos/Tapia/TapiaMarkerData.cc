//-*-c++-*-
#include <vector>

#include "DualCoding/ShapeMarker.h"
#include "DualCoding/Sketch.h"

#include "DualCoding/ShapeFuns.h"
#include "DualCoding/BlobData.h"
#include "DualCoding/ShapeBlob.h"
#include "Crew/MapBuilderRequest.h"

#include "Behaviors/Demos/Tapia/TapiaMarkerData.h"

using namespace std;

namespace DualCoding {

  const MarkerType_t TapiaMarkerData::tapiaMarkerType =
    MarkerData::registerMarkerType("tapiaMarkerType",TapiaMarkerData::extractMarkers);

  TapiaMarkerData::TapiaMarkerData(ShapeSpace& _space, const Point& _center, rgb _top, rgb _bottom) :
    MarkerData(_space, _center, _top), topColor(_top), bottomColor(_bottom)
  { typeOfMarker = tapiaMarkerType; }

  DATASTUFF_CC(TapiaMarkerData);

  void TapiaMarkerData::printParams() const {
    cout << "Type = " << getTypeName() << "(Tapia)" << "  ID=" << getId() << "  ParentID=" << getParentId() << endl;
    printf("  top color = %d %d %d",topColor.red,topColor.green,topColor.blue);
    printf("  bottom color = %d %d %d\n",bottomColor.red,bottomColor.green,bottomColor.blue);
    cout << "  center =" << center.getCoords() << endl
	 << endl;
  }

  bool TapiaMarkerData::isMatchingMarker(const Shape<MarkerData>& other) const {
    // make sure they are the same type
    if (!(typeOfMarker == other->typeOfMarker))
      return false;
  
    // make sure colors are the same
    const TapiaMarkerData& other_tapia = (const TapiaMarkerData &)(other.getData());
    if (!(topColor == other_tapia.topColor && bottomColor == other_tapia.bottomColor))
      return false;  

    return true;
  }

  string TapiaMarkerData::getMarkerDescription() const {
    stringstream ss;
    ss << "coords=" << getCentroid();
    ss << " topColor=[" << toString(topColor) << "] bottomColor=[" << toString(bottomColor) << "]";
    return ss.str();
  }

  vector<Shape<MarkerData> > 
  TapiaMarkerData::extractMarkers(const Sketch<uchar> &sketch, const MapBuilderRequest &req) {
    std::map<MarkerType_t, coordinate_t>::const_iterator it = req.markerHeights.find(tapiaMarkerType);
    coordinate_t markerHeight = (it != req.markerHeights.end()) ? it->second : MapBuilderRequest::defaultMarkerHeight;
    NEW_SHAPEVEC(blobs, BlobData, BlobData::extractBlobs(sketch, 50));
  
    vector<Shape<MarkerData> > markers;
  
    SHAPEVEC_ITERATE(blobs, BlobData, b1) {
      b1->setViewable(false);
    
      SHAPENEXT_ITERATE(blobs, BlobData, b1, b2) {
	Point c1 = b1->getCentroid();
	Point c2 = b2->getCentroid();
      
	// only look for markers with two different colors
	if (b1->getColor() == b2->getColor())
	  continue;
      
	// areas should be similar
	float ratio = b1->getArea()/b2->getArea();
	if ((ratio < 0.5) || (ratio > 2.0))
	  continue;
      
	float dy = abs(c1.coordY() - c2.coordY());
	float dx = abs(c1.coordX() - c2.coordX());
      
	float width  = ((b1->topRight.coordX() - b1->topLeft.coordX())
			+ (b2->topRight.coordX() - b2->topLeft.coordX()))/2.0f;
	float height = ((b1->bottomLeft.coordY() - b1->topLeft.coordY())
			+ (b2->bottomLeft.coordY() - b2->topLeft.coordY()))/2.0f;
      
	// should be roughly aligned on top of each other
	if ((dx > 0.5 * width) || (dy > 1.5 * height))
	  continue;
      
	// figure out which one is on top
	Point center = (c1 + c2)/2.0f;
	calculateCameraDistance(center, markerHeight);
      
	rgb topColor =
	  (c1.coordY() < c2.coordY()) ? b1->getColor() : b2->getColor();
	rgb bottomColor =
	  (c1.coordY() < c2.coordY()) ? b2->getColor() : b1->getColor();
      
	TapiaMarkerData *m = new TapiaMarkerData(sketch->getSpace().getDualSpace(),
						 center,
						 topColor,
						 bottomColor);
	markers.push_back(Shape<MarkerData>(m));
      } END_ITERATE;
    } END_ITERATE;
  
    return markers;
  }

}; // namespace
