//-*-c++-*-
#include <vector>

#include "Shared/ProjectInterface.h"

#include "DualCoding/ShapeMarker.h"
#include "DualCoding/Sketch.h"

#include "DualCoding/ShapeFuns.h"
#include "DualCoding/BlobData.h"
#include "DualCoding/ShapeBlob.h"
#include "DualCoding/VRmixin.h"  // for camShS and mapBuilder
#include "Crew/MapBuilder.h"

#include "BiColorMarkerData.h"

using namespace std;

namespace DualCoding {

  const MarkerType_t BiColorMarkerData::biColorMarkerType =
    MarkerData::registerMarkerType("biColorMarkerType",BiColorMarkerData::extractMarkers);

  BiColorMarkerData::BiColorMarkerData(ShapeSpace& _space, const Point& _center, rgb _top, rgb _bottom) :
    MarkerData(_space, _center, _top), topColor(_top), bottomColor(_bottom)
  { typeOfMarker = biColorMarkerType; }

  DATASTUFF_CC(BiColorMarkerData);

  void BiColorMarkerData::printParams() const {
    cout << "Type = " << getTypeName() << "(BiColor)" << "  ID=" << getId() << "  ParentID=" << getParentId() << endl;
    printf("  top color = %d %d %d",topColor.red,topColor.green,topColor.blue);
    printf("  bottom color = %d %d %d\n",bottomColor.red,bottomColor.green,bottomColor.blue);
    cout << "  center =" << center.getCoords() << endl
	 << endl;
  }

  bool BiColorMarkerData::isMatchingMarker(const Shape<MarkerData>& other) const {
    // make sure they are the same type
    if (!(typeOfMarker == other->typeOfMarker))
      return false;
  
    // make sure colors are the same
    const BiColorMarkerData& other_marker = (const BiColorMarkerData&)(other.getData());
    if (!(topColor == other_marker.topColor && bottomColor == other_marker.bottomColor))
      return false;  

    return true;
  }

  string BiColorMarkerData::getMarkerDescription() const {
    stringstream ss;
    ss << ProjectInterface::getColorName(topColor) << "/" << ProjectInterface::getColorName(bottomColor);
    ss << " coords=" << getCentroid();
    return ss.str();
  }

  vector<Shape<MarkerData> > 
  BiColorMarkerData::extractMarkers(const Sketch<uchar> &sketch, const MapBuilderRequest &req) {

    vector<Shape<MarkerData> > markers;

    // Figure out the designated height of the marker center above the ground plane
    std::map<MarkerType_t, coordinate_t>::const_iterator it = req.markerHeights.find(biColorMarkerType);
    coordinate_t markerHeight = (it != req.markerHeights.end()) ? it->second : MapBuilderRequest::defaultMarkerHeight;

    // Find the colors we're using for markers, so we can check each blob color
    std::map<ShapeType_t, std::set<color_index> >::const_iterator it_c = req.objectColors.find(markerDataType);
    if ( it_c == req.objectColors.end() )
      return markers;
    const std::set<color_index> &colors = it_c->second;

    NEW_SHAPEVEC(blobs, BlobData, select_type<BlobData>(VRmixin::camShS));
    if ( blobs.empty() ) {
      VRmixin::mapBuilder->getCamBlobs(colors, 50);
      blobs = select_type<BlobData>(VRmixin::camShS);
    }
  
    SHAPEVEC_ITERATE(blobs, BlobData, b1) {
      if ( colors.find(ProjectInterface::getColorIndex(b1->getColor())) == colors.end() )
	continue;
      Point c1 = b1->getCentroid();
    
      SHAPENEXT_ITERATE(blobs, BlobData, b1, b2) {
	// only look for markers with two different allowed colors
	if ( b1->getColor() == b2->getColor()  ||
	     colors.find(ProjectInterface::getColorIndex(b2->getColor())) == colors.end() )
 	  continue;
      
	// widths and areas should be similar
	const float widthRatio = 
	  (b1->topRight.coordX() - b1->topLeft.coordX()) /
	  (b2->topRight.coordX() - b2->topLeft.coordX());
	if ( widthRatio < 0.5 || widthRatio > 2.0 )
	  continue;
	const float areaRatio = b1->getArea()/b2->getArea();
	if ( areaRatio < 0.5 || areaRatio > 2.0 )
	  continue;
      
	Point c2 = b2->getCentroid();
	float dy = abs(c1.coordY() - c2.coordY());
	float dx = abs(c1.coordX() - c2.coordX());
      
	float width  = max(b1->topRight.coordX() - b1->topLeft.coordX(),
			   b2->topRight.coordX() - b2->topLeft.coordX());
	float height = max(b1->bottomLeft.coordY() - b1->topLeft.coordY(),
			   b2->bottomLeft.coordY() - b2->topLeft.coordY());
      
	// should be roughly aligned on top of each other, but
	// relax the width in case the marker is only partly in the camera frame
	float const minWidthForComparison = 10;
	if ( (dx > max(width/2, minWidthForComparison)) || (dy > 1.75 * height) || (dy < 0.5 * height) )  // change 0.75*height to 0.5*height to accomodate distorted views of planar markers
	  continue;
      
	// figure out which one is on top
	Shape<BlobData> &topBlob    = (c1.coordY() < c2.coordY()) ? b1 : b2;
	Shape<BlobData> &bottomBlob = (c1.coordY() < c2.coordY()) ? b2 : b1;
	
	Point center((c1.coordX()+c2.coordX())/2.0,
		     (topBlob->getBoundingBox().max[1] + bottomBlob->getBoundingBox().min[1])/2.0,
		     0,
		     camcentric);
	calculateCameraDistance(center, markerHeight);
      
	BiColorMarkerData *marker = 
	  new BiColorMarkerData(sketch->getSpace().getDualSpace(), center, 
				topBlob->getColor(), bottomBlob->getColor());
	markers.push_back(Shape<MarkerData>(marker));
      } END_ITERATE;
    } END_ITERATE;
  
    return markers;
  }

}; // namespace
