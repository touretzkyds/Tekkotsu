#include <iostream>
#include <iomanip>
#include <vector>
#include <sstream>
#include <string>
#include <float.h>
#include <cmath>

#include "ShapeRoot.h"
#include "ShapeAgent.h"
#include "ShapeLine.h"
#include "ShapeEllipse.h"
#include "ShapePoint.h"
#include "ShapeSphere.h"
#include "ShapeBlob.h"
#include "ShapePolygon.h"
#include "ShapeBrick.h"
#include "ShapePyramid.h"
#include "ShapeLocalizationParticle.h"
#include "ShapeMarker.h"
#include "ShapeCylinder.h"
#include "ShapeSift.h"
#include "ShapeAprilTag.h"
#include "ShapeGraphics.h"
#include "ShapeDomino.h"
#include "ShapeNaught.h"
#include "ShapeCross.h"
#include "ShapeSkeleton.h"

#include "BaseData.h"
#include "SketchSpace.h"
#include "ShapeSpace.h"
#include "VRmixin.h"

using namespace std;

#ifdef PLATFORM_APERIOS
// missing isfinite, Aibo crashes when it does denormalized float ops anyway... :(
static int isfinite(float s) { return (s == s) && ((s == 0) || (s != 2*s)); }
#endif

namespace DualCoding {

ShapeSpace::ShapeSpace(SketchSpace* dualSkS, int init_id, std::string const _name, 
		       ReferenceFrameType_t _refFrameType) : 
  name(_name.length() == 0 ? dualSkS->name : _name),
  dualSpace(dualSkS), id_counter(init_id), shapeCache(),
  refFrameType(_refFrameType)
{
  shapeCache = std::vector<ShapeRoot>(30);
  shapeCache.clear();
}

ShapeSpace::~ShapeSpace(void) {
  //cout << "Deleting ShapeSpace " << name << " at " << this << endl;
  // do not call clear() here; VRmixin::theAgent's reference count (in AgentData) may be wrong if the static Shape was already deleted
};

ShapeRoot& ShapeSpace::addShape(BaseData *p) {
  p->id = ++id_counter;
	p->importAdjust();  // adjust any components that need it (e.g., for polygons)
  shapeCache.push_back(ShapeRoot(p));
  return shapeCache.back();
};

void ShapeSpace::importShapes(std::vector<ShapeRoot>& foreign_shapes) {
  for (std::vector<ShapeRoot>::const_iterator it = foreign_shapes.begin();
       it != foreign_shapes.end(); ++it)
		importShape(*it);
}

BaseData* ShapeSpace::importShape(const ShapeRoot& foreign_shape) {
  BaseData *own = foreign_shape->clone();
  own->space = this;
  own->parentId = foreign_shape.id;
  own->lastMatchId = foreign_shape.id;
  own->refcount = 0;
  addShape(own);
  return own;  // return value is used by MapBuilder::importLocalToWorld
}

void ShapeSpace::deleteShape(ShapeRoot &b) {
  if ( b.isValid() )
    for ( std::vector<ShapeRoot>::iterator it = shapeCache.begin();
	  it != shapeCache.end(); ++it ) {
      if ( it->id == b.id ) {
	shapeCache.erase(it);
	break;
      }
    }
  else
    std::cerr << "ERROR: Attempt to delete an invalid " << b << std::endl;
}

void ShapeSpace::deleteShapes(std::vector<ShapeRoot>& shapes_vec) {
  for (size_t i=0; i < shapes_vec.size(); i++)
    deleteShape(shapes_vec[i]);
}

void ShapeSpace::clear() {
  shapeCache.clear();
  if ( this == &VRmixin::worldShS && VRmixin::theAgent.isValid() )
    shapeCache.push_back(VRmixin::theAgent);
}

void ShapeSpace::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
  std::vector<ShapeRoot> allShapes_vec = allShapes();
  const size_t nshapes = allShapes_vec.size();
  for(size_t i = 0; i < nshapes; i++)
    allShapes_vec[i]->applyTransform(Tmat,newref);
}

Point ShapeSpace::getCentroid() {
  std::vector<ShapeRoot> allShapes_vec = allShapes();
  return(getCentroidOfSubset(allShapes_vec));
}

Point ShapeSpace::getCentroidOfSubset(const std::vector<ShapeRoot>& subsetShapes_vec) {
  const size_t nshapes = subsetShapes_vec.size();
  Point subset_centroid_pt = Point(0,0);	
  for(size_t cur_shape = 0; cur_shape < nshapes; cur_shape++)
    subset_centroid_pt += subsetShapes_vec[cur_shape]->getCentroid();
  return(subset_centroid_pt/(float)nshapes);	
}

std::vector<ShapeRoot> ShapeSpace::allShapes(ShapeType_t type) {
  allShapes();  // make sure cache is valid;
  std::vector<ShapeRoot> result;
  for ( std::vector<ShapeRoot>::const_iterator it = shapeCache.begin();
	it != shapeCache.end(); it++ )
    if ( (*it)->getType() == type )
      result.push_back(*it);
  return result;
}

std::vector<ShapeRoot> ShapeSpace::allShapes(rgb color) {
  std::vector<ShapeRoot> result(shapeCache.size());
  result.clear();
  for ( std::vector<ShapeRoot>::const_iterator it = shapeCache.begin();
	it != shapeCache.end(); ++it )
    if ( (*it)->getColor() == color )
      result.push_back(*it);
  return result;
}

ShapeRoot ShapeSpace::getShapeFromId(int id) {
  for ( std::vector<ShapeRoot>::const_iterator it = shapeCache.begin();
	it != shapeCache.end(); ++it )
    if ( (*it)->getId() == id )
      return *it;
  return ShapeRoot();
}

// must used fixed output format because Java can't parse scientific notation
#define writept(x) fixed << setprecision(2) << x.coordX() \
  << " " << fixed << setprecision(2) << x.coordY() << " " << fixed << setprecision(2) << x.coordZ()

std::string ShapeSpace::getShapeListForGUI(void) {
  std::vector<ShapeRoot> &allShapes_vec = allShapes();
  std::ostringstream liststream;
  for(unsigned int i = 0; i < allShapes_vec.size(); i++) {
    if ( allShapes_vec[i]->isViewable() ) {
      liststream << "shape" << endl;
      liststream << "id: " << allShapes_vec[i]->getId() << endl;
      liststream << "parentId: " << allShapes_vec[i]->getParentId() << endl;
      liststream << "name: " << allShapes_vec[i]->getName() << endl;
      liststream << "shapetype: " << allShapes_vec[i]->getType() << endl;
      liststream << "color: " << toString(allShapes_vec[i]->getColor()) << endl;
      liststream << "cxyz: " << writept(allShapes_vec[i]->getCentroid()) << endl;
      liststream << "obst/lm: " << allShapes_vec[i]->isObstacle() << " " << allShapes_vec[i]->isLandmark() << endl;
		
      if(allShapes_vec[i]->isType(lineDataType)) { // lineshape
	const Shape<LineData>& current = ShapeRootTypeConst(allShapes_vec[i],LineData);
	liststream << "e1xyz: " << writept(current->end1Pt()) << endl; 
	liststream << "e2xyz: " << writept(current->end2Pt()) << endl; 
	liststream << "r:" << current->getRhoNorm() << endl;
	liststream << "theta: " << current->getThetaNorm() << endl;
	liststream << "end1: " << (current->end1Pt().isValid()) << " " 
		   << (current->end1Pt().isActive()) << endl;
	liststream << "end2: " << (current->end2Pt().isValid()) << " "
		   << (current->end2Pt().isActive()) << endl;
      } 
		
      else if (allShapes_vec[i]->isType(ellipseDataType)) { // ellipseshape
	const Shape<EllipseData>& current = ShapeRootTypeConst(allShapes_vec[i],EllipseData);
	liststream << "axes: " << current->getSemimajor()
		   << " " << current->getSemiminor() << endl;
	liststream << "orientation: " << current->getOrientation() 
		   << endl;
      } 

      else if (allShapes_vec[i]->isType(pointDataType)) { // pointshape
	;
      }

      else if (allShapes_vec[i]->isType(agentDataType)) { // agentshape
	const Shape<AgentData>& current = ShapeRootTypeConst(allShapes_vec[i],AgentData);
	liststream << "orientation: " << current->getOrientation() << endl;
	fmat::Column<3> offset = current->getBoundingBoxOffset();
	fmat::Column<3> halfDimXYZ = current->getBoundingBoxHalfDims();
	liststream << "offsetX: " << offset[0] << endl;
	liststream << "offsetY: " << offset[1] << endl;
	//offsetZ needs to be written when dealing with 3D bounding box
	liststream << "halfDimX: " << halfDimXYZ[0] << endl;
	liststream << "halfDimY: " << halfDimXYZ[1] << endl;
	//dimH needs to be written when dealing with 3D bounding box
      }

      else if (allShapes_vec[i]->isType(sphereDataType)) { // sphereshape
	const Shape<SphereData>& current = ShapeRootTypeConst(allShapes_vec[i],SphereData);
	liststream << "radius: " << current->getRadius() << endl;
      }

      else if (allShapes_vec[i]->isType(polygonDataType)) { // polygonshape
	const Shape<PolygonData>& current = ShapeRootTypeConst(allShapes_vec[i],PolygonData);
liststream << "num_vtx: " << current->getVertices().size() << endl;
	for (std::vector<Point>::const_iterator vtx_it = current->getVertices().begin();
	     vtx_it != current->getVertices().end(); vtx_it++)
	  liststream << "vertex: " << writept((*vtx_it)) << endl;
	liststream << "end1_valid: " << (current->end1Ln().end1Pt().isValid()) << endl;
	liststream << "end2_valid: " << (current->end2Ln().end2Pt().isValid()) << endl;
      }
		
      else if (allShapes_vec[i]->isType(blobDataType)) { // blobshape
	const Shape<BlobData>& current = ShapeRootTypeConst(allShapes_vec[i],BlobData);
	const char* colorname = ProjectInterface::getColorName(current->getColor());
	liststream << "colorname: " << (colorname != NULL ? string(colorname) : toString(current->getColor())) << endl;
	liststream << "area: " << current->getArea() << endl;
	liststream << "orient: " << current->orientation << endl;
	liststream << "topLeft: " << writept(current->topLeft) << endl;
	liststream << "topRight: " << writept(current->topRight) << endl;
	liststream << "bottomLeft: " << writept(current->bottomLeft) << endl;
	liststream << "bottomRight :" << writept(current->bottomRight) << endl;
        liststream << "validTop: " << (current->topValid ? "true" : "false") << endl;
        liststream << "validBottom: " << (current->bottomValid ? "true" : "false") << endl;
        liststream << "validLeft: " << (current->leftValid ? "true" : "false") << endl;
        liststream << "validRight: " << (current->rightValid ? "true" : "false") << endl;

      }
      else if (allShapes_vec[i]->isType(brickDataType)) { // brickshape
	const Shape<BrickData>& current = ShapeRootTypeConst(allShapes_vec[i],BrickData);
	liststream << "GFL: " << writept(current->getGFL()) << endl;
	liststream << "GBL: " << writept(current->getGBL()) << endl;
	liststream << "GFR: " << writept(current->getGFR()) << endl;
	liststream << "GBR: " << writept(current->getGBR()) << endl;
	liststream << "TFL: " << writept(current->getTFL()) << endl;
	liststream << "TBL: " << writept(current->getTBL()) << endl;
	liststream << "TFR: " << writept(current->getTFR()) << endl;
	liststream << "TBR: " << writept(current->getTBR()) << endl;
      }
      else if (allShapes_vec[i]->isType(pyramidDataType)) { // pyramidshape
	const Shape<PyramidData>& current = ShapeRootTypeConst(allShapes_vec[i],PyramidData);
	liststream << "FL: " << writept(current->getFL()) << endl;
	liststream << "BL: " << writept(current->getBL()) << endl;
	liststream << "FR: " << writept(current->getFR()) << endl;
	liststream << "BR: " << writept(current->getBR()) << endl;
	liststream << "Top: " << writept(current->getTop()) << endl;
      }
      else if (allShapes_vec[i]->isType(localizationParticleDataType)) { // localizationparticleshape
	const Shape<LocalizationParticleData>& current = ShapeRootTypeConst(allShapes_vec[i],LocalizationParticleData);
	const float weight = current->getWeight();
	liststream << "orient/weight: " << current->getOrientation()
		  << " " << (!isfinite(weight) ? (weight>0 ? FLT_MAX : -FLT_MAX) : weight) << endl;
      }
      else if (allShapes_vec[i]->isType(markerDataType)) { // markershape
	const Shape<MarkerData>& current = ShapeRootTypeConst(allShapes_vec[i],MarkerData);
	liststream << MarkerData::getMarkerTypeName(current->typeOfMarker) << ":" << current->getMarkerDescription() << endl;
      }
      else if (allShapes_vec[i]->isType(cylinderDataType)) {
	const Shape<CylinderData>& current = ShapeRootTypeConst(allShapes_vec[i], CylinderData);
	liststream << "height: " << current->getHeight() << endl << "radius: " << current->getRadius() << endl;
	const fmat::Quaternion q = current->getOrientation().aboutX(M_PI/2);
        liststream << "orientation: " << q.getW() << " " << q.getX()
		   << " " << q.getY() << " " << q.getZ() << endl;
      }
      else if (allShapes_vec[i]->isType(siftDataType)) { // siftshape
 	const Shape<SiftData>& current = ShapeRootTypeConst(allShapes_vec[i],SiftData);
	liststream << "objectName: " << current->getObjectName() << endl;
	liststream << "objectID: " << current->getObjectID() << endl;
	liststream << "modelName: " << current->getModelName() << endl;
 	liststream << "topLeft: " << writept(current->getTopLeft()) << endl;
 	liststream << "topRight: " << writept(current->getTopRight()) << endl;
 	liststream << "bottomLeft: " << writept(current->getBottomLeft()) << endl;
 	liststream << "bottomRight :" << writept(current->getBottomRight()) << endl;
      }
      else if (allShapes_vec[i]->isType(aprilTagDataType)) { // apriltagshape
 	const Shape<AprilTagData>& current = ShapeRootTypeConst(allShapes_vec[i],AprilTagData);
	const AprilTags::TagDetection &td = current->getTagDetection();
	const fmat::Quaternion q = current->getQuaternion();
	liststream << "tagID: " << current->getTagID() << endl;
	liststream << "orient: " << q.ypr()[0] << endl;
	liststream << "quaternion: " << q.getW() << " " << q.getX() << " " << q.getY() << " " << q.getZ() << endl;
	liststream << "hammingDistance: " << td.hammingDistance << endl;
	liststream << "p0: " << td.p[0].first << " " << td.p[0].second << endl;
	liststream << "p1: " << td.p[1].first << " " << td.p[1].second << endl;
	liststream << "p2: " << td.p[2].first << " " << td.p[2].second << endl;
	liststream << "p3: " << td.p[3].first << " " << td.p[3].second << endl;
      }
      else if (allShapes_vec[i]->isType(graphicsDataType)) { // graphicsshape
	const Shape<GraphicsData>& current = ShapeRootTypeConst(allShapes_vec[i],GraphicsData);
	liststream << "numelts: " << current->elements.size() << endl;
	for (unsigned int j = 0; j < current->elements.size(); j++) {
	  const GraphicsData::GraphicsElement *elt = current->elements[j];
	  switch ( elt->getType() ) {
	    
	  case GraphicsData::lineGType: {
	    const GraphicsData::LineElement *line = 
	      dynamic_cast<const GraphicsData::LineElement*>(elt);
	    liststream << "lineElement: " << elt->getType() << " " << toString(elt->getColor()) << " " << elt->getName() << endl;
	    liststream << "points: " << line->pt1.first << " " << line->pt1.second << " "
		       << line->pt2.first << " " << line->pt2.second << endl;
	    break; }
	    
	  case GraphicsData::polygonGType: {
	    const GraphicsData::PolygonElement *polygon =
	      dynamic_cast<const GraphicsData::PolygonElement*>(elt);
	    liststream<< "polygonElement: " << elt->getType() << " "<< toString(elt->getColor()) << " " << elt->getName() << endl;
	    liststream << "numvtx: "<< polygon->vertices.size() << endl;
	    for (unsigned int k = 0; k < polygon->vertices.size(); k++) {
	      std::pair<float,float> vert = polygon->vertices[k];
	      liststream << "vertex: " << vert.first << " " << vert.second << endl;
	    }
        liststream << "closed: " << (polygon->closed ? "true" : "false") << endl;
	    break; }
	    
	  case GraphicsData::ellipseGType: {
	    const GraphicsData::EllipseElement *ellipse = 
	      dynamic_cast<const GraphicsData::EllipseElement*>(elt);
	    liststream << "ellipseElement: " << elt->getType() << " " << toString(elt->getColor()) << " " << elt->getName() << endl;
	    liststream << "centroid: " << ellipse->center.first << " " << ellipse->center.second << endl;
	    liststream << "semimajor: " << ellipse->semimajor << endl;
	    liststream << "semiminor: " << ellipse->semiminor << endl;
	    liststream << "orientation: " << ellipse->orientation << endl;
	    liststream << "filled: : " << (ellipse->filled ? "true" : "false") << endl;
	    break; }

	  case GraphicsData::textGType: {
	    const GraphicsData::TextElement *text =
	      dynamic_cast<const GraphicsData::TextElement*>(elt);
	    liststream << "textElement: " << elt->getType() << " " << toString(elt->getColor()) << " " << elt->getName() << endl;
	    liststream << "startpoint: " << text->startpt.first << " " << text->startpt.second << endl;
	    liststream << "message: " << text->msg << endl;
	    break; }
	    
	  case GraphicsData::locParticleGType: {
	    const GraphicsData::LocalizationParticleElement *locParticle =
	      dynamic_cast<const GraphicsData::LocalizationParticleElement*>(elt);
	    liststream << "localizationParticleElement: " << elt->getType() << " " << toString(elt->getColor()) << " " << elt->getName() << endl;
	    liststream << "centroid: " << locParticle->getCentroid().coordX() << " " << locParticle->getCentroid().coordY() <<  endl;
	    liststream << "orientation: " << locParticle->getOrientation() 
		<< " weight: " << locParticle->getWeight() << endl;
	    break; }

          case GraphicsData::axisAngleGType: {
            const GraphicsData::AxisAngleElement *axisAngle = dynamic_cast<const GraphicsData::AxisAngleElement*>(elt);
            liststream << "axisAngleElement: " << elt->getType() << " " << toString(elt->getColor()) << endl;
            liststream << "Quaternion: " << axisAngle->q.getW() << " " << axisAngle->q.getX() << " " << axisAngle->q.getY() << " " << axisAngle->q.getZ() << endl;
            liststream << "Centroid: " << axisAngle->centroid[0] << " " << axisAngle->centroid[1] << " " << axisAngle->centroid[2] << endl;
            break; }
              
          case GraphicsData::pointGType: {
	    const GraphicsData::PointElement *point = 
	      dynamic_cast<const GraphicsData::PointElement*>(elt);
	    liststream << "pointElement: " << elt->getType() << " " << toString(elt->getColor()) << " " << elt->getName() << endl;
	    liststream << "centroid: " << point->center.first << " " << point->center.second << endl;
	    break; }
          case GraphicsData::boundingGType: {
            const GraphicsData::BoundingBoxElement *bb = dynamic_cast<const GraphicsData::BoundingBoxElement*>(elt);
            liststream << "boundingElement: " << elt->getType() << " " << toString(elt->getColor()) << endl;
            if (bb->vertices.size() <= 0) {
                liststream << "draw vertex: " << "false" << endl;
                liststream << "centroid: " << bb->centroid[0] << " " << bb->centroid[1] << " " << bb->centroid[2] << endl;
                liststream << "dimensions: " << bb->w << " " << bb->h << " " << bb->l << endl;
                liststream << "quaternion: " << bb->q.getW() << " " << bb->q.getX() << " " << bb->q.getY() << " " << bb->q.getZ() << endl;
            }
            else {
                liststream << "draw vertex: " << "true" << endl;
                liststream << "numer of components: " << bb->vertices.size() << endl;
                for (size_t vi = 0; vi < bb->vertices.size(); vi++)
                    for (size_t vj = 0; vj < bb->vertices[vi].size(); vj++)
                        liststream << "vertex point: " << bb->vertices[vi][vj][0]
				   << " " << bb->vertices[vi][vj][1]
				   << " " << bb->vertices[vi][vj][2] << endl;
            }
            break; }
              
	  default:
	    std::cerr << "Bad GraphicsElement type: " << elt->getType() << std::endl;
	  }
	}
      }
			else if (allShapes_vec[i]->isType(dominoDataType)) { // dominoshape
				const Shape<DominoData>& current = ShapeRootTypeConst(allShapes_vec[i],DominoData);
				liststream << "GFL: " << writept(current->getGFL()) << endl;
				liststream << "GBL: " << writept(current->getGBL()) << endl;
				liststream << "GFR: " << writept(current->getGFR()) << endl;
				liststream << "GBR: " << writept(current->getGBR()) << endl;
				liststream << "TFL: " << writept(current->getTFL()) << endl;
				liststream << "TBL: " << writept(current->getTBL()) << endl;
				liststream << "TFR: " << writept(current->getTFR()) << endl;
				liststream << "TBR: " << writept(current->getTBR()) << endl;
				liststream << "values: " << current->getLowValue() << " " << current->getHighValue() << endl;
			}
			else if (allShapes_vec[i]->isType(naughtDataType)) { // naughtshape
        const Shape<NaughtData>& current = ShapeRootTypeConst(allShapes_vec[i], NaughtData);
        liststream << "height: " << current->getHeight() << endl;
        liststream << "radius: " << current->getRadius() << endl;
      }
			else if (allShapes_vec[i]->isType(crossDataType)) { // crossshape
        const Shape<CrossData>& current = ShapeRootTypeConst(allShapes_vec[i], CrossData);
        liststream << "e1xyz: " << writept(current->getLine1().end1Pt()) << endl;
        liststream << "e2xyz: " << writept(current->getLine1().end2Pt()) << endl;
        liststream << "e3xyz: " << writept(current->getLine2().end1Pt()) << endl;
        liststream << "e4xyz: " << writept(current->getLine2().end2Pt()) << endl;
        liststream << "armwidth: " << current->getArmWidth() << endl;
      }
      else if (allShapes_vec[i]->isType(skeletonDataType)) { // skeletonshape
 	const Shape<SkeletonData>& skelshape = ShapeRootTypeConst(allShapes_vec[i],SkeletonData);
	const Skeleton &skel = skelshape->getSkeleton();
	int numJoints = 0;
	for (unsigned int j = 1; j<=Skeleton::NumSkelJoints; j++)
	  if ( skel.validJoint(j) ) ++numJoints;
	liststream << "numJoints: " << numJoints << endl;
	for (unsigned int j = 1; j<=Skeleton::NumSkelJoints; j++)
	  if ( skel.validJoint(j) )
	    liststream << "joint: " << j << " "
		       << skel.joints[j].coordX() << " " << skel.joints[j].coordY() << " " << skel.joints[j].coordZ() << std::endl;
      }
    }
  }
  return liststream.str();	
}

#undef writept

void ShapeSpace::printParams() {
  cout << endl;
  cout << "SHAPE SPACE : PARAMETERS BEGIN" << endl;
  int cur, num;
  
  std::vector<ShapeRoot> &allShapes_vec = allShapes();
  num = (int)(allShapes_vec.size());
  
  for(cur = 0; cur < num; cur++) {
    
    cout << "SHAPE " <<  allShapes_vec[cur].getId() 
	 << " (" << cur+1 << " of " << num << ")" 
	 << endl;
    
    allShapes_vec[cur]->printParams();
    
    cout << "===========================================" << endl;
    cout << endl;
    
  }
  
  cout << endl;
  cout << "SHAPE SPACE : PARAMETERS END" << endl;
}

void ShapeSpace::printSummary()
{
  // JJW will this cause a memory leak?
  std::vector<ShapeRoot> allShapes_vec = allShapes();
  int cur;
  int num = (int)(allShapes_vec.size());
  cout << "SHAPE SPACE : SUMMARY BEGIN" << endl;
  std::vector<int> shape_counts;
  shape_counts.resize(numDataTypes,0);
  
  cout << endl << "Shape Listing:" << endl;
  for(cur = 0; cur < num; cur++) {
    cout << "Shape " << allShapes_vec[cur].getId() 
	 << " (" << cur+1 << " of " << num << ")."
	 << "\tType: " << allShapes_vec[cur]->getTypeName() 
	 << endl;
    shape_counts[allShapes_vec[cur]->getType()]++;
  }
  
  cout << endl << "Shape Counts:" << endl;
  num = numDataTypes;
  for(cur = 0; cur < num; cur++) {
    cout << "Shape Type " << cur << " " << data_name(cur) 
	 << ":\t" << shape_counts[cur] << endl;
  }
  cout << endl;
  
  cout << "SHAPE SPACE : SUMMARY END" << endl;
}

void ShapeSpace::deleteShapeType(ShapeType_t t) {
  std::vector<ShapeRoot> temp;
  temp.reserve(shapeCache.size());
  for ( std::vector<ShapeRoot>::const_iterator it = shapeCache.begin();
	it != shapeCache.end(); it++ )
    if ( (*it)->getType() != t )
      temp.push_back(*it);
  shapeCache = temp;
}

} // namespace
