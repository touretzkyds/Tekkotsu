//-*-c++-*-

#include <iostream>
#include <vector>

#include "BaseData.h"    // superclass
#include "Point.h"       // Point data member
#include "ShapeTypes.h"  // pyramidDataType

#include "SketchSpace.h"
#include "Sketch.h"
#include "visops.h"

#include "ShapeSpace.h"  // required by DATASTUFF_CC
#include "ShapeRoot.h"   // required by DATASTUFF_CC

#include "PyramidData.h"
#include "ShapePyramid.h"
#include "ShapePoint.h"
#include "Region.h"

using namespace std;

namespace DualCoding {

  PyramidData::PyramidData(ShapeSpace& _space,
			   const EndPoint &_FL, const EndPoint &_FR, 
			   const EndPoint &_BL, const EndPoint &_BR, 
			   const EndPoint &_Top)
    : BaseData(_space,pyramidDataType), 
      FL(_FL), FR(_FR), BL(_BL), BR(_BR), Top(_Top),
      centroid((FL + FR + BL + BR + Top) / 5)
  {
  }

  
  DATASTUFF_CC(PyramidData);

  bool PyramidData::isMatchFor(const ShapeRoot& other) const {
    if (!(isSameTypeAs(other) && isSameColorAs(other)))
      return false;

    // needs implementation
    float dist = 0;
    return dist < 20; 
  }
  
  void PyramidData::mergeWith(const ShapeRoot& other) {
    const Shape<PyramidData>& other_pyramid = ShapeRootTypeConst(other,PyramidData);
    if (other_pyramid->confidence <= 0)
      return;
  }

  bool PyramidData::updateParams(const ShapeRoot& other, bool) {
    const Shape<PyramidData>& other_pyramid = *static_cast<const Shape<PyramidData>*>(&other);
    //  ++confidence;
    FL = (FL*(confidence-1) + other_pyramid->getFL())/confidence;
    FR = (FR*(confidence-1) + other_pyramid->getFR())/confidence;
    BL = (BL*(confidence-1) + other_pyramid->getBL())/confidence;
    BR = (BR*(confidence-1) + other_pyramid->getBR())/confidence;
    Top = (Top*(confidence-1) + other_pyramid->getTop())/confidence;
    deleteRendering();
    return true;
  }

  //! Print information about this shape. (Virtual in BaseData.)
  void PyramidData::printParams() const {
    cout << "Type = " << getTypeName();
    cout << "Shape ID = " << getId() << endl;
    cout << "Parent ID = " << getParentId() << endl;
  
    printf("color = %d %d %d\n",getColor().red,getColor().green,getColor().blue);
    cout << "viewable = " << isViewable() << endl;
  }


  //! Transformations. (Virtual in BaseData.)
  void PyramidData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
    FL.applyTransform(Tmat,newref);
    FR.applyTransform(Tmat,newref);
    BL.applyTransform(Tmat,newref);
    BR.applyTransform(Tmat,newref);
    Top.applyTransform(Tmat,newref);
  }

  /*void PyramidData::projectToGround(const fmat::Column<4>& groundplane) {
    FL.projectToGround(groundplane);
    FR.projectToGround(groundplane);
    BL.projectToGround(groundplane);
    BR.projectToGround(groundplane);

    Point bottomCenter = (FL + FR + BL + BR) / 4.0;
    float height;
    Top.projectToGround(groundplane);
    height = Top.getHeightAbovePoint(bottomCenter, groundplane);
    Top.setCoords(bottomCenter.coordX(), bottomCenter.coordY(), bottomCenter.coordZ() + height);
    centroid = bottomCenter;
    centroid.setCoords(centroid.coordX(), centroid.coordY(), centroid.coordZ() + height/5);
    std::cout<<"New centroid: ("<<centroid.coordX()<<","<<centroid.coordY()<<","<<centroid.coordZ()<<")\n";
    }*/

  void PyramidData::projectToGround(const fmat::Transform & camToBase, const PlaneEquation& groundplane) {
    FL.projectToGround(camToBase, groundplane);
    FR.projectToGround(camToBase, groundplane);
    BL.projectToGround(camToBase, groundplane);
    BR.projectToGround(camToBase, groundplane);

    Point bottomCenter = (FL + FR + BL + BR) / 4.f;
    bottomCenter.setRefFrameType(egocentric);
    float height;
    Top.projectToGround(camToBase, groundplane);
    height = Top.getHeightAbovePoint(bottomCenter, groundplane);
    Top.setCoords(bottomCenter.coordX(), bottomCenter.coordY(), bottomCenter.coordZ() + height);
    centroid = bottomCenter;
    centroid.setCoords(centroid.coordX(), centroid.coordY(), centroid.coordZ() + height/5);
    std::cout << "New centroid: " << centroid << endl;
  }

  // =====================================================
  // BEGIN SKETCH MANIPULATION AND PYRAMID EXTRACTION CODE
  // =====================================================


  //! Pyramid extraction.


  //! Render into a sketch space and return reference. (Private.)
  Sketch<bool>* PyramidData::render() const {
    SketchSpace &renderspace = space->getDualSpace();
    //int const width = renderspace.getWidth();
    //int const height = renderspace.getHeight();
    //float x1,y1,x2,y2;
    Sketch<bool>* draw_result = 
      new Sketch<bool>(renderspace, "render("+getName()+")");
    (*draw_result)->setParentId(getViewableId());
    (*draw_result)->setColor(getColor());
    *draw_result = 0;
    LineData F(*space, FL, FR);
    *draw_result = *draw_result & F.getRendering();
    LineData L(*space, FL, BL);
    *draw_result = *draw_result & L.getRendering();
    LineData B(*space, BL, BR);
    *draw_result = *draw_result & B.getRendering();
    LineData R(*space, BR, FR);
    *draw_result = *draw_result & R.getRendering();
    LineData FT(*space, FL, Top);
    *draw_result = *draw_result & FT.getRendering();
    LineData LT(*space, FR, Top);
    *draw_result = *draw_result & LT.getRendering();
    LineData BT(*space, BL, Top);
    *draw_result = *draw_result & BT.getRendering();
    LineData RT(*space, BR, Top);
    *draw_result = *draw_result & RT.getRendering();
  
    return draw_result;
  }




  /* Same idea as extractBrick, but simpler
   */
  Shape<PyramidData> PyramidData::extractPyramid(ShapeSpace& space, vector<Shape<BlobData> > &blobs)
  {
    unsigned int nblobs = blobs.size();
    std::vector<Point> centroids;

    ShapeRoot invalid;
    if (nblobs != 2) {
      return ShapeRootType(invalid,PyramidData);
    }

    for (unsigned int i=0; i<nblobs; i++) {
      centroids.push_back(blobs[i]->getCentroid());
    }
  
    // Check inter-blob distance
    if (centroids[0].distanceFrom(centroids[1]) >= 
	blobs[0]->topLeft.distanceFrom(centroids[0]) + 
	blobs[1]->topLeft.distanceFrom(centroids[1])){
      return ShapeRootType(invalid,PyramidData);	
    }

    int left=-1, right=-1;
  
    if (centroids[0].isLeftOf(centroids[1], camcentric)) {
      left = 0;
      right = 1;
    }
    else {
      left = 1;
      right = 0;
    }

    const int INTERSECT_DIST = 5, MIN_REQUIRED_DIST = 2;
    NEW_SKETCH_N(leftEdist, uchar, visops::mdist(blobs[left]->getRendering()));
    NEW_SKETCH_N(rightEdist, uchar, visops::mdist(blobs[right]->getRendering()));
    NEW_SKETCH(small, bool, (leftEdist<MIN_REQUIRED_DIST) & (rightEdist<MIN_REQUIRED_DIST));
    if (!small->max()){
      return ShapeRootType(invalid,PyramidData);	
    }
    NEW_SKETCH(boundary, bool, (leftEdist<INTERSECT_DIST) & (rightEdist<INTERSECT_DIST));
    rgb green_rgb(0, 255, 0);
    boundary->setColor(green_rgb);

    // Extract candidate points
    std::vector<std::vector<Point> > points(5);

    /*
     *          0
     *         /|\ 
     *        / | \  
     *       /  |  \ 
     *      / L | R \ 
     *     /    |    \ <--- 4
     *    1-----2-----3
     */


    NEW_SHAPE(boundaryline, LineData, LineData::extractLine(boundary));

    if (boundaryline.isValid()) {
      points[0].push_back(boundaryline->topPt());
      points[2].push_back(boundaryline->bottomPt());
    }


    // Find the corners for each face individually
    // blobData::findCorners takes the number of corners you expect, so this 
    // uses the same method as extractBrick

    std::vector<Point> candidates; 

    candidates = findBoundingTriangle(space, blobs[left], 
				      centroids[left], boundaryline);

    float cornerValue;

    std::vector<Point> lcorners = blobs[left]->findCorners(3, candidates, cornerValue);

    if (lcorners.size() == 3) {

      unsigned int leftCorner = 0;
      for (unsigned int i=1; i<3; i++){
	if (lcorners[i].isLeftOf(lcorners[leftCorner], camcentric)) {
	  leftCorner = i;
	}
      }

      points[1].push_back(lcorners[leftCorner]);
      points[2].push_back(lcorners[(leftCorner+1)%3]);
      points[0].push_back(lcorners[(leftCorner+2)%3]);
    
    }
    else {
      return ShapeRootType(invalid,PyramidData);
    }


    // find the corners of the right face.

    candidates.clear();
    candidates = findBoundingTriangle(space, blobs[right], 
				      centroids[right], boundaryline);

    std::vector<Point> rcorners = blobs[right]->findCorners(3, candidates, cornerValue);

    if (rcorners.size() == 3) {

      unsigned int rightCorner = 0;
      for (unsigned int i=1; i<3; i++){
	if (rcorners[i].isRightOf(rcorners[rightCorner], camcentric)) {
	  rightCorner = i;
	}
      }

      points[3].push_back(rcorners[rightCorner]);
      points[0].push_back(rcorners[(rightCorner+1)%3]);
      points[2].push_back(rcorners[(rightCorner+2)%3]);
    
    }
    else {
      return ShapeRootType(invalid,PyramidData);
    }

    // debug output
    for (unsigned int i=0; i<5; i++){
      for (unsigned int j=0; j<points[i].size(); j++) {
	NEW_SHAPE(corner, PointData, PointData(space, points[i][j]));
	char name[10];
	sprintf(name, "corner%d%d",i,j);
	corner->setName(name);
      }
    }


    vector<Point> finalCorners(5);

    Point empty(0,0);

    for (unsigned int i=0; i<5; i++) {
      finalCorners[i] = empty;
      for (unsigned int j=0; j<points[i].size(); j++) {
	finalCorners[i]+=points[i][j];
      }
      if (points[i].size() > 0) {
	finalCorners[i]/=points[i].size();
      }
    }




    finalCorners[4] = finalCorners[1] + finalCorners[3] - finalCorners[2];


    NEW_SHAPE(returnPyramid, PyramidData, new PyramidData(space,
							  finalCorners[1],
							  finalCorners[2],
							  finalCorners[4],
							  finalCorners[3],
							  finalCorners[0]));
  
    return returnPyramid;
  }



  // A bounding triangle with room around it
  // contracted in quadrilateral (polygon) fitting
  vector<Point> PyramidData::findBoundingTriangle(ShapeSpace& space, Shape<BlobData> blob, 
						  Point /*centroid*/, Shape<LineData> parallel)
  {

    const float PARALLEL_EXTEND_FACTOR = .4f, ORTHO_EXTEND_FACTOR = .4f;
    
    vector<Point> candidates;
    
    Point thirdPoint = (Region::extractRegion(blob->getRendering())).mostDistantPtFrom(parallel.getData());
    
    LineData perpendicular(space, thirdPoint, parallel->getThetaNorm());

    Point isect(perpendicular.intersectionWithLine(parallel));
    
    Point orthoExtend((thirdPoint - isect)*ORTHO_EXTEND_FACTOR);
    Point parallelExtend((parallel->end2Pt() - parallel->end1Pt())*PARALLEL_EXTEND_FACTOR);

    Point pt1(parallel->end1Pt() - parallelExtend - orthoExtend);
    Point pt2(parallel->end2Pt() + parallelExtend - orthoExtend);

    if ((pt1.isAbove(pt2, camcentric) && thirdPoint.isLeftOf(pt1, camcentric)) || 
	(!pt1.isAbove(pt2, camcentric) && !thirdPoint.isLeftOf(pt1, camcentric))){
      candidates.push_back(pt2);
      candidates.push_back(pt1);
    }
    else {
      candidates.push_back(pt1);
      candidates.push_back(pt2);
    }
    candidates.push_back(thirdPoint + orthoExtend);

    // debug output
    NEW_SHAPE(candidate31, PointData, PointData(space, candidates[0]));
    NEW_SHAPE(candidate32, PointData, PointData(space, candidates[1]));
    NEW_SHAPE(candidate33, PointData, PointData(space, candidates[2]));
    candidate31->setParentId(blob->getViewableId());
    candidate32->setParentId(blob->getViewableId());
    candidate33->setParentId(blob->getViewableId());

    return candidates;
  }


} // namespace
