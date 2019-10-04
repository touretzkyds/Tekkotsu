//-*-c++-*-

#include <iostream>
#include <vector>

#include "BaseData.h"    // superclass
#include "Point.h"       // Point data member
#include "ShapeTypes.h"  // brickDataType

#include "SketchSpace.h"
#include "Sketch.h"
#include "visops.h"

#include "ShapeSpace.h"  // required by DATASTUFF_CC
#include "ShapeRoot.h"   // required by DATASTUFF_CC

#include "BrickData.h"
#include "ShapeBrick.h"
#include "ShapePoint.h"
#include "Region.h"

using namespace std;

namespace DualCoding {

BrickData::BrickData(ShapeSpace& _space,
										 const EndPoint &_GFL, const EndPoint &_GBL, 
										 const EndPoint &_GFR, const EndPoint &_GBR, 
										 const EndPoint &_TFL, const EndPoint &_TBL, 
										 const EndPoint &_TFR, const EndPoint &_TBR,
										 const fmat::Quaternion &orient)
  : BaseData(_space,brickDataType), 
    GFL(_GFL), GFR(_GFR), GBL(_GBL), GBR(_GBR),
    TFL(_TFL), TFR(_TFR), TBL(_TBL), TBR(_TBR),
    centroid((GFL + GFR + GBL + GBR + TFL + TFR + TBL + TBR) / 8),
		orientation(orient) {}
  
BrickData::BrickData(ShapeSpace& _space,
                       const fmat::SubVector<3,const fmat::fmatReal>& _centroid,
                       fmat::Column<3> extents,
                       const fmat::SubMatrix<3,3,const fmat::fmatReal>& o)
  : BaseData(_space, brickDataType),
		GFL(), GFR(), GBL(), GBR(), TFL(), TFR(), TBL(), TBR(),
		centroid(_centroid, _space.getRefFrameType()),
		orientation(fmat::Quaternion::fromMatrix(o)) {
	TFR.coords = o * extents + _centroid; //+++
	extents[2] = -extents[2];
	GFR.coords = o * extents + _centroid; //++-
	extents[1] = -extents[1];
	GBR.coords = o * extents + _centroid; //+--
	extents[2] = -extents[2];
	TBR.coords = o * extents + _centroid; //+-+
	extents[0] = -extents[0];
	TBL.coords = o * extents + _centroid; //--+
	extents[2] = -extents[2];
	GBL.coords = o * extents + _centroid; //---
	extents[1] = -extents[1];
	GFL.coords = o * extents + _centroid; //-+-
	extents[2] = -extents[2];
	TFL.coords = o * extents + _centroid; //-++
	GFL.refFrameType = GFR.refFrameType = GBL.refFrameType = GBR.refFrameType =
		TFL.refFrameType = TFR.refFrameType = TBL.refFrameType = TBR.refFrameType = centroid.refFrameType;
}

DATASTUFF_CC(BrickData);

BoundingBox2D BrickData::getBoundingBox() const {
	BoundingBox2D b;
	b.expand(fmat::pack(GFL.coordX(), GFL.coordY()));
	b.expand(fmat::pack(GBR.coordX(), GBR.coordY()));
	return b;
}

bool BrickData::isMatchFor(const ShapeRoot& other) const {
  if (!(isSameTypeAs(other) && isSameColorAs(other)))
    return false;
  const Shape<BrickData>& other_brick = ShapeRootTypeConst(other,BrickData);
  float dist = getCentroid().distanceFrom(other_brick->getCentroid());
  return dist < 25;
}

void BrickData::mergeWith(const ShapeRoot& other) {
  const Shape<BrickData>& other_brick = ShapeRootTypeConst(other,BrickData);
  if (other_brick->confidence <= 0)
    return;
  /*
  const int other_conf = other_point->confidence;
  confidence += other_conf;
  the_point = (the_point*confidence + other_point->getCentroid()*other_conf) / (confidence+other_conf);*/
}

bool BrickData::updateParams(const ShapeRoot& other, bool) {
  const Shape<BrickData>& other_brick = *static_cast<const Shape<BrickData>*>(&other);
	++confidence;
  /*
	cout << "*Brick*Update*  id=" << getId() << "," << other->getId() << " conf=" << confidence
			 << " GFL=" << GFL.coordX() << " <- " << other_brick->getGFL().coordX()
			 << " GFR=" << GFR.coordX() << " <- " << other_brick->getGFR().coordX() << endl;
	*/
  GFL = (GFL*(confidence-1) + other_brick->getGFL()) / confidence;
  GFR = (GFR*(confidence-1) + other_brick->getGFR()) / confidence;
  GBL = (GBL*(confidence-1) + other_brick->getGBL()) / confidence;
  GBR = (GBR*(confidence-1) + other_brick->getGBR()) / confidence;
  TFL = (TFL*(confidence-1) + other_brick->getTFL()) / confidence;
  TFR = (TFR*(confidence-1) + other_brick->getTFR()) / confidence;
  TBL = (TBL*(confidence-1) + other_brick->getTBL()) / confidence;
  TBR = (TBR*(confidence-1) + other_brick->getTBR()) / confidence;
  centroid = (GFL + GFR + GBL + GBR + TFL + TFR + TBL + TBR)/8;
	// not updating the orientation yet
  deleteRendering();
  return true;
}

//! Print information about this shape. (Virtual in BaseData.)
void
BrickData::printParams() const {
  cout << "Type = " << getTypeName();
  cout << "Shape ID = " << getId() << endl;
  cout << "Parent ID = " << getParentId() << endl;
	float orient = getOrientation().angle();
	cout << "Orient = " << getOrientation() << " angle " << orient
			 << " = " << orient*180.0/M_PI << " deg." << endl;
	cout << "GFL: "; GFL.printData(); cout << endl;
  cout << "GFR: "; GFR.printData(); cout << endl;
  cout << "GBL: "; GBL.printData(); cout << endl;
  cout << "GBR: "; GBR.printData(); cout << endl;
  cout << "TFL: "; TFL.printData(); cout << endl;
  cout << "TFR: "; TFR.printData(); cout << endl;
  cout << "TBL: "; TBL.printData(); cout << endl;
  cout << "TBR: "; TBR.printData(); cout << endl;
  printf("color = %d %d %d\n",getColor().red,getColor().green,getColor().blue);
}


//! Transformations. (Virtual in BaseData.)
void BrickData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
  GFL.applyTransform(Tmat,newref);
  GFR.applyTransform(Tmat,newref);
  GBL.applyTransform(Tmat,newref);
  GBR.applyTransform(Tmat,newref);
  TFL.applyTransform(Tmat,newref);
  TFR.applyTransform(Tmat,newref);
  TBL.applyTransform(Tmat,newref);
  TBR.applyTransform(Tmat,newref);
	centroid.applyTransform(Tmat,newref);
	orientation = fmat::Quaternion::fromMatrix<fmat::Transform>(orientation * Tmat);
}

void BrickData::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {
  GFL.projectToGround(camToBase,groundplane);
  GFR.projectToGround(camToBase,groundplane);
  GBL.projectToGround(camToBase,groundplane);
  GBR.projectToGround(camToBase,groundplane);

  // Compute height at every corner except BL (because we didn't actually observe that corner)
  float FLHeight, FRHeight, BRHeight;
  TFL.projectToGround(camToBase,groundplane);
  TFR.projectToGround(camToBase,groundplane);
  TBR.projectToGround(camToBase,groundplane);
  FLHeight = TFL.getHeightAbovePoint(GFL, groundplane);
  FRHeight = TFR.getHeightAbovePoint(GFR, groundplane);
  BRHeight = TBR.getHeightAbovePoint(GBR, groundplane);
  float brickHeight = (FLHeight + FRHeight+ BRHeight) / 3.f;
  TFL.setCoords(GFL.coordX(), GFL.coordY(), GFL.coordZ() + FLHeight);
  TFR.setCoords(GFR.coordX(), GFR.coordY(), GFR.coordZ() + FRHeight);
  TBL.setCoords(GBL.coordX(), GBL.coordY(), GBL.coordZ() + brickHeight);
  TBR.setCoords(GBR.coordX(), GBR.coordY(), GBR.coordZ() + BRHeight);
  centroid = (GFL + GFR + GBL + GBR)/4;
  centroid.setCoords(centroid.coordX(), centroid.coordY(), brickHeight/2);
  centroid.setRefFrameType(egocentric);
	orientation = fmat::Quaternion::fromMatrix<fmat::Transform>(orientation * camToBase);
  std::cout << "New centroid: " << centroid << endl;

}

// ==================================================
// BEGIN SKETCH MANIPULATION AND BRICK EXTRACTION CODE
// ==================================================


//! Brick extraction.


//! Render into a sketch space and return reference. (Private.)
Sketch<bool>* BrickData::render() const {
  SketchSpace &renderspace = space->getDualSpace();
  //int const width = renderspace.getWidth();
  //int const height = renderspace.getHeight();
  //float x1,y1,x2,y2;
  Sketch<bool>* draw_result = 
    new Sketch<bool>(renderspace, "render("+getName()+")");
  (*draw_result)->setParentId(getViewableId());
  (*draw_result)->setColor(getColor());
  *draw_result = 0;
  LineData GF(*space, GFL, GFR);
  *draw_result = *draw_result | GF.getRendering();
  LineData GL(*space, GFL, GBL);
  *draw_result = *draw_result | GL.getRendering();
  LineData GB(*space, GBL, GBR);
  *draw_result = *draw_result | GB.getRendering();
  LineData GR(*space, GBR, GFR);
  *draw_result = *draw_result | GR.getRendering();
  
  return draw_result;
}




  // Old version of brick extraction
  // Final version resides in extractBrick
std::vector<Shape<BrickData> > BrickData::findBricks(ShapeSpace& ShS, std::vector<Shape<LineData> > lines)
{
  const float lengthConst = .3f;
  std::vector<Shape<LineData> > resultLines;
  std::vector<Shape<BrickData> > resultBricks;
  float longLength, shortLength;

  if (lines.size() < 3)
    {
      return resultBricks;
    }

  lines = stable_sort(lines, not2(LineData::LengthLessThan()));

  DO_SHAPEVEC(lines, LineData, l1, {
    DO_SHAPENEXT(lines, LineData, l1, l2, {
    if (l1->getLength() > l2->getLength()){
      longLength = l1->getLength();
      shortLength = l2->getLength();
    }
    else {
      longLength = l2->getLength();
      shortLength = l1->getLength();
    }
    if (LineData::ParallelTest()(l1,l2) && !LineData::ColinearTest()(l1,l2) && 
	(shortLength / longLength > lengthConst)) {
      DO_SHAPENEXT(lines, LineData, l2, l3, {
	if (LineData::ParallelTest()(l1,l3) && 
	    !LineData::ColinearTest()(l1,l3) &&
	    LineData::ParallelTest()(l2,l3) && 
	    !LineData::ColinearTest()(l2,l3) &&
	    (l3->getLength() / longLength > lengthConst) && 
	    (shortLength / l3->getLength() > lengthConst)) {
	  NEW_SHAPE_N(l1_2, LineData,  new LineData(ShS, l1->leftPt(), l1->rightPt()));
	  NEW_SHAPE_N(l2_2, LineData, new LineData(ShS, l2->leftPt(), l2->rightPt()));
	  NEW_SHAPE_N(l3_2, LineData, new LineData(ShS, l3->leftPt(), l3->rightPt()));
	  l1_2->setParentId(l1->getViewableId());
	  l2_2->setParentId(l1->getViewableId());
	  l3_2->setParentId(l1->getViewableId());
	  resultLines.push_back(l1_2);
	  resultLines.push_back(l2_2);
	  resultLines.push_back(l3_2);
	}
      });
    }
    });
  });
	      
  if (resultLines.size() < 3) {
      return resultBricks;
  }
  
  for (unsigned int i=0; i<resultLines.size(); i+=3)
    {
      const Shape<LineData>& final1 = ShapeRootTypeConst(resultLines[i+0],LineData);
      const Shape<LineData>& final2 = ShapeRootTypeConst(resultLines[i+1],LineData);
      const Shape<LineData>& final3 = ShapeRootTypeConst(resultLines[i+2],LineData);
      /*cout<<"3 lines:"<<endl;
	final1->printEnds();
	final2->printEnds();
	final3->printEnds();*/
      
      std::vector<Shape<LineData> > threeLines;
      
      if (final1->bottomPt().isBelow(final2->bottomPt(),camcentric))
	{
	  if (final1->bottomPt().isBelow(final3->bottomPt(),camcentric))
	    {
	      threeLines.push_back(final1);
	      if (final2->bottomPt().isBelow(final3->bottomPt(),camcentric))
		{
		  threeLines.push_back(final2);
		  threeLines.push_back(final3);
		}
	      else
		{
		  threeLines.push_back(final3);
		  threeLines.push_back(final3);
		}
	    }
	  else
	    {
	      threeLines.push_back(final3);
	      threeLines.push_back(final1);
	      threeLines.push_back(final2);
	    }
	}
      else
	{
	  if (final2->bottomPt().isBelow(final3->bottomPt(),camcentric))
	    {
	      threeLines.push_back(final2);
	      if (final1->bottomPt().isBelow(final3->bottomPt(),camcentric))
		{
		  threeLines.push_back(final1);
		  threeLines.push_back(final3);
		}
	      else
		{
		  threeLines.push_back(final3);
		  threeLines.push_back(final1);
		}
	    }
	  else
	    {
	      threeLines.push_back(final3);
	      threeLines.push_back(final2);
	      threeLines.push_back(final1);
	    }
	}
      const Shape<LineData> &bottom = ShapeRootTypeConst(threeLines[0], LineData);
      const Shape<LineData> &mid = ShapeRootTypeConst(threeLines[1], LineData);
      const Shape<LineData> &top = ShapeRootTypeConst(threeLines[2], LineData);
      
      /*cout<<"Sorted Lines: "<<endl;
	bottom->printEnds();
	mid->printEnds();
	top->printEnds();*/
      
      Point gbl(top->leftPt()+(bottom->leftPt() - mid->leftPt()));
      Point gbr(top->rightPt()+(bottom->rightPt() - mid->rightPt()));
      Shape<BrickData> newBrick (ShS, (const Point&)(bottom->leftPt()), (const Point&)bottom->rightPt(), 
				 gbl, gbr, 
				 (const Point&)mid->leftPt(), (const Point&)mid->rightPt(), 
				 (const Point&)top->leftPt(), (const Point&)top->rightPt());
      
      newBrick->setParentId(final1->getViewableId());

      NEW_SHAPE(brickl1, LineData, Shape<LineData>(bottom.getData()));
      NEW_SHAPE(brickl2, LineData, Shape<LineData>(mid.getData()));
      NEW_SHAPE(brickl3, LineData, Shape<LineData>(top.getData()));

      brickl1->setParentId(newBrick->getViewableId());
      brickl2->setParentId(newBrick->getViewableId());
      brickl3->setParentId(newBrick->getViewableId());
      
      resultBricks.push_back(newBrick);
    }
  
  return resultBricks;
}


// Find bricks from 3 vectors of candidate blobs
// Each vector is blobs of a different face color
//
// Also an old version. Final version is in extractBrick
std::vector<Shape<BrickData> > BrickData::findBricksFromBlobs(ShapeSpace& ShS, 
							      std::vector<Shape<BlobData> > blobs1,
							      std::vector<Shape<BlobData> > blobs2,
							      std::vector<Shape<BlobData> > blobs3) 
{

  const float distanceThresh = 10;

  unsigned int i, i1, i2;
  Shape<BlobData> blob1, blob2, blob3;

  std::vector<Shape<BrickData> > resultBricks;

  Point GFL, GFR, GBL, GBR, TFL, TFR, TBL, TBR;
  Point c12,c13,c23;

  std::vector<std::vector<Point> > corners1;
  std::vector<std::vector<Point> > corners2;
  std::vector<std::vector<Point> > corners3;
  std::vector<bool> used1;
  std::vector<bool> used2;
  std::vector<bool> used3;
  for (i=0; i<blobs1.size(); i++) { 
    used1.push_back(false); 
    corners1.push_back(blobs1[i]->findCornersDiagonal()); 
  }
  for (i=0; i<blobs2.size(); i++) { 
    used2.push_back(false); 
    corners2.push_back(blobs2[i]->findCornersDiagonal()); 
  }
  for (i=0; i<blobs3.size(); i++) { 
    used3.push_back(false); 
    corners3.push_back(blobs3[i]->findCornersDiagonal()); 
  }

  // Look for bricks with a good common edge between each pair of viable brick face colors
  int used;
  for (i1=0; i1<blobs1.size(); i1++){
    for (i2=0; i2<blobs2.size();i2++){
      if (!used1[i1] && !used2[i2]) {
	used = addBrickWithTwoSides(ShS, corners1[i1], corners2[i2], corners3, 
				    resultBricks, distanceThresh);
	if (used>=0) {
	  used1[i1] = true;
	  used2[i2] = true;
	  used3[used] = true;
	  resultBricks[resultBricks.size()-1]->setParentId(blobs1[i1]->getViewableId());
	}
      }
    }
  }

  for (i1=0; i1<blobs1.size(); i1++){
    for (i2=0; i2<blobs3.size();i2++){
      if (!used1[i1] && !used3[i2]) {
	used = addBrickWithTwoSides(ShS, corners1[i1], corners3[i2], corners2, 
				    resultBricks, distanceThresh);
	if (used>=0) {
	  used1[i1] = true;
	  used3[i2] = true;
	  used2[used] = true;
	  resultBricks[resultBricks.size()-1]->setParentId(blobs1[i1]->getViewableId());
	}
      }
    }
  }

  for (i1=0; i1<blobs3.size(); i1++){
    for (i2=0; i2<blobs2.size();i2++){
      if (!used3[i1] && !used2[i2]) {
	used = addBrickWithTwoSides(ShS, corners3[i1], corners2[i2], corners1, 
				    resultBricks, distanceThresh);
	if (used>=0) {
	  used3[i1] = true;
	  used2[i2] = true;
	  used1[used] = true;
	  resultBricks[resultBricks.size()-1]->setParentId(blobs3[i1]->getViewableId());
	}
      }
    }
  }


  return resultBricks;
}


// Subroutine for blob brick detection
// If the two sides match, finds the third side that matches
// Extrapolates all the points and adds the brick to the vector
// returns the index of the third face, or -1 if no match is found
//
// subroutine for an old version
int BrickData::addBrickWithTwoSides(ShapeSpace &ShS,
				    std::vector<Point>& corners1, 
				    std::vector<Point>& corners2, 
				    std::vector<std::vector<Point> >& blobs3, 
				    std::vector<Shape<BrickData> >& result, 
				    float distanceThresh)
{
  unsigned int blobi;
  int i1=-1,j1=-1, i2=-1, j2=-1;
  for (int i=0; i<4 && i2<0; i++) {
    for (int j=0; j<4 && j2<0; j++) {
      if (corners1[i].distanceFrom(corners2[j]) < distanceThresh) {
	if (corners1[(i+1)%4].distanceFrom(corners2[(j+3)%4]) < distanceThresh) {
	  i1 = i;
	  i2 = (i+1)%4;
	  j1 = (j+3)%4;
	  j2 = j;
	}
	else if (corners1[(i+3)%4].distanceFrom(corners2[(j+1)%4]) < distanceThresh) {
	  i1 = (i+3)%4;
	  i2 = i;
	  j1 = j;
	  j2 = (j+1)%4;
	}
	if (i2>=0) {
	  // Two matching corners have been found: (i1,j2), (i2, j1)
	  // Look for the third side
	  bool found = false;
	  Point center, mid1, mid2, mid3, c1, c2, c3;
	  for (blobi=0; blobi<blobs3.size() && !found; blobi++) {
	    for(int k=0; k<4 && !found; k++) {
	      if (((blobs3[blobi][k].distanceFrom(corners1[i1]) < distanceThresh) +
		   (blobs3[blobi][(k+1)%4].distanceFrom(corners1[(i1+3)%4]) < distanceThresh) +
		   (blobs3[blobi][(k+3)%4].distanceFrom(corners2[(j2+1)%4]) < distanceThresh)) > 1) {
		// (i1,j2, k) is the center
		found = true;
		mid1 = (corners1[i2]+corners2[j1])/2;
		if (blobs3[blobi][k].distanceFrom(corners1[i1]) < distanceThresh) 
		  center = (corners1[i1]+corners2[j2]+blobs3[blobi][k])/3;
		else
		  center = (corners1[i1]+corners2[j2])/2;
		if (blobs3[blobi][(k+1)%4].distanceFrom(corners1[(i1+3)%4]) < distanceThresh)
		  mid2 = (blobs3[blobi][(k+1)%4] + corners1[(i1+3)%4])/2;
		else
		  mid2 = corners1[(i1+3)%4];
		if (blobs3[blobi][(k+3)%4].distanceFrom(corners2[(j2+1)%4]) < distanceThresh)
		  mid3 = (blobs3[blobi][(k+3)%4] + corners2[(j2+1)%4])/2;
		else
		  mid3 = corners2[(j2+1)%4];
		c1 = corners1[(i1+2)%4];
		c2 = blobs3[blobi][(k+2)%4];
		c3 = corners2[(j2+2)%4];
	      }
	      else if (((blobs3[blobi][k].distanceFrom(corners1[i2]) < distanceThresh) +
		   (blobs3[blobi][(k+1)%4].distanceFrom(corners2[(j1+3)%4]) < distanceThresh) +
		   (blobs3[blobi][(k+3)%4].distanceFrom(corners1[(i2+1)%4]) < distanceThresh)) > 1) {
		// (i2, j1, k) is the center
		found = true;
		mid1 = (corners1[i1]+corners2[j2])/2;
		if (blobs3[blobi][k].distanceFrom(corners1[i2]) < distanceThresh) 
		  center = (corners1[i2]+corners2[j1]+blobs3[blobi][k])/3;
		else
		  center = (corners1[i2]+corners2[j1])/2;
		if (blobs3[blobi][(k+1)%4].distanceFrom(corners2[(j1+3)%4]) < distanceThresh)
		  mid2 = (blobs3[blobi][(k+1)%4] + corners2[(j1+3)%4])/2;
		else
		  mid2 = corners2[(j1+3)%4];
		if (blobs3[blobi][(k+3)%4].distanceFrom(corners1[(i2+1)%4]) < distanceThresh)
		  mid3 = (blobs3[blobi][(k+3)%4] + corners1[(i2+1)%4])/2;
		else
		  mid3 = corners1[(i2+1)%4];
		c1 = corners2[(j1+2)%4];
		c2 = blobs3[blobi][(k+2)%4];
		c3 = corners1[(i2+2)%4];
	      }
	    }

	  }

	  if (found) {
	    // Found a brick, figure out where the top / left / etc sides are

	    // Debug shapes
	    NEW_SHAPE(centerp, PointData, new PointData(ShS, center));
	    NEW_SHAPE(mid1p, PointData, new PointData(ShS, mid1));
	    NEW_SHAPE(mid2p, PointData, new PointData(ShS, mid2));
	    NEW_SHAPE(mid3p, PointData, new PointData(ShS, mid3));
	    NEW_SHAPE(c1p, PointData, new PointData(ShS, c1));
	    NEW_SHAPE(c2p, PointData, new PointData(ShS, c2));
	    NEW_SHAPE(c3p, PointData, new PointData(ShS, c3));
	    centerp->setColor(rgb(150,0,255));
	    mid1p->setColor(rgb(150,0,255));
	    mid2p->setColor(rgb(150,0,255));
	    mid3p->setColor(rgb(150,0,255));
	    c1p->setColor(rgb(150,0,255));
	    c2p->setColor(rgb(150,0,255));
	    c3p->setColor(rgb(150,0,255));
	    
	    Point GFL, GFR, GBL, GBR, TFL, TFR, TBL, TBR;
	    
	    TFR = center;
	    if (mid1.isBelow(center, camcentric) && mid1.isBelow(mid2, camcentric) && mid1.isBelow(mid3, camcentric)) {
	      GFR = mid1;
	      GBR = c1;
	      if (mid2.isRightOf(mid3, camcentric)) {
		TBR = mid2;
		TBL = c2;
		TFL = mid3;
		GFL = c3;
	      }
	      else {
		TBR = mid3;
		TBL = c3;
		TFL = mid2;
		GFL = c2;
	      }
	    }
	    else if (mid2.isBelow(center, camcentric) && mid2.isBelow(mid1, camcentric) && mid2.isBelow(mid3, camcentric)) {
	      GFR = mid2;
	      GBR = c2;
	      if (mid1.isRightOf(mid3, camcentric)) {
		TBR = mid1;
		TBL = c1;
		TFL = mid3;
		GFL = c3;
	      }
	      else {
		TBR = mid3;
		TBL = c3;
		TFL = mid1;
		GFL = c1;
	      }
	    }
	    else {
	      GFR = mid3;
	      GBR = c3;
	      if (mid2.isRightOf(mid1, camcentric)) {
		TBR = mid2;
		TBL = c2;
		TFL = mid1;
		GFL = c1;
	      }
	      else {
		TBR = mid1;
		TBL = c1;
		TFL = mid2;
		GFL = c2;
	      }
	    }
	    
	    GBL = GFL + (TBL - TFL);

	    Shape<BrickData> newBrick(ShS, GFL, GFR, GBL, GBR, TFL, TFR, TBL, TBR);
	    result.push_back(newBrick);
	    centerp->setParentId(newBrick->getViewableId());
	    mid1p->setParentId(newBrick->getViewableId());
	    mid2p->setParentId(newBrick->getViewableId());
	    mid3p->setParentId(newBrick->getViewableId());
	    c1p->setParentId(newBrick->getViewableId());
	    c2p->setParentId(newBrick->getViewableId());
	    c3p->setParentId(newBrick->getViewableId());
	    return blobi;
	  }
	  else {
	    // What do we do if we get two good faces and no third?
	  }
	  
	}
      }
    }
  }

  return -1;

}





/* Unified brick extraction starting from 2 or 3 blobs, each of a different color 
 *
 *************    Final Version   ***************
 *
 * Checks the relative locations of the blobs (to ensure they are adjacent enough to 
 * be part of the same brick)
 *
 * Computes the interior edge lines from the regions close to two faces.
 *
 * Computes the corners of each face individually. 
 * This step is handled in blobData::findCorners
 *
 * Combine all the corner guesses together and extrapolate missing corners.
 *
 *
 * Some helper functions can be found in BrickOps.h 
 */
Shape<BrickData> BrickData::extractBrick(ShapeSpace& space, vector<Shape<BlobData> > &blobs)
{
  unsigned int nblobs = blobs.size();
  std::vector<Point> centroids;

  ShapeRoot invalid;
  if (nblobs > 3 || nblobs < 2) {
    return ShapeRootType(invalid,BrickData);
  }

  for (unsigned int i=0; i<nblobs; i++) {
    centroids.push_back(blobs[i]->getCentroid());
  }

  // Check inter-blob distance
  // If any pair of blobs are too far apart, this set is invalid
  for (unsigned int i=0; i<nblobs; i++) {
    for (unsigned int j=i+1; j<nblobs; j++) {
      if (centroids[i].distanceFrom(centroids[j]) >= 
	  blobs[i]->topLeft.distanceFrom(centroids[i]) + 
	  blobs[j]->topLeft.distanceFrom(centroids[j])){
	return ShapeRootType(invalid,BrickData);	
      }
    }
  }

	for(unsigned i = 0; i < centroids.size(); i++)
		centroids[i].refFrameType = blobs.front()->getRefFrameType();


  // arbitrary face assignment, aligned as the brick is in camera space
  // this will probably break down if the dog tilts its head
  // 
  // if we only have two faces, we're assuming only the top and "left" faces are visible
  // always assume a top face is visible, becuase any shape without a top face is much more likely
  // to be a pyramid than a tall brick
  int top=-1, left=-1, right=-1;
  
  if (centroids[0].isAbove(centroids[1])) {
    top = 0;
  }
  else {
    top = 1;
  }
  if (nblobs > 2 && centroids[2].isAbove(centroids[top])) {
    top = 2;
  }

  if ((top != 0 && centroids[0].isLeftOf(centroids[1])) || top == 1) {
    left = 0;
  }
  else {
    left = 1;
  }
  if (nblobs>2 && top != 2 && centroids[2].isLeftOf(centroids[left])) {
    left = 2;
  }

  if (nblobs > 2) {
    if (top != 0 && left != 0) {
      right = 0;
    }
    else if (top != 1 && left != 1) {
      right = 1;
    }
    else {
      right = 2;
    }
  }

  if (top == left || top == right || left == right){
    std::cout<<"ERROR: Brick side misclassification!"<<std::endl;
    return ShapeRootType(invalid,BrickData);	
  }


  // Compute two-blob boundary regions
  // This is used to find the interior edges of the brick
  // It is also used as a second check for inter-brick distance
  const int INTERSECT_DIST = 5, MIN_REQUIRED_DIST = 2;
  std::vector<Sketch<bool> > boundaries;
  NEW_SKETCH_N(topEdist, uchar, visops::mdist(blobs[top]->getRendering()));
  NEW_SKETCH_N(leftEdist, uchar, visops::mdist(blobs[left]->getRendering()));
  NEW_SKETCH(tlsmall, bool, (topEdist<MIN_REQUIRED_DIST) & (leftEdist<MIN_REQUIRED_DIST));
  if (!tlsmall->max()){
    return ShapeRootType(invalid,BrickData);	
  }
  NEW_SKETCH(tl, bool, (topEdist<INTERSECT_DIST) & (leftEdist<INTERSECT_DIST));
  rgb green_rgb(0,0,255);
  tl->setColor(green_rgb);
  boundaries.push_back(tl);

  if (nblobs>2) {
    NEW_SKETCH_N(rightEdist, uchar, visops::mdist(blobs[right]->getRendering()));
    NEW_SKETCH(trsmall, bool, (topEdist<MIN_REQUIRED_DIST) & (rightEdist<MIN_REQUIRED_DIST));
    if (!trsmall->max()){
      return ShapeRootType(invalid,BrickData);
    }	
    NEW_SKETCH(tr, bool, (topEdist<INTERSECT_DIST) & (rightEdist<INTERSECT_DIST));
    tr->setColor(green_rgb);
    boundaries.push_back(tr);

    NEW_SKETCH(lrsmall, bool, (leftEdist<MIN_REQUIRED_DIST) & (rightEdist<MIN_REQUIRED_DIST));
    if (!lrsmall->max()){
      return ShapeRootType(invalid,BrickData);
    }	
    NEW_SKETCH(lr, bool, (leftEdist<INTERSECT_DIST) & (rightEdist<INTERSECT_DIST));
    lr->setColor(green_rgb);
    boundaries.push_back(lr);
  }



  // Begin gathering evidence for each point
  // This should probably be augmented by an extra vector of confidences
  std::vector<std::vector<Point> > points(8);

  // Arbitrary corner / face assignemt (in cam space)
  //
  //      5------6
  //     /   T  /|
  //    /(4)   / 7
  //   1------2 / <-- R
  //   |  L   |/
  //   0------3
  //    


  // Construct the interior lines
  // Add their guesses to the appropriate points on the brick
  NEW_SHAPE(tlline, LineData, LineData::extractLine(boundaries[0]));
  Shape<LineData> trline, lrline;

  if (right!=-1) {
    trline = LineData::extractLine(boundaries[1]);
    lrline = LineData::extractLine(boundaries[2]);

    // shorten interior lines based on their intersections
    bool on1=false, on2=false;
    if (tlline.isValid() && trline.isValid()) {
      Point isectPt = tlline->intersectionWithLine(trline, on1, on2);
      if (on1) {
	tlline->rightPt().setCoords(isectPt);
      }
      if (on2) {
	trline->leftPt().setCoords(isectPt);
      }
    }

    if (tlline.isValid() && lrline.isValid()) {
      Point isectPt = tlline->intersectionWithLine(lrline, on1, on2);
      if (on1) {
	tlline->rightPt().setCoords(isectPt);
      }
      if (on2) {
	lrline->topPt().setCoords(isectPt);
      }
    }

    if (trline.isValid() && lrline.isValid()) {
      Point isectPt = trline->intersectionWithLine(lrline, on1, on2);
      if (on1) {
	trline->leftPt().setCoords(isectPt);
      }
      if (on2) {
	lrline->topPt().setCoords(isectPt);
      }
    }


    if (trline.isValid()) {
      points[2].push_back(trline->leftPt());
      points[6].push_back(trline->rightPt());
    }

    if (lrline.isValid()) {
      points[2].push_back(lrline->topPt());
      points[3].push_back(lrline->bottomPt());
    }
  }

  if (tlline.isValid()) {
    points[1].push_back(tlline->leftPt());
    points[2].push_back(tlline->rightPt());
  }



  // Extract the corners of the blob using the derivative approach
  //
  // corners are coming out of findCorners() in counter-clock-wise order around the blob
  //  with unknown starting position
  //
  // the findCorners function in BlobData is the other important part of the algorithm
  // Several different methods of corner extraction can be connected there. 
  std::vector<Point> candidates;

  // top face

  // Compute an orthogonal bounding box from the top-left line
  if (tlline.isValid()) {
    if (trline.isValid() && trline->getLength() > tlline->getLength()) {
      candidates = findOrthogonalBoundingBox(space, blobs[top], centroids[top], trline);
    }
    else {
      candidates = findOrthogonalBoundingBox(space, blobs[top], centroids[top], tlline);      
    }
  }
  else if (trline.isValid()) {
    candidates = findOrthogonalBoundingBox(space, blobs[top], centroids[top], trline);
  }
	
	for(unsigned i = 0; i < candidates.size(); i++)
		candidates[i].refFrameType = blobs.front()->getRefFrameType();
  
  float cornerValue; 

  std::vector<Point> tcorners = blobs[top]->findCorners(4, candidates, cornerValue);

	for(unsigned i = 0; i < tcorners.size(); i++)
		tcorners[i].refFrameType = blobs.front()->getRefFrameType();
	
  if (tcorners.size() == 4) {
    
    // Sort out which corner is which
    // if there's a clear left-most corner, its corner 1
    // if two corners are close in left-ness, then they should be 1 and 5
    unsigned int leftCorner = 0;
    for (unsigned int i=1; i<4; i++){
      if (tcorners[i].isLeftOf(tcorners[leftCorner])) {
	leftCorner = i;
      }
    }
    int closeCorner = -1;
    float mostLeft = tcorners[leftCorner].coordX();
    const int MAX_CLOSE_DIST = 5;
    for (unsigned int i=0; i<4; i++) {
      if (i != leftCorner && tcorners[i].coordX() - mostLeft < MAX_CLOSE_DIST) {
	closeCorner = i;
      }
    }
    
    if (closeCorner != -1) {
      // If 5 was actually left-most, switch leftCorner to 1
      if ((unsigned int)closeCorner == (leftCorner+1)%4) {
	leftCorner = closeCorner;
      }
      else if ((unsigned int)closeCorner != (leftCorner+3)%4) {
	std::cout<<"WARNING, opposite corners are close together!"<<std::endl;
      }
    }
    NEW_SHAPE(top1, PointData, PointData(space, tcorners[leftCorner]));
    points[1].push_back(tcorners[leftCorner]);
    points[2].push_back(tcorners[(leftCorner+1)%4]);
    points[6].push_back(tcorners[(leftCorner+2)%4]);
    points[5].push_back(tcorners[(leftCorner+3)%4]);

  }
  else {
    // Some versions of findCorners don't always return 4 corners. 
    // How to handle these cases is ambiguous at best. 
  }

  // repeat with other faces

  // left face
  candidates.clear();
  if (tlline.isValid()) {
    if (lrline.isValid() && lrline->getLength() > tlline->getLength()) {
      candidates = findOrthogonalBoundingBox(space, blobs[left], centroids[left], lrline);
    }
    else {
      candidates = findOrthogonalBoundingBox(space, blobs[left], centroids[left], tlline);      
    }
  }
  else if (trline.isValid()) {
    candidates = findOrthogonalBoundingBox(space, blobs[left], centroids[left], lrline);
  }

  for (unsigned int i=0; i<candidates.size(); i++) {
    NEW_SHAPE(candidate, PointData, PointData(space, candidates[i]));
    candidate->setParentId(blobs[left]->getViewableId());
  }
  std::vector<Point> lcorners = blobs[left]->findCorners(4, candidates, cornerValue);
	for(unsigned i = 0; i < lcorners.size(); i++)
		lcorners[i].refFrameType = blobs.front()->getRefFrameType();
	
  if (lcorners.size() == 4) {
    
    // if there's a clear right-most corner, its corner 3
    // if two corners are close in right-ness, then they should 3 and 2
    unsigned int rightCorner = 0;
    for (unsigned int i=1; i<4; i++){
      if (lcorners[i].isRightOf(lcorners[rightCorner])) {
	rightCorner = i;
      }
    }
    int closeCorner = -1;
    float mostRight = lcorners[rightCorner].coordX();
    const int MAX_CLOSE_DIST = 5;
    for (unsigned int i=0; i<4; i++) {
      if (i != rightCorner && mostRight - lcorners[i].coordX() < MAX_CLOSE_DIST) {
	closeCorner = i;
      }
    }
    
    if (closeCorner != -1) {
      // If 2 was actually right-most, switch rightCorner to 3
      if ((unsigned int)closeCorner == (rightCorner+3)%4) {
	rightCorner = closeCorner;
      }
      else if ((unsigned int)closeCorner != (rightCorner+1)%4) {
	std::cout<<"WARNING, opposite corners are close together!"<<std::endl;
      }
    }
    NEW_SHAPE(left2, PointData, PointData(space, lcorners[rightCorner]));
    points[3].push_back(lcorners[rightCorner]);
    points[2].push_back(lcorners[(rightCorner+1)%4]);
    points[1].push_back(lcorners[(rightCorner+2)%4]);
    points[0].push_back(lcorners[(rightCorner+3)%4]);

  }
  else {

  }

  // right face
  if (right != -1) {

    candidates.clear();
    if (trline.isValid()) {
      if (lrline.isValid() && lrline->getLength() > trline->getLength()) {
	candidates = findOrthogonalBoundingBox(space, blobs[right], centroids[right], lrline);
      }
      else {
	candidates = findOrthogonalBoundingBox(space, blobs[right], centroids[right], trline);      
      }
    }
    else if (lrline.isValid()) {
      candidates = findOrthogonalBoundingBox(space, blobs[right], centroids[right], lrline);
    }

    std::vector<Point> rcorners = blobs[right]->findCorners(4, candidates, cornerValue);
	  
	  for(unsigned i = 0; i < rcorners.size(); i++)
		  rcorners[i].refFrameType = blobs.front()->getRefFrameType();
	  
	  
    if (rcorners.size() == 4) {
    
      // if there's a clear bottom-most corner, its corner 3
      // if two corners are close in bottom-ness, then they should 3 and 7
      unsigned int bottomCorner = 0;
      for (unsigned int i=1; i<4; i++){
	if (rcorners[i].isBelow(rcorners[bottomCorner])) {
	  bottomCorner = i;
	}
      }
      int closeCorner = -1;
      float mostBottom = rcorners[bottomCorner].coordY();
      const int MAX_CLOSE_DIST = 5;
      for (unsigned int i=0; i<4; i++) {
	if (i != bottomCorner && mostBottom - rcorners[i].coordY() < MAX_CLOSE_DIST) {
	  closeCorner = i;
	}
      }
    
      if (closeCorner != -1) {
	// If 7 was actually bottom-most, switch bottomCorner to 3
	if ((unsigned int)closeCorner == (bottomCorner+3)%4) {
	  bottomCorner = closeCorner;
	}
	else if ((unsigned int)closeCorner != (bottomCorner+1)%4) {
	  std::cout<<"WARNING, opposite corners are close together!"<<std::endl;
	}
      }
      NEW_SHAPE(right3, PointData, PointData(space,rcorners[bottomCorner]));
      points[3].push_back(rcorners[bottomCorner]);
      points[7].push_back(rcorners[(bottomCorner+1)%4]);
      points[6].push_back(rcorners[(bottomCorner+2)%4]);
      points[2].push_back(rcorners[(bottomCorner+3)%4]);

    }
  }


  // If we have additional ways of extracting candidate corners, 
  // add the corners they find to the vector of corners now.


  // debug output
  /*for (unsigned int i=0; i<8; i++){
    for (unsigned int j=0; j<points[i].size(); j++) {
      NEW_SHAPE(corner, PointData, PointData(space, points[i][j]));
      char name[10];
      sprintf(name, "corner%d%d",i,j);
      corner->setName(name);
    }
    }*/


  // Now merge all our candidate points to get the final brick

  vector<Point> finalCorners(8);

  // for now just average corners together to get the last corners
  // should at least weight the average based on confidence
  // could also do statistics
  // produce a final confidence based on the std. deviation

  // if we didn't get the center point of the brick then we don't have a shot
  // top-left line also is pretty critical, 
  // though I probably should re-work below to work in the case where top and right only work well
  if (points[2].size()==0 || points[1].size()==0){
    return ShapeRootType(invalid,BrickData);    
  }

  Point empty(0,0);


  // Lots of cases for which corners are visible and extrapolating the others

  for (unsigned int i=0; i<8; i++) {
    finalCorners[i] = empty;
    for (unsigned int j=0; j<points[i].size(); j++) {
      finalCorners[i]+=points[i][j];
    }
    if (points[i].size() > 0) {
      finalCorners[i]/=points[i].size();
    }
  }

  if (points[3].size()==0){
    // These should be easier cases
    return ShapeRootType(invalid,BrickData);        
  }

  if (points[6].size() == 0) {
    // If we have the top left line but not corner 6, infer corner 6 from the thickness of the top side
    if (tlline.isValid()) {
      NEW_SKETCH_N(topRendering, bool, blobs[top]->getRendering());
      Point topPt = (Region::extractRegion(topRendering)).mostDistantPtFrom(tlline.getData());
      NEW_SHAPE(intersectLine, LineData, LineData(space, topPt, tlline->getThetaNorm()));
      Point intersectPoint = intersectLine->intersectionWithLine(tlline);
      // lower confidence result
      finalCorners[6] = topPt+finalCorners[2]-intersectPoint;
    }
    // can also use right left line and the right side, but the top side is more likely to be good
    else {
      return ShapeRootType(invalid,BrickData);        
    }
 }

  if (points[0].size() == 0) {
    finalCorners[0] = finalCorners[3] + finalCorners[1]-finalCorners[2];
  }

  if (points[5].size() == 0) {
    finalCorners[5] = finalCorners[6] + finalCorners[1]-finalCorners[2];
  }

  if (points[7].size() == 0) {
    finalCorners[7] = finalCorners[3] + finalCorners[6]-finalCorners[2];
  }

  finalCorners[4] = finalCorners[5] + finalCorners[7] - finalCorners[6] +
    finalCorners[5] + finalCorners[0] - finalCorners[1] +
    finalCorners[0] + finalCorners[7] - finalCorners[3];
  finalCorners[4]/=3.f;

  // left == front for generalized brick
  NEW_SHAPE(returnbrick, BrickData, new BrickData(space, 
						  finalCorners[0], finalCorners[3],
						  finalCorners[4], finalCorners[7],
						  finalCorners[1], finalCorners[2],
						  finalCorners[5], finalCorners[6]));
  return returnbrick;
    
}



  // Generates a wide bounding box around the blob from a parallel line along one edge
  // This bounding box will be used as a starting point for the quadrilateral-fitting algorithm
  // 
  // Generates 4 points based on the line, centroid of the blob, and parameters for extending the bounds
  // Then sorts the points into counter-clockwise order to satisfy the ordering constraints to make other 
  // extrapolations possible. 
vector<Point> BrickData::findOrthogonalBoundingBox(ShapeSpace& space, Shape<BlobData> blob, Point centroid, Shape<LineData> parallel)
{
  const float PARALLEL_EXTEND_FACTOR = .6f, ORTHO_EXTEND_FACTOR = .6f;

  vector<Point> candidates;
  LineData candidate1(space, parallel->end1Pt(), parallel->end2Pt());

  //candidate1.setEndPts(parallel->end1Pt(), parallel->end2Pt());
  Point parallelExtend(candidate1.end2Pt() - candidate1.end1Pt());
  parallelExtend *= PARALLEL_EXTEND_FACTOR;
  LineData ortho(space, centroid, candidate1.getThetaNorm());
  Point orthoIsect = candidate1.intersectionWithLine(ortho);
  Point candidate2Offset = (centroid - orthoIsect) * 2.f;
  Point orthoExtend = candidate2Offset * ORTHO_EXTEND_FACTOR;

  LineData candidate2(space, candidate1.end1Pt() + candidate2Offset + orthoExtend - parallelExtend,
		      candidate1.end2Pt() + candidate2Offset + orthoExtend + parallelExtend);
  candidate1.setEndPts(candidate1.end1Pt() - orthoExtend - parallelExtend,
		       candidate1.end2Pt() - orthoExtend + parallelExtend);
  candidates.push_back(candidate1.leftPt());
  candidates.push_back(candidate1.rightPt());
  candidates.push_back(candidate2.rightPt());
  candidates.push_back(candidate2.leftPt());

  // debug output
  for (unsigned int i=0; i<candidates.size(); i++) {
    NEW_SHAPE_N(candidate, PointData, PointData(space, candidates[i]));
    candidate->setParentId(blob->getViewableId());
  }

  // order counter-clockwise around the blob before returning
  Point centerPoint = (candidates[0] + candidates[1] + candidates[2] + candidates[3]) / 4.f;
  vector<float> centerAngles;
  for (unsigned int i=0; i<candidates.size(); i++) {
    NEW_SHAPE_N(centerLine, LineData, LineData(space, centerPoint, candidates[i]));
    centerLine->setParentId(blob->getViewableId());
    centerAngles.push_back(atan2(centerLine->end2Pt().coordY() - centerLine->end1Pt().coordY(),
				 centerLine->end2Pt().coordX() - centerLine->end1Pt().coordX()));
  }


  vector<Point> orderedPoints;
  
  int maxi = 0, mini = 0;
  float maxAng = centerAngles[0];
  float minAng = maxAng;
  
  for (unsigned int i=1; i<candidates.size(); i++) {
    if (centerAngles[i] > maxAng) {
      maxAng = centerAngles[i];
      maxi = i;
    }
    if (centerAngles[i] < minAng) {
      minAng = centerAngles[i];
      mini = i;
    }
       
  }

  orderedPoints.push_back(candidates[maxi]);
  
  float lastMax = maxAng;
  for (unsigned int i=1; i<candidates.size(); i++) {
    maxi = mini;
    maxAng = minAng;
    for (unsigned int j=0; j<candidates.size(); j++) {
      float curAng = centerAngles[j];
      if (curAng > maxAng && curAng < lastMax) {
	maxi = j;
	maxAng = curAng;
      }
    }
    orderedPoints.push_back(candidates[maxi]);
    lastMax = maxAng;
  }
  
  // debug output
  for (unsigned int i=0; i<candidates.size(); i++) {
    NEW_SHAPE(orderedcandidate, PointData, PointData(space, orderedPoints[i]));
    orderedcandidate->setParentId(blob->getViewableId());
  }

  return orderedPoints;
}



} // namespace
