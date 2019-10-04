//-*-c++-*-
#include <iostream>
#include <vector>

#include "Vision/cmvision.h"

#include "SketchSpace.h"
#include "Sketch.h"
#include "ShapeSpace.h"
#include "ShapeRoot.h"

#include "BlobData.h"
#include "LineData.h"  // for drawline2d
#include "ShapeBlob.h"
#include "visops.h"
#include "Region.h"
#include "ShapeLine.h"
#include "ShapePoint.h"

#include "BrickOps.h" 

using namespace std;

namespace DualCoding {

BlobData::BlobData(ShapeSpace& _space,
		   const Point &_topLeft, const Point &_topRight,
		   const Point &_bottomLeft, const Point &_bottomRight,
		   const float _area, const std::vector<run> &_runvec,
		   const BlobOrientation_t _orientation, coordinate_t _assumedHeight,
		   rgb _rgbvalue,
		   bool _topValid, bool _bottomValid, bool _leftValid, bool _rightValid) :
  BaseData(_space, getStaticType()),
  topValid(_topValid), bottomValid(_bottomValid), leftValid(_leftValid), rightValid(_rightValid),
  orientation(_orientation), assumedHeight(_assumedHeight),
  topLeft(_topLeft), topRight(_topRight),
  bottomLeft(_bottomLeft), bottomRight(_bottomRight),
  area(_area), runvec(_runvec)
{
  setColor(_rgbvalue);
}
      
DATASTUFF_CC(BlobData);

//! return the centroid of the shape in point format
Point BlobData::getCentroid() const {
  return Point((topLeft.coords + topRight.coords + bottomLeft.coords + bottomRight.coords) / 4,
	       getRefFrameType());
}
  
BoundingBox2D BlobData::getBoundingBox() const {
	// if we want to assume normal coordinates, could just do:
	// return BoundingBox2D(bottomLeft.coords,topRight.coords);
	BoundingBox2D result(topLeft.coords);
	result.expand(topRight.coords);
	result.expand(bottomLeft.coords);
	result.expand(bottomRight.coords);
	return result;
}

void BlobData::printParams() const {
  cout << "Type = " << getTypeName() << "  ID=" << getId() << "  ParentID=" << getParentId() << endl;
  printf("  color = %d %d %d\n",getColor().red,getColor().green,getColor().blue);
  cout << "  tl=" << topLeft.coords << endl
       << "  tr=" << topRight.coords << endl
       << "  bl=" << bottomLeft.coords << endl
       << "  br=" << bottomRight.coords << endl
       << "  area=" << area << ", assumedHeight=" << assumedHeight
       << endl;
}

Sketch<bool>* BlobData::render() const {
  SketchSpace &SkS = space->getDualSpace();
  Sketch<bool>* result = new Sketch<bool>(SkS, "render("+getName()+")");
  *result = false;
  switch ( orientation ) {
  case groundplane:
    if ( space->getRefFrameType() == camcentric ) {
      for ( std::vector<BlobData::run>::const_iterator it = runvec.begin();
	    it != runvec.end(); it++ ) {
	const BlobData::run &r = *it;
	int const xstop = r.x + r.width;
	for ( int xi=r.x; xi<xstop; xi++ )
	  (*result)(xi, r.y) = true;
      }
    } else {
      fmat::Column<3> tl(topLeft.getCoords()), tr(topRight.getCoords()),
	bl(bottomLeft.getCoords()), br(bottomRight.getCoords());
      SkS.applyTmat(tl); SkS.applyTmat(tr); SkS.applyTmat(bl); SkS.applyTmat(br);
      LineData::drawline2d(*result, (int)tl[0], (int)tl[1], (int)tr[0], (int)tr[1]);
      LineData::drawline2d(*result, (int)tr[0], (int)tr[1], (int)br[0], (int)br[1]);
      LineData::drawline2d(*result, (int)br[0], (int)br[1], (int)bl[0], (int)bl[1]);
      LineData::drawline2d(*result, (int)bl[0], (int)bl[1], (int)tl[0], (int)tl[1]);
    }
    break;
  case pillar:
  {
    const float semimajor = std::abs(topLeft.coordY() - topRight.coordY()) / 2;
    const float semiminor = semimajor;
    fmat::Column<3> ctr;
    ctr[0] = getCentroid().coordX() + semimajor;
    ctr[1] = getCentroid().coordY();
    ctr[2] = getCentroid().coordZ();
    SkS.applyTmat(ctr);
    const float &cx = ctr[0];
    const float &cy = ctr[1];
    const fmat::Transform &Tmat = SkS.getTmat();
    const float orient = M_PI / 2;
    fmat::Column<2> ori;
    ori[0] = cos(orient);
    ori[1] = sin(orient);
    fmat::Matrix<2,2> rot;
    rot(0,0) = Tmat(0,0);
    rot(0,1) = Tmat(0,1);
    rot(1,0) = Tmat(1,0);
    rot(1,1) = Tmat(1,1);
    ori = rot * ori;
    const float &cosT = ori[0];
    const float &sinT = ori[1];
    const float xRange = semimajor;  // don't scale this because cosT and sinT are scaled
    const float majorSq = xRange*xRange;
    const float mnrDevMjr = semiminor/semimajor;
    for (float xDist = -xRange; xDist <= xRange; xDist+=0.2f) {
      const float yRange = sqrt(max((float)0, majorSq - xDist*xDist)) * mnrDevMjr;
      for (float yDist = -yRange; yDist <= yRange; yDist+=0.2f) {
        int const px = round(cx+xDist*cosT-yDist*sinT);
        int const py = round(cy+yDist*cosT+xDist*sinT);
        if ( px >= 0 && px < result->width &&
	     py >= 0 && py < result->height )
	  (*result)(px,py) = true;
      }
    }
    }
    break;
  case poster:
  {
    const float semimajor = std::abs(topLeft.coordY() - topRight.coordY()) / 2;
    const float semiminor = std::abs(topLeft.coordX() - bottomLeft.coordX()) / 2;  //const float semiminor = 0;
    fmat::Column<3> ctr(getCentroid().getCoords());
    SkS.applyTmat(ctr);
    const float &cx = ctr[0];
    const float &cy = ctr[1];
    const fmat::Transform &Tmat = SkS.getTmat();
    const float orient = M_PI / 2;
    fmat::Column<2> ori;
    ori[0] = cos(orient);
    ori[1] = sin(orient);
    fmat::Matrix<2,2> rot;
    rot(0,0) = Tmat(0,0);
    rot(0,1) = Tmat(0,1);
    rot(1,0) = Tmat(1,0);
    rot(1,1) = Tmat(1,1);
    ori = rot * ori;
    const float &cosT = ori[0];
    const float &sinT = ori[1];
    const float xRange = semimajor;  // don't scale this because cosT and sinT are scaled
    const float majorSq = xRange*xRange;
    const float mnrDevMjr = semiminor/semimajor;
    for (float xDist = -xRange; xDist <= xRange; xDist+=0.2f) {
      const float yRange = sqrt(max((float)0, majorSq - xDist*xDist)) * mnrDevMjr;
      for (float yDist = -yRange; yDist <= yRange; yDist+=0.2f) {
        int const px = round(cx+xDist*cosT-yDist*sinT);
        int const py = round(cy+yDist*cosT+xDist*sinT);
        if ( px >= 0 && px < result->width &&
	     py >= 0 && py < result->height )
	  (*result)(px,py) = true;
      }
    }
    }
    break;
  }
  return result;
}

//! Transformations. (Virtual in BaseData.)
void BlobData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
  topLeft.applyTransform(Tmat,newref);
  topRight.applyTransform(Tmat,newref);
  bottomLeft.applyTransform(Tmat,newref);
  bottomRight.applyTransform(Tmat,newref);
}

void BlobData::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& gndplane) {
  switch ( orientation ) {
  case groundplane:
    topLeft.projectToGround(camToBase,gndplane);
    topRight.projectToGround(camToBase,gndplane);
    bottomLeft.projectToGround(camToBase,gndplane);
    bottomRight.projectToGround(camToBase,gndplane);
    break;
  case pillar:
    bottomLeft.projectToGround(camToBase,gndplane);
    bottomRight.projectToGround(camToBase,gndplane);
    topLeft = bottomLeft;
    topRight = bottomRight;
    topLeft.coords[2] += assumedHeight;
    topRight.coords[2] += assumedHeight;
    break;
  case poster:
    // create an elevated plane by shifting the ground plane by assumedHeight
    //*** this code seems wrong *** -- DST
    PlaneEquation elevatedPlane = gndplane;
    float const new_displacement = elevatedPlane.getDisplacement() + assumedHeight*elevatedPlane.getZsign();
    elevatedPlane.setDisplacement(new_displacement);
    
    // Project the centroid onto the elevated plane and set all 4 points to the new centroid
    Point centroid = getCentroid();
    centroid.projectToGround(camToBase, elevatedPlane);
    bottomLeft = centroid;
    bottomRight = centroid;
    topLeft = centroid;
    topRight = centroid;
    break;
  }
  update_derived_properties();
}

void BlobData::update_derived_properties() {
  switch ( orientation ) {

  case groundplane:
    // DST HACK -- should project each run to ground and integrate,
		float side1, side2, side3, side4,side5, areatri1, areatri2, s;
		side1 = topLeft.distanceFrom(topRight);
		side2 = topRight.distanceFrom(bottomRight);
		side3 = topLeft.distanceFrom(bottomRight);
		side4 = bottomLeft.distanceFrom(bottomRight);
		side5 = topLeft.distanceFrom(bottomLeft);

		s =(side1+side2+side3) / 2; // compute semi perimeter for first triangle
		areatri1=sqrt(s*(s-side1)*(s-side2)*(s-side3));

		s =(side3+side4+side5) / 2; // compute semi perimeter for second triangle
		areatri2=sqrt(s*(s-side3)*(s-side4)*(s-side5));

		area = areatri1 + areatri2;
		break;

  case poster:
    area = (bottomRight-bottomLeft).xyNorm() * assumedHeight;
    break;

  case pillar:
		// calculate area of base assuming cylindrical shape
		float radius = (bottomRight-bottomLeft).xyNorm() / 2;
		area = radius * radius * M_PI;
		break;
  }
}

bool BlobData::isMatchFor(const ShapeRoot& other) const {
  if ( ! (isSameTypeAs(other) && isSameColorAs(other)) )
    return false;
	const Shape<BlobData>& other_blob = ShapeRootTypeConst(other,BlobData);
	float dist = getCentroid().distanceFrom(other_blob->getCentroid());
	return dist < 2*sqrt(area);
}

bool BlobData::updateParams(const ShapeRoot& other, bool forceUpdate) {
  const Shape<BlobData>& other_blob = ShapeRootTypeConst(other,BlobData);
  int other_conf = other_blob->confidence;
  if (other_conf <= 0) {
    if (forceUpdate)
      other_conf = 1;
    else
      return false;
  }
  confidence += other_conf;
  const int sumconf = confidence + other_conf;
  topLeft = (topLeft*confidence + other_blob->topLeft*other_conf) / sumconf;
  topRight = (topRight*confidence + other_blob->topRight*other_conf) / sumconf;
  bottomLeft = (bottomLeft*confidence + other_blob->bottomLeft*other_conf) / sumconf;
  bottomRight = (bottomRight*confidence + other_blob->bottomRight*other_conf) / sumconf;
  topValid |= other_blob->topValid;
  bottomValid |= other_blob->bottomValid;
  leftValid |= other_blob->leftValid;
  rightValid |= other_blob->rightValid;
  float const newArea = (area*confidence + other_blob->area*other_conf) / sumconf;
  update_derived_properties();
  area = newArea; // fixup because update_derived_properties currently messes up on pillar and poster areas
  return true;
}

//================================================================
// Blob extraction

std::vector<Shape<BlobData> >
BlobData::extractBlobs(const Sketch<bool> &sketch, 
		       int minarea,
		       BlobOrientation_t orient, 
		       coordinate_t height,
		       int maxblobs) {
  // We could multiply sketch*color_index to convert to uchar and
  // spare ourselves the for loop that follows for setting blob
  // colors, but this way is actually much faster because we skip all
  // the pixel-wise multiplications.  Casting Sketch<bool> to
  // Sketch<uchar> is done with a simple reinterpret_cast; no copying.
  set<color_index> dummyColors;
  dummyColors.insert(1); // converting bool to uchar always yields color index 1
  map<color_index,int> minareas;
  minareas[1] = minarea;
  map<color_index,BlobOrientation_t> orients;
  orients[1] = orient;
  map<color_index,coordinate_t> heights;
  heights[1] = height;
  std::vector<Shape<BlobData> > result(extractBlobs((const Sketch<uchar>&)sketch,
																										dummyColors,minareas,orients,heights,maxblobs));
  rgb sketchColor(sketch->getColor());
  for ( std::vector<Shape<BlobData> >::iterator it = result.begin();
	it != result.end(); it++ )
    (*it)->setColor(sketchColor);
  return result;
}

std::vector<Shape<BlobData> > 
BlobData::extractBlobs(const Sketch<uchar> &sketch,
		       int minarea,
		       BlobOrientation_t orient,
		       coordinate_t height,
		       int maxblobs) {
  const int numColors = ProjectInterface::getNumColors();
  set<color_index> colors;
  map<color_index,int> minareas;
  map<color_index,BlobOrientation_t> orients;
  map<color_index,coordinate_t> heights;
  for (int i = 1; i < numColors; i++) {
    colors.insert(i);
    minareas[i] = minarea;
    orients[i] = orient;
    heights[i] = height;
  }
  return extractBlobs(sketch, colors, minareas, orients, heights, maxblobs);
}

std::vector<Shape<BlobData> >
BlobData::extractBlobs(const Sketch<uchar> &sketch, 
		       const std::set<color_index>& colors,
		       const std::map<color_index,int>& minareas,
		       const std::map<color_index,BlobOrientation_t>& orients,
		       const std::map<color_index,coordinate_t>& heights,
		       int maxblobs) {
  int parent = sketch->getId();
  uchar *pixels = &((*sketch.pixels)[0]);

  // convert pixel array to RLE
  int const maxRuns = (sketch.width * sketch.height) / 8;
  CMVision::run<uchar> *rle_buffer = new CMVision::run<uchar>[maxRuns];
  unsigned int const numRuns = CMVision::EncodeRuns(rle_buffer, pixels, sketch.width, sketch.height, maxRuns);

  // convert RLE to region list
  CMVision::ConnectComponents(rle_buffer,numRuns);
  int const maxRegions = (sketch.width * sketch.height) / 16;   // formula from RegionGenerator.h
  CMVision::region *regions = new CMVision::region[maxRegions];
  unsigned int numRegions = CMVision::ExtractRegions(regions, maxRegions, rle_buffer, numRuns);
  unsigned int const numColors = ProjectInterface::getNumColors();
  CMVision::color_class_state *ccs = new CMVision::color_class_state[numColors];
  unsigned int const maxArea = CMVision::SeparateRegions(ccs, numColors, regions, numRegions);
  CMVision::SortRegions(ccs, numColors, maxArea);
  CMVision::MergeRegions(ccs, int(numColors), rle_buffer);

  // extract blobs from region list
  std::vector<Shape<BlobData> > result(20);
  result.clear();
  ShapeSpace &ShS = sketch->getSpace().getDualSpace();
  //  for ( size_t color=1; color < numColors; color++ ) {
  for (set<color_index>::const_iterator it = colors.begin();
       it != colors.end(); it++) {
    int const color = *it;
    const CMVision::region* list_head = ccs[color].list;
    if ( list_head != NULL ) {
      const rgb rgbvalue = ProjectInterface::getColorRGB(color);
      int const minarea = (minareas.find(color)==minareas.end()) ? 50 : const_cast<map<color_index,int>&>(minareas)[color];
      BlobOrientation_t const orient = (orients.find(color)==orients.end()) ? 
	groundplane : const_cast<map<color_index,BlobOrientation_t>&>(orients)[color];
      coordinate_t const height = (heights.find(color)==heights.end()) ? 0 : const_cast<map<color_index,coordinate_t>&>(heights)[color];

      for (int blobcount=0; list_head!=NULL && blobcount<maxblobs && list_head->area >= minarea;
	   list_head = list_head->next, blobcount++) {
	BlobData* blobdat = new_blob(ShS,*list_head, rle_buffer, orient, height, rgbvalue);
	blobdat->setParentId(parent);
	result.push_back(Shape<BlobData>(blobdat));
      }
    }
  }
  delete[] ccs;
  delete[] regions;
  delete[] rle_buffer;
  return result;
}

BlobData* BlobData::new_blob(ShapeSpace& space,
			     const CMVision::region &reg,
			     const CMVision::run<CMVision::uchar> *rle_buff, 
			     const BlobOrientation_t orient,
			     const coordinate_t height,
			     const rgb rgbvalue) {
  int const x1 = reg.x1;
  int const y1 = reg.y1;
  int const x2 = reg.x2;
  int const y2 = reg.y2;
  // Count the number of runs so we can allocate a vector of the right size.
  // The first run might be numbered 0, so we must use -1 as end of list indicator.
  int numruns = 0;
  for (int runidx = reg.run_start; runidx != -1; 
       runidx = rle_buff[runidx].next ? rle_buff[runidx].next : -1)
    ++numruns;
  std::vector<BlobData::run> runvec(numruns);
  runvec.clear();
  // now fill in the run vector
  for (int runidx = reg.run_start; runidx != -1; 
       runidx = rle_buff[runidx].next ? rle_buff[runidx].next : -1) {
    const CMVision::run<uchar> &this_run = rle_buff[runidx];
    runvec.push_back(BlobData::run(this_run.x, this_run.y, this_run.width));
  }
  BlobData *blobdat =
    new BlobData(space,
		 Point(x1,y1,0,camcentric),Point(x2,y1,0,camcentric),
		 Point(x1,y2,0,camcentric),Point(x2,y2,0,camcentric),
		 reg.area, runvec, orient, height, rgbvalue); 
  const int left_edge_limit = 2;
  const int right_edge_limit = space.getDualSpace().getWidth() - 2;
  const int top_edge_limit = 2;
  const int bottom_edge_limit = space.getDualSpace().getHeight() - 2;
  blobdat->topValid = blobdat->topLeft.coordX() > left_edge_limit
    && blobdat->topLeft.coordY() > top_edge_limit
    && blobdat->topRight.coordX() < right_edge_limit
    && blobdat->topRight.coordY() > top_edge_limit;
  blobdat->rightValid = blobdat->topRight.coordX() < right_edge_limit
    && blobdat->topRight.coordY() > top_edge_limit
    && blobdat->bottomRight.coordX() < right_edge_limit
    && blobdat->bottomRight.coordY() < bottom_edge_limit;
  blobdat->bottomValid = blobdat->bottomRight.coordX() < right_edge_limit
    && blobdat->bottomRight.coordY() < bottom_edge_limit
    && blobdat->bottomLeft.coordX() > left_edge_limit
    && blobdat->bottomLeft.coordY() < bottom_edge_limit;
  blobdat->leftValid = blobdat->bottomLeft.coordX() > left_edge_limit
    && blobdat->bottomLeft.coordY() < bottom_edge_limit
    && blobdat->topLeft.coordX() < right_edge_limit
    && blobdat->topLeft.coordY() > top_edge_limit;
  return blobdat;
}

//================================================================
// Corner extraction: General call to find the corners of a blob.
// Used in brick / pyramid extraction
//
//
// Requires the number of corners you expect to find.
// Currently just uses shape fitting to find the corners
// Originally used either the derivative or diagonal approaches to finding corners
//
// Shape fitting works very well, but is very slow.
// (no attempt was made to optimize it though)
//
// The old diagonal/derivative approach is at the bottom
// It is much faster, but less robust.

  std::vector<Point> BlobData::findCorners(unsigned int nExpected, 
					   std::vector<Point>& candidates,
					   float &bestValue) {

    std::vector<Point> fitCorners = findCornersShapeFit(nExpected, candidates, bestValue);

    // debug output
    for (unsigned int i=0; i<fitCorners.size(); i++){
      NEW_SHAPE(fitline, LineData, LineData(*space, fitCorners[i], fitCorners[(i+1)%nExpected]));
      fitline->setParentId(getViewableId());
    }

    // Handling off-screen bricks:
    // If our fit of corners is close to the edge of the image,
    // re-try fitting 3 or 5 corners to the brick
    bool onEdge = false;
    int width = space->getDualSpace().getWidth();
    int height = space->getDualSpace().getHeight();
    for (unsigned int i=0; i<nExpected; i++) {
      if (fitCorners[i].coordX() < 5 || fitCorners[i].coordX() > width - 5 ||
	  fitCorners[i].coordY() < 5 || fitCorners[i].coordY() > height - 5) {
	onEdge = true;
	break;
      }
    }
    if (onEdge && nExpected == 4) {
      std::vector<int> outsideCandidates;
      for (unsigned int i=0; i<nExpected; i++) {
	if (candidates[i].coordX() < 5 || candidates[i].coordX() > width - 5 ||
	    candidates[i].coordY() < 5 || candidates[i].coordY() > height - 5) {
	  outsideCandidates.push_back(i);
	}
      }


      std::vector<Point> candidates3(candidates), candidates5;

      if (outsideCandidates.size() == 0) {
	std::cout<<"Err? final points are near the edge, but the candidates aren't?"<<std::endl;
      }
      
      // If just one candidate is outside the scene, 
      // try removing it for 3 points
      // and adding the points where it crosses the border of the image for 5 points
      if (outsideCandidates.size() == 1) {
	int outC = outsideCandidates[0];
	candidates3.erase(candidates3.begin() + outC);
      
	Point p1, p2;
	if (candidates[outC].coordX() < 5) {
	  p1.setCoords(0,0);
	  p2.setCoords(0,height);
	}
	else if (candidates[outC].coordX() > width - 5) {
	  p1.setCoords(width,0);
	  p2.setCoords(width,height);
	}
	else if (candidates[outC].coordY() < 5) {
	  p1.setCoords(0,0);
	  p2.setCoords(width,0);
	}
	else {
	  p1.setCoords(0,height);
	  p2.setCoords(width,height);
	}
	LineData edgeLine(*space, p1,p2);
	LineData l1(*space, candidates[(outC+3)%4], candidates[outC]);
	LineData l2(*space, candidates[(outC+1)%4], candidates[outC]);
	candidates5.push_back(candidates[(outC+3)%4]);
	candidates5.push_back(l1.intersectionWithLine(edgeLine));
	candidates5.push_back(l2.intersectionWithLine(edgeLine));
	candidates5.push_back(candidates[(outC+1)%4]);
	candidates5.push_back(candidates[(outC+2)%4]);
      }
      
      if (outsideCandidates.size() == 2) {
	Point betweenOutside = (candidates[outsideCandidates[0]] + candidates[outsideCandidates[1]])/2;
	candidates3[outsideCandidates[0]].setCoords(betweenOutside);
	candidates3.erase(candidates3.begin() + outsideCandidates[1]);
	
	int dC = outsideCandidates[1] - outsideCandidates[0];
	int c1 = outsideCandidates[1];
	candidates5.push_back(candidates[outsideCandidates[0]]);
	candidates5.push_back(betweenOutside);
	candidates5.push_back(candidates[outsideCandidates[1]]);
	candidates5.push_back(candidates[(c1+dC)%4]);
	candidates5.push_back(candidates[(c1+2*dC)%4]);
      }
      
      if (outsideCandidates.size() > 2) {
	// Not gonna get very good points out of this, probably
      }
    
      float value3, value5;
      for (unsigned int i=0; i<candidates3.size(); i++) {
	NEW_SHAPE(candidate3, PointData, PointData(*space, candidates3[i]));
	candidate3->setParentId(getViewableId());
      }
      for (unsigned int i=0; i<candidates5.size(); i++) {
	NEW_SHAPE(candidate5, PointData, PointData(*space, candidates5[i]));
	candidate5->setParentId(getViewableId());
      }
      std::vector<Point> fitcorners3 = findCornersShapeFit(3, candidates3, value3);
      std::vector<Point> fitcorners5 = findCornersShapeFit(5, candidates5, value5);
      for (unsigned int i=0; i<fitcorners3.size(); i++) {
	NEW_SHAPE(fit3, PointData, PointData(*space, fitcorners3[i]));
	fit3->setParentId(getViewableId());
      }
      for (unsigned int i=0; i<fitcorners5.size(); i++) {
	NEW_SHAPE(fit5, PointData, PointData(*space, fitcorners5[i]));
	fit5->setParentId(getViewableId());
      }
    }

    // right now just take the corners from quadrilateral fitting
    return fitCorners;


    // Old method, used the diagonal and derivative approaches and took the better results
    // Was generally much faster, but broke down in some special cases
    /* 
    const float MIN_BOUNDING_SCORE = .75;

    float derivScore = -1, diagScore = -1;

    std::vector<Point> derivCorners = findCornersDerivative();

    if (derivCorners.size() == nExpected) {

      derivScore = getBoundingQuadrilateralInteriorPointRatio(derivCorners);
      std::cout<<"Derivative score for ("<<getViewableId()<<") = "<<derivScore<<std::endl;
      if (derivScore > MIN_BOUNDING_SCORE) {
	return derivCorners;
      }
    }
    
    std::vector<Point> diagCorners = findCornersDiagonal();

    if (diagCorners.size() == nExpected) {
      diagScore = getBoundingQuadrilateralInteriorPointRatio(diagCorners);
      std::cout<<"Diagonal score for ("<<getViewableId()<<") = "<<diagScore<<std::endl;
      if (diagScore > derivScore) {
	return diagCorners;
      }
      else if (derivScore > .5) {
	return derivCorners;
      }
    }

    // can we integrate sets of incomplete / overcomplete points?

    std::vector<Point> result;

    return result;
    */
  }


  /*
   * Derivative approach to finding the corners of the blobs. 
   * Computes the distance from the center to the edge in a circle around the blob
   * Takes a couple derivatives, and looks for peaks.
   *
   * Works well on big shapes, poorly on small ones
   * Doesn't make any guarantees as to how many points are returned. 
   */
  std::vector<Point> BlobData::findCornersDerivative()
  {
    std::vector<Point> corners;

    float radius = sqrt((topRight.coordX()-topLeft.coordX())*(topRight.coordX()-topLeft.coordX()) + 
			(bottomLeft.coordY()-topLeft.coordY())*(bottomLeft.coordY()-topLeft.coordY()))/2 + 1;
    int len = (int)(2*M_PI*radius + 1);
    float distances[len];
    std::vector<Point> points(len);
    Point centroid = getCentroid();
    NEW_SKETCH(rendering, bool, getRendering());
    
    int i=0;

    findRadialDistancesFromPoint(centroid, radius, rendering, distances, points);
    float gdist[len], ddist[len], d2dist[len], d3dist[len];
    applyGaussian(distances, gdist, len);
    takeDerivative(gdist, ddist, len);
    takeDerivative(ddist, d2dist, len);
    takeDerivative(d2dist, d3dist, len);

    drawHist(gdist, len, rendering);
    drawHist(ddist, len, rendering);
    drawHist(d2dist, len, rendering);
    

    // Corners are negative peaks in the second derivative of the distance from the center
    // Zero-crossings in the first derivative should work, but we want to capture contour changes that 
    // don't necessarily cross zero (steep increase -> shallow increase could be a corner)

    const float MIN_D2 = 5.f;

    float curmin = -MIN_D2;
    int curi = -1;
    int curonpeak = 0;
    for (i=0; i<len; i++) {
      if (d2dist[i]  < curmin) {
	curmin = d2dist[i];
	curi = i;
	curonpeak = 1;
      }
      else if (curonpeak) {
	if (d2dist[i] > -MIN_D2 || (d3dist[i-1] > 0 && d3dist[i] <= 0)) {
	  curonpeak = 0;
	  curmin = -MIN_D2;
	  corners.push_back(points[curi]);
	  NEW_SHAPE(cornerpoint, PointData, Shape<PointData>(*space, points[curi])); 
	  cornerpoint->setParentId(rendering->getViewableId());
	} 
      }
    }


    // Normalize returned corners to counter-clock-wise order;
    vector<Point> reversedCorners;
    for (i=corners.size()-1; i>=0; i--){
      reversedCorners.push_back(corners[i]);
    }

    return reversedCorners;
  }


  /*
   * Diagonal approach to finding corners
   * ad-hoc heuristic works well for normal parallelograms 
   *
   * fails on trapezoids and triangles, and where nCorners != 4. 
   *
   *
   * Computes radial distance from the center like the derivative method
   * Finds the peak in distance (furthest point is once corner)
   * Finds the peak in the opposite region (another corner)
   * 
   * At this point it can draw the diagonal. The breakdown occurs when 
   * it thinks it has a diagonal at this point but it isn't an actual diagonal
   *
   * Then split the quadrilateral into two triangles, and the furthest points from the 
   * diagonal are the two remaining corners. 
   */
  std::vector<Point> BlobData::findCornersDiagonal()
  {
    std::vector<Point> corners;

    float radius = sqrt((topRight.coordX()-topLeft.coordX())*(topRight.coordX()-topLeft.coordX()) + 
			(bottomLeft.coordY()-topLeft.coordY())*(bottomLeft.coordY()-topLeft.coordY()))/2 + 1;
    int len = (int)(2*M_PI*radius + 1);
    float distances[len];
    std::vector<Point> points(len);
    Point centroid = getCentroid();
    NEW_SKETCH(rendering, bool, getRendering());
    
    int i=0;
    int maxi = 0, origmaxi = 0;
    bool stillmax = false;

    maxi = findRadialDistancesFromPoint(centroid, radius, rendering, distances, points);

    // Find second max
    int maxi2 = 0;
    float max2 = 0;
    stillmax = false;
    origmaxi = -1;
    for (i=0; i<len; i++) {
      if (distances[i] >= max2 && 
	  abs(i-maxi) > len*3/8 && 
	  abs(i-maxi) < len*5/8) {
	if (distances[i] > max2) {
	  maxi2 = i;
	  max2 = distances[i];
	  origmaxi = maxi2;
	  stillmax = true;
	}
	else if (stillmax){
	  maxi2 = (origmaxi+i)/2;
	}
      }
      else {
	stillmax = false;
      }
    }
    
    corners.push_back(points[maxi]);
    corners.push_back(points[maxi2]);
    std::cout<<"Corners: ("<<corners[0].coordX()<<","<<corners[0].coordY()<<")  ("<<
      corners[1].coordX()<<","<<corners[1].coordY()<<")\n";

    // Get the regions on either side of the line made by the two corners
    // The most distant points in those regions are the other two corners
    NEW_SHAPE(diag, LineData, Shape<LineData>(*space, corners[0], corners[1]));
    diag->firstPt().setActive(false);
    diag->secondPt().setActive(false);
    diag->setParentId(rendering->getViewableId());

    NEW_SKETCH_N(filled, bool, visops::topHalfPlane(diag));
    NEW_SKETCH(side1, bool, filled & rendering);
    NEW_SKETCH(side2, bool, (!filled) & rendering);
    
    const float MIN_PT_DIST = 3.f;

    Point pt3 = (Region::extractRegion(side1)).mostDistantPtFrom(diag.getData());
    Point pt4 = (Region::extractRegion(side2)).mostDistantPtFrom(diag.getData());
    if (diag->perpendicularDistanceFrom(pt3) > MIN_PT_DIST)
      corners.push_back(pt3);
    if (diag->perpendicularDistanceFrom(pt4) > MIN_PT_DIST)
      corners.push_back(pt4);
    

    // Sort the corners into order going around the brick
    std::vector<Point> resultCorners;
    std::vector<float> angles;

    float ta;
    Point tp;
    for (i=0; i<(int)corners.size(); i++) {
      Point di = corners[i] - centroid;
      angles.push_back(atan2(di.coordY(), di.coordX()));
      resultCorners.push_back(corners[i]);
      for (int j=i-1; j>=0; j--) {
	if (angles[j+1] > angles[j]) {
	  ta = angles[j];
	  angles[j] = angles[j+1];
	  angles[j+1] = ta;
	  tp = resultCorners[j];
	  resultCorners[j] = resultCorners[j+1];
	  resultCorners[j+1] = tp;
	}
	else{
	  break;
	}
      }
    }

    NEW_SHAPE(cornerline1, LineData, Shape<LineData>(*space, resultCorners[0], resultCorners[1])); 
    cornerline1->setParentId(rendering->getViewableId());
    if (resultCorners.size() > 3) {
      NEW_SHAPE(cornerline2, LineData, Shape<LineData>(*space, resultCorners[2], resultCorners[3]));
      cornerline2->setParentId(rendering->getViewableId());
    }
    
     return resultCorners;
  }





  // find corners by fitting a quadrilateral to the blob
  //
  // Its expecting a set of candidate points that are roughly aligned with one set of edges and are
  // significantly wider than the blob itself. 
  // It contracts the candidate points to form a rough bounding box, 
  // then does simulated annealing on random perturbations of the end points to get a good fit
  //
  // Potential quadrilateral fits are scored by maximizing the number of edge points of the blob that 
  // lie under one of the lines, and minimizing the total area. 
  // A fixed minimum edge length constraint is also enforced. 
  std::vector<Point> BlobData::findCornersShapeFit(unsigned int ncorners, std::vector<Point>& candidates, 
						   float &bestValue)
  { 
    NEW_SKETCH(rendering, bool, getRendering());

    std::vector<Point> bestPoints(ncorners), curTest(ncorners);
    float bestScore;
    int bestEdgeCount;
    std::vector<std::vector<Point> > testPoints;
    std::vector<float> testScores;
    std::vector<int> testEdgeCounts;

    if (candidates.size() == ncorners) {
      for (unsigned int i=0; i<ncorners; i++) {
	bestPoints[i].setCoords(candidates[i]);
	curTest[i].setCoords(candidates[i]);
      }
    }
    else {
      std::cout<<"Warning: incorrect number of candidates provided. Got "<<candidates.size()<<" but wanted "<<ncorners<<std::endl;
      return bestPoints;
    }

    NEW_SKETCH_N(nsum, uchar, visops::neighborSum(getRendering()));
    NEW_SKETCH(borderPixels, bool, (nsum > 0) & (nsum < 8) & getRendering());
    int edgeTotal = 0;
    for (unsigned int x=0; x<borderPixels->getWidth(); x++) {
      borderPixels(x,0) = 0;
      borderPixels(x,borderPixels->getHeight() - 1) = 0;
    }
    for (unsigned int y=0; y<borderPixels->getHeight(); y++) {
      borderPixels(0,y) = 0;
      borderPixels(borderPixels->getWidth() - 1, y) = 0;
    }
    for (unsigned int i=0; i<borderPixels->getNumPixels(); i++) {
      if (borderPixels->at(i))
	edgeTotal++;
    }

    bestScore = getBoundingQuadrilateralScore(*this, bestPoints, borderPixels, bestEdgeCount, *space);

    int testCount = 0, testEdgeCount;
    Point dp;
    float dpDist, testRatio;
    bool hasMoved;

    float annealingScalar = 1.f;
    bool doingRandomMovement = false;
    const float ANNEALING_CAP = 25.f;
    const float WEIGHT_SCALAR = .2f;
    
    const float MIN_DISTANCE = 10.f;
    const float MIN_BOUNDING_RATIO = 0.8f;

    int iterationCount = 0, annealingStart = 0;
    // Right now it just keeps going until the annealing weight gets to a set threshold
    // Should probably be improved by checking the quality of the fit at intervals
    // especially if the points have stopped moving
    while (annealingScalar < ANNEALING_CAP) {

      hasMoved = false;
      
      // Test each corner in succession
      for (unsigned int i=0; i<ncorners; i++) {

	testScores.clear();
	testPoints.clear();
	testEdgeCounts.clear();
	testCount = 0;
	// Try moving the corner toward or away from either adjacent corner by 1 pixel
	// Only look at moving toward adjacent corners until we start doing random movement

	for (unsigned int j=0; j<ncorners; j++)
	  curTest[j].setCoords(bestPoints[j]);
	dp.setCoords(curTest[(i+1)%ncorners] - curTest[i]);
	dpDist = curTest[i].distanceFrom(curTest[(i+1)%ncorners]);
	// Don't allow corners to get too close to each other
	if (dpDist > MIN_DISTANCE) {

	  dp/=dpDist;
	  curTest[i]+=dp;
	  testRatio = getBoundingQuadrilateralInteriorPointRatio(*this, curTest, *space);
	  if (testRatio > MIN_BOUNDING_RATIO) {
	    testScores.push_back(getBoundingQuadrilateralScore(*this, curTest, borderPixels, 
							       testEdgeCount, *space));
	    testPoints.push_back(curTest);
	    testEdgeCounts.push_back(testEdgeCount);
	    testCount++;
	  }

	  // Look outward too if we're in the annealing stage
	  if (doingRandomMovement) {
	    curTest[i].setCoords(bestPoints[i]);
	    curTest[i]-=dp;
	    testRatio = getBoundingQuadrilateralInteriorPointRatio(*this, curTest, *space);
	    if (testRatio > MIN_BOUNDING_RATIO) {
	      testScores.push_back(getBoundingQuadrilateralScore(*this, curTest, borderPixels, 
								 testEdgeCount, *space));
	      testPoints.push_back(curTest);
	      testEdgeCounts.push_back(testEdgeCount);
	      testCount++;
	    }

	  }
	  
	}

	curTest[i].setCoords(bestPoints[i]);
	dp.setCoords(curTest[(i+ncorners-1)%ncorners] - curTest[i]);
	dpDist = curTest[i].distanceFrom(curTest[(i+ncorners-1)%ncorners]);
	// Don't allow corners to get too close to each other
	if (dpDist > MIN_DISTANCE) {

	  dp/=dpDist;
	  curTest[i]+=dp;
	  testRatio = getBoundingQuadrilateralInteriorPointRatio(*this, curTest, *space);
	  if (testRatio > MIN_BOUNDING_RATIO) {
	    testScores.push_back(getBoundingQuadrilateralScore(*this, curTest, borderPixels, 
							       testEdgeCount, *space));
	    testPoints.push_back(curTest);
	    testEdgeCounts.push_back(testEdgeCount);
	    testCount++;
	  }

	  // Look outward too if we're in the annealing stage
	  if (doingRandomMovement) {
	    curTest[i].setCoords(bestPoints[i]);
	    curTest[i]-=dp;
	    testRatio = getBoundingQuadrilateralInteriorPointRatio(*this, curTest, *space);
	    if (testRatio > MIN_BOUNDING_RATIO) {
	      testScores.push_back(getBoundingQuadrilateralScore(*this, curTest, borderPixels, 
								 testEdgeCount, *space));
	      testPoints.push_back(curTest);
	      testEdgeCounts.push_back(testEdgeCount);
	      testCount++;
	    }
	  }
	  
	}
	

	testScores.push_back(bestScore);
	testPoints.push_back(bestPoints);
	testEdgeCounts.push_back(bestEdgeCount);
	testCount++;

	int move = -1;
	if (doingRandomMovement) {
	  move = pickMove(testScores, annealingScalar);
	}
	else {
	  move = 0;
	  for (int j=0; j<testCount; j++) {
	    if (testScores[j] < testScores[move])
	      move = j;
	  }
	  if (move != testCount-1) {
	    hasMoved = true;
	  }
	}

	if (move < 0 || move >= testCount)
	  std::cout<<"Hmm, picked a bad move somewhere ("<<move<<")\n";
	else {
	  bestPoints[i].setCoords(testPoints[move][i]);
	  bestScore = testScores[move];
	  bestEdgeCount = testEdgeCounts[move];
	}

      }

      // Increase the weight proportional to how much of the edges are covered
      // Only increase it if we're at the annealing stage though. 
      if (doingRandomMovement) {
	annealingScalar += bestEdgeCount*WEIGHT_SCALAR/edgeTotal;
      }
      else if (!hasMoved) {
	doingRandomMovement = true;

	// debug output
	annealingStart = iterationCount;
	for (unsigned int z=0; z<ncorners; z++) {
	  NEW_SHAPE(preannealing, PointData, PointData(*space, bestPoints[z]));
	  preannealing->setParentId(borderPixels->getViewableId());
	}
      }

      iterationCount++;
      if (iterationCount > 500) {
	std::cout<<"Warning, annealing stopped by max iteration count\n"<<std::endl;
	break;
      }
      
    }

    std::cout<<"Shape fit took "<<iterationCount<<" iterations ("<<iterationCount - annealingStart<<" of annealing)\n";
    bestValue = bestScore;
    return bestPoints;

  }


// Comparison predicates

bool BlobData::AreaLessThan::operator() (const Shape<BlobData> &b1, const Shape<BlobData> &b2) const {
      return b1->getArea() < b2->getArea();
}

} // namespace
