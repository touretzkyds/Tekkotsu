#include <iostream>
#include <vector>
#include <list>
#include <math.h>

#include "SketchSpace.h"
#include "Sketch.h"
#include "Region.h"
#include "visops.h"

#include "ShapeSpace.h"
#include "ShapeRoot.h"

#include <DualCoding/NaughtData.h>
#include <DualCoding/ShapeNaught.h>

#include "Crew/MapBuilder.h"
#include "VRmixin.h"


using namespace std;

namespace DualCoding {

DATASTUFF_CC(NaughtData);

NaughtData::NaughtData(ShapeSpace& _space, const Point& _centroid, float _height, float _radius) :
	BaseData(_space, getStaticType()), centroid(_centroid), height(_height), radius(_radius) {}

NaughtData::NaughtData(Region &region, float assumedHeight) :
	BaseData(region.getSpace().getDualSpace(), getStaticType()),
  centroid(region.findCentroid()), height(assumedHeight), radius() {
  int xmin1 = region.findLeftBound(), xmax1 = region.findRightBound();
  int xmin2 = region.findTopBound(), xmax2 = region.findBotBound();
  radius = std::max(std::abs(xmax1 - xmin1), std::abs(xmax2 - xmin2)) / 2;
}

bool NaughtData::isMatchFor(const ShapeRoot& other) const {
	if (!(isSameTypeAs(other) && isSameColorAs(other)))
		return false;
	const Shape<NaughtData>& otherCyl = ShapeRootTypeConst(other,NaughtData);
	float diff = (centroid - otherCyl->centroid).xyzNorm();
	if (diff < 3*radius && fabs(radius - otherCyl->radius) < radius*0.8)
		return true;
	else
		return false;
}

bool NaughtData::updateParams(const ShapeRoot& other, bool) {
  const Shape<NaughtData>& other_naught = ShapeRootTypeConst(other,NaughtData);
  if (other_naught->confidence <= 0)
    return false;
  const int other_conf = other_naught->confidence;
  centroid = (centroid*confidence + other_naught->getCentroid()*other_conf) / (confidence+other_conf);
  radius = (radius*confidence + other_naught->getRadius()*other_conf) / (confidence+other_conf);
  height = (height*confidence + other_naught->getHeight()*other_conf) / (confidence+other_conf);
  return true;
}

void NaughtData::printParams() const {
	cout << "Type = " << getTypeName() << "  ID=" << getId() << "  ParentID=" << getParentId() << endl;
	printf("  color = %d %d %d\n",getColor().red,getColor().green,getColor().blue);
	cout << "  centroid = " << centroid << endl;
	cout << "  radius = " << radius << endl;
	cout << "  height = " << height << endl;
}

void NaughtData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
  centroid.applyTransform(Tmat, newref);
}

void NaughtData::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {
  PlaneEquation raisedGroundPlane(groundplane);
  raisedGroundPlane.setDisplacement(groundplane.getDisplacement() + height);
  Point perimeter = centroid + Point(radius,0,0,egocentric);
  centroid.projectToGround(camToBase, raisedGroundPlane);
  perimeter.projectToGround(camToBase, raisedGroundPlane);
  radius = (perimeter-centroid).xyNorm();
}

std::vector<Shape<NaughtData>>
NaughtData::extractNaughts(const Sketch<bool> &sketch,
                           const fmat::Column<3>& dimensions) {
  NEW_SKETCH_N(labels,uint,visops::oldlabelcc(sketch,visops::EightWayConnect));
  NEW_SKETCH_N(fatmask, bool, visops::fillin(sketch,1,2,8));
  const int REGION_THRESH = 25;  // ignore regions smaller than this
  list<Region> regionlist = Region::extractRegions(labels,REGION_THRESH);
  std::vector<Shape<NaughtData>> naughts;
  const int THRESHOLD = 130; // for detection

  for ( Region reg : regionlist ) {
    Point center = reg.findCentroid();
    bool good = true;
    int semimaj = 0, semimin = 4000;
    float theta = 0.0;
    
    int centerCheck = 0;
    for (int i = -3; i <= 3; i++) {
      for (int j = -3; j <= 3; j++) {
	float xc = center.coordX() + i;
        float yc = center.coordY() + j;
	if( xc >= 0 && xc < sketch.width && yc >= 0 && yc < sketch.height )
	  centerCheck += fatmask(xc, yc);
    	else
	  good = false;
      }
    }
    good = (centerCheck > 4) ? false :true;

    int maxSum = 0;
    // find appropriate semimajor and semiminor 
    if ( good == true ) {
      for ( float deg = 0; deg < 360; deg++ ) {
        int r = 0;
        while (true) {
          float x1 = center.coordX() + r*cos(deg*M_PI/180);
          float y1 = center.coordY() + r*sin(deg*M_PI/180);
          if( x1 >= 0 && x1 < sketch.width && y1 >= 0 && y1 < sketch.height )
            if( !fatmask(x1,y1) )
              r++;
            else
              break;
          else {
            good = false;
            break;
          }
        }
        int r1 = r;
        if ( good ) {
          while(1) {
          float x2 = center.coordX() + r*cos(deg*M_PI/180);
          float y2 = center.coordY() + r*sin(deg*M_PI/180);
          if( x2 >= 0 && x2 < sketch.width && y2 >= 0 && y2 < sketch.height )
            if( fatmask(x2,y2) )
              r++;
            else
              break;
          else
            break;
          }
         }
	if (semimaj < (float)(r1+r-1)/2) {
	  semimaj = (float)(r1+r-1)/2;
	  theta = (semimaj > (float)(r1+r-1)/2) ? theta : deg;
          theta = theta*M_PI/180;
	}
        semimin = (semimin < (float)(r1+r-1)/2) ? semimin : (float)(r1+r-1)/2;
      }

      float theta0 = theta;
      // adjust the semimajor and semiminor
      for(int i = -5; i <=5 ; i++) {
        for(int j = -5; j <=5; j++) {
          for( int k = -10; k <= 10; k++) {
            float semimaj1 = semimaj + i;
            float semimin1 = semimin + j;
            float theta1 = theta0 + k/200;
            int sum = 0;
            float deg1 = 0;
            while( deg1 < 360 ) {
              float xtmp = center.coordX()
                + semimaj1*cos(deg1*M_PI/180)*cos(theta1)
                - semimin1*sin(deg1*M_PI/180)*sin(theta1);
              float ytmp = center.coordY()
                + semimaj1*cos(deg1*M_PI/180)*sin(theta1)
                + semimin1*sin(deg1*M_PI/180)*cos(theta1);
              if( xtmp >= 0 && xtmp < sketch.width && ytmp >= 0 && ytmp < sketch.height )
                sum += fatmask(xtmp,ytmp);
              else {
                sum = 0;
                break;
              }
              deg1 += 2;
            }
            if (maxSum < sum) {
              maxSum = sum;
              semimaj = semimaj1;
              semimin = semimin1;
              theta = theta1;
            }
          }
        }
      }
    }

    if ( maxSum <= THRESHOLD )
      good = false;

    if ( good ) {   // all tests passed
      Shape<NaughtData> temp_naught(new NaughtData(reg, dimensions[2]));
      temp_naught->setParentId(sketch->getViewableId());
      temp_naught->setColor(sketch->getColor());
      naughts.push_back(temp_naught);
    }
  }
  return naughts;
}

Sketch<bool>* NaughtData::render() const {
  SketchSpace &SkS = space->getDualSpace();
  fmat::Column<3> ctr(centroid.getCoords());
  SkS.applyTmat(ctr);
  const float &cx = ctr[0];
  const float &cy = ctr[1];
  fmat::Column<3> rad = fmat::pack(radius,0,0);
  SkS.applyTmat(rad);
  const float &scaledRadius = fabs(rad[0]);
  Sketch<bool>* result = new Sketch<bool>(SkS, "render("+getName()+")");
  (*result)->inheritFrom(*this);
  *result = 0;
  for ( float dx=-scaledRadius; dx<=scaledRadius; dx+=0.2 ) {
    float dy = sqrt(scaledRadius*scaledRadius - dx*dx);
      int const px = round(cx + dx);
      int const py1 = round(cy - dy);
      int const py2 = round(cy + dy);
      if ( px >= 0 && px < result->width) {
        if ( py1 >= 0 && py1 < result->height )
          (*result)(px,py1) = true;
        if ( py2 >= 0 && py2 < result->height )
          (*result)(px,py2) = true;
      }
  }
  return result;
}

} // namespace
