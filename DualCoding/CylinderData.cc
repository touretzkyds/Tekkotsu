#include <DualCoding/CylinderData.h>
#include <DualCoding/ShapeCylinder.h>
#include "SketchSpace.h"
#include "Sketch.h"
#include "ShapeSpace.h"
#include "ShapeRoot.h"

#include "ShapeBlob.h" //To extract pillar blobs before extracting cylinders
using namespace std;

namespace DualCoding {

DATASTUFF_CC(CylinderData);

CylinderData::CylinderData(ShapeSpace& _space, Point _centroid, float _height, float _radius, fmat::Quaternion _orientation) :
	BaseData(_space, getStaticType()), centroid(_centroid), height(_height), radius(_radius), orientation(_orientation) {
	centroid.setRefFrameType(getRefFrameType());
	mobile = true;
}

bool CylinderData::isMatchFor(const ShapeRoot& other) const {
	if (!(isSameTypeAs(other) && isSameColorAs(other)))
		return false;
	
	const Shape<CylinderData>& otherCyl = ShapeRootTypeConst(other,CylinderData);
	
	float diff = (centroid - otherCyl->centroid).xyzNorm();
	//if (diff < 2*radius && fabs(radius - otherCyl->radius) < radius/2)
	if (diff < 3*radius && fabs(radius - otherCyl->radius) < radius*0.8) // *** quick fix for Tapia
		return true;
	else
		return false;
}

//! return the centroid of the shape in point format
Point CylinderData::getCentroid() const { return centroid; }

bool CylinderData::updateParams(const ShapeRoot&, bool) { return true; }

BoundingBox2D CylinderData::getBoundingBox() const {
	BoundingBox2D b;
	b.expand(fmat::pack(centroid.coordX()-radius, centroid.coordY()+height/2));
	b.expand(fmat::pack(centroid.coordX()+radius, centroid.coordY()-height/2));
	return b;
}

void CylinderData::printParams() const {
	cout << "Type = " << getTypeName() << "  ID=" << getId() << "  ParentID=" << getParentId() << endl;
	printf("  color = %d %d %d\n",getColor().red,getColor().green,getColor().blue);
	cout << "  centroid = " << centroid << endl;
	cout << "  radius = " << radius << endl;
	cout << "  height = " << height << endl;
	cout << "  orientation = " << orientation << endl;
}

void CylinderData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
	centroid.applyTransform(Tmat,newref);
}

void CylinderData::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {
	float camRadius = radius;
	float camHeight = height;
	float assumedHalfHeight = centroid.coordZ(); // we stored the assumed half-height as the z coord in camera space
	Point bottomCenterPt = Point (centroid.coordX(), centroid.coordY()+height/2, 0, camcentric);
	Point bottomLeftPt = Point (centroid.coordX()-radius, centroid.coordY()+height/2, 0, camcentric);
	bottomLeftPt.projectToGround(camToBase,groundplane);
	bottomCenterPt.projectToGround(camToBase,groundplane);
	float diff = (bottomCenterPt - bottomLeftPt).xyNorm();
	radius = diff;
	float theta = std::atan2(bottomCenterPt.coordY(), bottomCenterPt.coordX());
	float xcenter = bottomCenterPt.coordX() + radius * cos(theta);
	float ycenter = bottomCenterPt.coordY() + radius * sin(theta);
	if ( assumedHalfHeight > 0 )
		height = assumedHalfHeight * 2;   // use assumed height if provided
	else
		height = radius/camRadius * camHeight;  // else infer height from cam shape's aspect ratio
	centroid = Point(xcenter, ycenter, height/2, egocentric);
}

Sketch<bool>* CylinderData::render() const {
    SketchSpace &SkS = space->getDualSpace();
    Sketch<bool>* result = new Sketch<bool>(SkS, "render("+getName()+")");
    *result = false;
    const float semimajor = radius;
    const float semiminor = semimajor;
    fmat::Column<3> ctr;
    ctr[0] = getCentroid().coordX();
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
    return result;
}

std::vector<Shape<CylinderData> >
CylinderData::extractCylinders(const Sketch<uchar> &sketch, 
															 const std::set<color_index>& colors,
															 const std::map<color_index,coordinate_t>& assumedHeights,
															 const std::map<color_index,int>& minCylinderAreas,
															 int maxcylinders,
															 std::vector<GazePoint> &addGazePts) {

	//const std::map<color_index, coordinate_t> heights;
	std::map<color_index, BlobData::BlobOrientation_t> orients;
	for (std::map<color_index, BlobData::BlobOrientation_t>::iterator it = orients.begin(); it != orients.end(); ++it) {
		it->second = BlobData::pillar;
	}

	std::vector<Shape<BlobData> > blobs = BlobData::extractBlobs(sketch, colors, minCylinderAreas, orients, assumedHeights, maxcylinders);
	std::vector<Shape<CylinderData> > resultingCylinders;

	ShapeSpace &shs = sketch->getSpace().getDualSpace();
	for (std::vector<Shape<BlobData> >::const_iterator it = blobs.begin(); it != blobs.end(); it++) {
		const Shape<BlobData> &blob = *it;
		Point centroid = blob->getCentroid();
		if ( blob->bottomValid && blob->leftValid && blob->rightValid ) {   // topValid not needed since we have assumed height
			//height and radius are calculated from the corner points of the blob
			float Ydist = blob->bottomLeft.coordY() - blob->topLeft.coordY();
			float height = blob->topValid ? Ydist : 0;
			float diameter = blob->topRight.coordX() - blob->topLeft.coordX();
			float radius = diameter / 2;
			//std::cout << "Cylinder blob: height=" << height << " diameter=" << diameter << " area=" << blob->getArea() << std::endl;
			if ( blob->getArea() > height*diameter/3 ) { // not a hollow blob
				fmat::Quaternion orientation = fmat::Quaternion(); //=?
				std::map<color_index,coordinate_t>::const_iterator it_h = assumedHeights.find(ProjectInterface::getColorIndex(blob->getColor()));
				coordinate_t assumedHeight = (it_h != assumedHeights.end()) ? it_h->second : 0;
				(centroid.getCoords())[2] = assumedHeight/2;  // cache the assumed half-height in the centroid z coordinate
				CylinderData *cylinder = new CylinderData(shs, centroid, height, radius, orientation);
				cylinder->setColor(blob->getColor());
				cylinder->setParentId(blob->getParentId());
				resultingCylinders.push_back(Shape<CylinderData>(cylinder));
			}
		}
		else {
			// add gaze points to get a better look at the blob
			addGazePts.push_back(GazePoint(GazePoint::centered,centroid));
		}
		shs.deleteShape(const_cast<Shape<BlobData>&>(blob));
	}
	return resultingCylinders;
}	

} // namespace
