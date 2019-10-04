#include "PlannerObstacles.h"

/* Splitting the DualCoding stuff into a separate file allows us to
 * avoid compilation dependancy on the DualCoding ecosystem when
 * we just want to test PlannerObstacle math in external tools */

#include "DualCoding/ShapeBlob.h"
#include "DualCoding/ShapeBrick.h"
#include "DualCoding/ShapeCross.h"
#include "DualCoding/ShapeDomino.h"
#include "DualCoding/ShapeCylinder.h"
#include "DualCoding/ShapeEllipse.h"
#include "DualCoding/ShapeLine.h"
#include "DualCoding/ShapeNaught.h"
#include "DualCoding/ShapePoint.h"
#include "DualCoding/ShapePolygon.h"
#include "DualCoding/ShapeSphere.h"


using namespace DualCoding;

template <>
void PlannerObstacle2D::convertShapeToPlannerObstacle
(const DualCoding::ShapeRoot &shape, float inflation, std::vector<PlannerObstacle2D*> &obstacles) {
  switch ( shape->getType() ) {
      
    case pointDataType: {
      const Shape<PointData> &p = ShapeRootTypeConst(shape,PointData);
      float pointWidth = 1;  // default point width 1 mm
      CircularObstacle *circ =
      new CircularObstacle(p->getCentroid().coordX(),
                           p->getCentroid().coordY(),
                           pointWidth+inflation);
      std::ostringstream nameStream;
      nameStream << p->getName() << "-" << p->getId();
      circ->name = nameStream.str();
      circ->shapeId = p->getId();
      obstacles.push_back(circ);
    } break;
      
    case aprilTagDataType:
      // Can't handle AprilTags correctly until we have a way to find their 3D orientation
      break;
      
    case lineDataType: {
      const Shape<LineData> &line = ShapeRootTypeConst(shape,LineData);
      const fmat::SubVector<2, const fmat::fmatReal> p1(line->end1Pt().coords);
      const fmat::SubVector<2, const fmat::fmatReal> p2(line->end2Pt().coords);
      const fmat::Column<2> axis = p2 - p1;   
      float const lineWidth = 1; // default line width 1 mm
      RectangularObstacle *rect =
	new RectangularObstacle((p1 + p2)/2,
				fmat::pack(axis.norm()/2+inflation, lineWidth+inflation),
				line->getOrientation());
      std::ostringstream nameStream;
      nameStream << line->getName() << "-" << line->getId();
      rect->name = nameStream.str();
      rect->shapeId = line->getId();
      obstacles.push_back(rect);
    } break;
      
    case polygonDataType: {
      const Shape<PolygonData>& poly = ShapeRootTypeConst(shape,PolygonData);
      std::vector<LineData> lines =  poly->getEdges();
      for (unsigned int i = 0; i < lines.size(); i++) {
        const fmat::SubVector<2, const fmat::fmatReal> p1(lines[i].end1Pt().coords);
        const fmat::SubVector<2, const fmat::fmatReal> p2(lines[i].end2Pt().coords);
        const fmat::Column<2> axis = p2 - p1;
        float const lineWidth = 1; // default line width 1 mm
        RectangularObstacle *rect =
	  new RectangularObstacle((p1 + p2)/2, fmat::pack(axis.norm()/2+inflation, lineWidth+inflation),
				  lines[i].getOrientation());
        std::ostringstream nameStream;
        nameStream << poly->getName() << "-" << poly->getId() << ":" << i;
        rect->name = nameStream.str();
        rect->shapeId = poly->getId();
        obstacles.push_back(rect);
      }
      break;
    }
      
    case ellipseDataType: {
      const Shape<EllipseData> &e = ShapeRootTypeConst(shape,EllipseData);
      EllipticalObstacle* ellipse =
      new EllipticalObstacle(fmat::Column<2>(e->getCentroid().coords),
                             e->getSemimajor()+inflation,
                             e->getSemiminor()+inflation,
                             e->getOrientation());
      std::ostringstream nameStream;
      nameStream << e->getName() << "-" << e->getId();
      ellipse->name = nameStream.str();
      ellipse->shapeId = e->getId();
      obstacles.push_back(ellipse);
    } break;
      
    case sphereDataType: {
      const Shape<SphereData> &sph = ShapeRootTypeConst(shape,SphereData);
      CircularObstacle *circ =
      new CircularObstacle(sph->getCentroid().coordX(),
                           sph->getCentroid().coordY(),
                           sph->getRadius()+inflation);
      std::ostringstream nameStream;
      nameStream << sph->getName() << "-" << sph->getId();
      circ->name = nameStream.str();
      circ->shapeId = sph->getId();
      obstacles.push_back(circ);
    } break;
      
    case cylinderDataType: {
      const Shape<CylinderData>& c = ShapeRootTypeConst(shape,CylinderData);
      CircularObstacle *circ =
	new CircularObstacle(c->getCentroid().coordX(), c->getCentroid().coordY(), c->getRadius()+inflation);
      std::ostringstream nameStream;
      nameStream << c->getName() << "-" << c->getId();
      circ->name = nameStream.str();
      circ->shapeId = c->getId();
      obstacles.push_back(circ);
    } break;
      
    case blobDataType: {
      const Shape<BlobData> &b = ShapeRootTypeConst(shape,BlobData);
      RectangularObstacle *rect =
	new RectangularObstacle(fmat::Column<2>(b->getCentroid().coords), b->getBoundingBox().getDimensions()+inflation, 0);
      std::ostringstream nameStream;
      nameStream << b->getName() << "-" << b->getId();
      rect->name = nameStream.str();
      rect->shapeId = b->getId();
      obstacles.push_back(rect);
    } break;
		
	case brickDataType:{
			const Shape<BrickData> &brick = ShapeRootTypeConst(shape,BrickData);
			RectangularObstacle *rect =
	new RectangularObstacle(fmat::Column<2>(brick->getCentroid().coords), fmat::pack( 
																				(brick->getBoundingBox().getDimension(0))/2 +inflation,
																				(brick->getBoundingBox().getDimension(1))/2 +inflation), 0);
			// ^ The dimensions are divided by ~two because the RectangularObstacle() is expecting half width and half height
      std::ostringstream nameStream;
      nameStream << brick->getName() << "-" << brick->getId();
      rect->name = nameStream.str();
      rect->shapeId = brick->getId();
      obstacles.push_back(rect);
		} break;

	case dominoDataType: {
		const Shape<DominoData> &domino = ShapeRootTypeConst(shape,DominoData);
		EllipticalObstacle *ell = new EllipticalObstacle(fmat::Column<2>(domino->getCentroid().coords),
			domino->getLength()/2+inflation, domino->getWidth()/2+inflation,
			domino->getOrientation().angle());
	std::ostringstream nameStream;
	nameStream << domino->getName() << "-" << domino->getId();
	ell->name = nameStream.str();
	ell->shapeId = domino->getId();
	obstacles.push_back(ell);
		} break;

	case crossDataType: {
    const Shape<CrossData>& c = ShapeRootTypeConst(shape,CrossData);
    CircularObstacle *circ =
      new CircularObstacle(c->getCentroid().coordX(), c->getCentroid().coordY(), c->getArmSemiLength()+inflation);
    std::ostringstream nameStream;
    nameStream << c->getName() << "-" << c->getId();
    circ->name = nameStream.str();
    circ->shapeId = c->getId();
    obstacles.push_back(circ);
  } break;

	case naughtDataType: {
    const Shape<NaughtData>& naught = ShapeRootTypeConst(shape,NaughtData);
    CircularObstacle *circ =
      new CircularObstacle(naught->getCentroid().coordX(), naught->getCentroid().coordY(), naught->getRadius()+inflation);
    std::ostringstream nameStream;
    nameStream << naught->getName() << "-" << naught->getId();
    circ->name = nameStream.str();
    circ->shapeId = naught->getId();
    obstacles.push_back(circ);
  } break;
 
  default:
    std::cout << std::endl << std::endl 
              <<"Planner error: conversion of '" << shape->getName() << "' " 
              << " to PlannerObstacle2D not yet implemented." << std::endl << std::endl;
  }
}

template <>
void PlannerObstacle3D::convertShapeToPlannerObstacle
(const DualCoding::ShapeRoot &shape, float inflation, std::vector<PlannerObstacle3D*> &obstacles) {
  switch ( shape->getType() ) {
      
    case pointDataType: {
      const Shape<PointData> &p = ShapeRootTypeConst(shape,PointData);
      float pointWidth = 1;  // default point width 1 mm
      SphericalObstacle *sphere =
      new SphericalObstacle(p->getCentroid().coordX(),
                            p->getCentroid().coordY(),
                            p->getCentroid().coordZ(),
                            pointWidth+inflation);
      std::ostringstream nameStream;
      nameStream << p->getName() << "-" << p->getId();
      sphere->name = nameStream.str();
      sphere->shapeId = p->getId();
      obstacles.push_back(sphere);
    } break;
      
    case sphereDataType: {
      const Shape<SphereData> &sph = ShapeRootTypeConst(shape,SphereData);
      SphericalObstacle *so =
      new SphericalObstacle(sph->getCentroid().coordX(),
                            sph->getCentroid().coordY(),
                            sph->getCentroid().coordZ(),
                            sph->getRadius()+inflation);
      std::ostringstream nameStream;
      nameStream << sph->getName() << "-" << sph->getId();
      so->name = nameStream.str();
      so->shapeId = sph->getId();
      obstacles.push_back(so);
    } break;
      
    case cylinderDataType: {
      const Shape<CylinderData>& c = ShapeRootTypeConst(shape,CylinderData);
      CylindricalObstacle *cyl =
      new CylindricalObstacle(fmat::pack(c->getCentroid().coordX(),
                                         c->getCentroid().coordY(),
                                         c->getCentroid().coordZ()),
                              fmat::Matrix<3,3>::identity(),
                              c->getRadius(),
                              c->getHeight()/2);
      std::ostringstream nameStream;
      nameStream << c->getName() << "-" << c->getId();
      cyl->name = nameStream.str();
      cyl->shapeId = c->getId();
      obstacles.push_back(cyl);
    } break;
      
	case brickDataType:
	case dominoDataType: {
      const Shape<BrickData> &br = ShapeRootTypeConst(shape,BrickData);
      // get centroid
      const fmat::Column<3> centroid = br->getCentroid().coords;
      // determine extents
      fmat::Column<3> extents = br->getTFR().coords - centroid;
      // create orientation matrix from normalized axes
      fmat::Matrix<3,3> o;
      o.column(0) = (br->getTFR() - br->getTFL()).coords; o.column(0) /= o.column(0).norm();
      o.column(1) = (br->getTFR() - br->getTBR()).coords; o.column(1) /= o.column(1).norm();
      o.column(2) = (br->getTFR() - br->getGFR()).coords; o.column(2) /= o.column(2).norm();
      BoxObstacle *box = new BoxObstacle(centroid, o.transpose() * extents, o);
      std::ostringstream nameStream;
      nameStream << br->getName() << "-" << br->getId();
      box->name = nameStream.str();
      box->shapeId = br->getId();
      obstacles.push_back(box);
    } break;
      
    default:
      std::cout << std::endl << std::endl 
      <<"Planner error: conversion of '" << shape->getName() << "' " 
      << " to PlannerObstacle3D not yet implemented." << std::endl << std::endl;
  }
}
