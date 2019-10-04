#include "CrossData.h"

#include "ShapeCross.h"
#include "VRmixin.h"
#include "visops.h"
#include "ShapeLine.h"
#include "ShapeRoot.h"

#include "Shared/Measures.h" // for angdist
#include "Shared/mathutils.h" // for deg2rad

namespace DualCoding {

  using namespace mathutils;

	DATASTUFF_CC(CrossData);

	// CONSTRUCTOR
	CrossData::CrossData(ShapeSpace& _space, const LineData& _line1, const LineData& _line2,
                       const fmat::Column<3>& extents) :
    BaseData(_space, crossDataType), center(), armWidth(extents[1]), crossHeight(extents[2]), 
    line1(_line1), line2(_line2)
  {
		center = line1.intersectionWithLine(line2);
	}

	// Compute Bounding Box via the lines' endpoints
	BoundingBox2D CrossData::getBoundingBox() const {
		BoundingBox2D b;
		Point endPt1 = line1.end1Pt();
		Point endPt2 = line1.end2Pt();
		Point endPt3 = line2.end1Pt();
		Point endPt4 = line2.end2Pt();

		fmat::Column<3>  p1 = endPt1.getCoords();
		fmat::Column<3>  p2 = endPt2.getCoords();
		fmat::Column<3>  p3 = endPt3.getCoords();
		fmat::Column<3>  p4 = endPt4.getCoords();

		b.expand(p1);
		b.expand(p2);
		b.expand(p3);
		b.expand(p4);
		return b;
	}

	// Get Points of Bounding Box
  Point CrossData::getTopLeft() const {
		BoundingBox2D b = getBoundingBox();
		fmat::Column<2> pt1 = b.min;
		fmat::Column<2> pt2 = b.min;

		float x1 = pt1[0];
		float y1 = pt1[1];
		float x2 = pt2[0];
		float y2 = pt2[1];

		Point p = Point(std::min(x1,x2), std::max(y1,y2));
		return p;
	}

  Point CrossData::getTopRight() const {
		BoundingBox2D b = getBoundingBox();
		fmat::Column<2> pt1 = b.min;
		fmat::Column<2> pt2 = b.min;

		float x1 = pt1[0];
		float y1 = pt1[1];
		float x2 = pt2[0];
		float y2 = pt2[1];

		Point p = Point(std::max(x1,x2), std::max(y1,y2));
		return p;
	}

  Point CrossData::getBottomLeft() const {
		BoundingBox2D b = getBoundingBox();
		fmat::Column<2> pt1 = b.min;
		fmat::Column<2> pt2 = b.min;

		float x1 = pt1[0];
		float y1 = pt1[1];
		float x2 = pt2[0];
		float y2 = pt2[1];

		Point p = Point(std::min(x1,x2), std::min(y1,y2));
		return p;
	}

  Point CrossData::getBottomRight() const {
		BoundingBox2D b = getBoundingBox();
		fmat::Column<2> pt1 = b.min;
		fmat::Column<2> pt2 = b.min;

		float x1 = pt1[0];
		float y1 = pt1[1];
		float x2 = pt2[0];
		float y2 = pt2[1];

		Point p = Point(std::max(x1,x2),std::min(y1,y2));
		return p;
	}

	// Matching by checking both type and color and then moves on to look at the lines themselves
	bool CrossData::isMatchFor(const ShapeRoot& other) const {
		if ( !isSameTypeAs(other) || !isSameColorAs(other) )
		  return false;
		else {
      // std:: cout << "CrossData::isMatchFor " << getId() << "@" << getCentroid() << " against " << other << std:: endl;
		  const Shape<CrossData>& other_cross = ShapeRootTypeConst(other,CrossData);
		  return isMatchFor(other_cross.getData());
		}
	}

	bool CrossData::isMatchFor(const CrossData& other) const {
    float dist = (getCentroid() - other.getCentroid()).xyNorm();
    float semi = std::min(getArmSemiLength(),other.getArmSemiLength());
    //std::cout << "CrossData::isMatchFor  dist=" << dist << " semi=" << semi << std::endl;
    if ( dist <= 2*semi )
			return true;
		else
			return false;
	}

  bool CrossData::isAdmissible() const {
    float adist = angdist(line1.getOrientation(), line2.getOrientation());
    float len1 = line1.getLength(), len2 = line2.getLength();
    Point intersection = line1.intersectionWithLine(line2);
    if ( adist > M_PI * (1.0/3.0) && adist < M_PI * (2.0/3.0)
             && std::min(len1,len2) / std::max(len1,len2) >= 0.75
             && intersection.distanceFrom(line1.firstPt()) < line1.getLength()*0.6
             && intersection.distanceFrom(line1.firstPt()) > line1.getLength()*0.4
             && intersection.distanceFrom(line2.firstPt()) < line2.getLength()*0.6
             && intersection.distanceFrom(line2.firstPt()) > line2.getLength()*0.4)
      return true;
    else
      return false;
  }

	// updates the lines and the center point given a new cross
	bool CrossData::updateParams(const ShapeRoot& ground_root, bool force) {
		const Shape<CrossData>& ground_cross = ShapeRootTypeConst(ground_root,CrossData);
		return updateParams(ground_cross.getData(), force);
	}

	bool CrossData::updateParams(const CrossData &ground_cross, bool force) {
		if (force) {
		  setCentroid(ground_cross.getCentroid());
			setLine1(ground_cross.getLine1());
			setLine2(ground_cross.getLine2());
		}
		return force;
	}

	// Print out centerpoint and the line data for both lines
	void CrossData::printParams() const {
    std::cout << "Type = " << getTypeName() << "  ID=" << getId() << "  ParentID=" << getParentId() << std::endl;
    printf("  color = %d %d %d\n",getColor().red,getColor().green,getColor().blue);
    std::cout << "Centroid: " << center << std::endl;
		line1.printParams();
		line2.printParams();
	}

	// The 2 line representation of a cross allows us to transform the lines and the intersection point
	void CrossData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
		line1.applyTransform(Tmat,newref);
		line2.applyTransform(Tmat,newref);
		center.applyTransform(Tmat,newref);
	}

	// The 2 line representation of a cross allows us to project to ground the lines and the intersection point
	void CrossData::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {
		line1.projectToGround(camToBase,groundplane);
		line2.projectToGround(camToBase,groundplane);
		line1.setSpace(space);
		line2.setSpace(space);
		center.projectToGround(camToBase,groundplane);
	}

	// The 2 line representation of a cross allows us to render a dcross by rendering both lines
	Sketch<bool>* CrossData::render() const {
		SketchSpace &renderspace = space->getDualSpace();
		Sketch<bool>* draw_result = new Sketch<bool>(renderspace, "render("+getName()+")");
		(*draw_result)->inheritFrom(*this);
		*draw_result = 0;
		line1.renderOnTo(draw_result);
		line2.renderOnTo(draw_result);
		return draw_result;
	}

std::vector<Point> CrossData::computeGraspPoints() const {
  std::vector<Point> results;
  results.push_back(line1.end1Pt());
  results.push_back(line1.end2Pt());
  results.push_back(line2.end1Pt());
  results.push_back(line2.end2Pt());
  return results;
}

std::vector<Shape<CrossData>>
CrossData::extractCrosses(ShapeSpace &cs,
                          Sketch<bool>& sketch,
                          const Sketch<bool>& occlusions, 
                          const fmat::Column<3>& dimensions) {
  std::vector<Shape<LineData>> camLines = LineData::extractLines(sketch,occlusions);
  std::vector<Shape<CrossData>> crosses;
  // Iterate through all possible combinations of pairs of lines
  SHAPEVEC_ITERATE(camLines, LineData, line1) {
    SHAPENEXT_ITERATE(camLines, LineData, line1, line2) {
      Point intersection = line1->intersectionWithLine(line2);
      //  Make sure intersection point is on both lines and that
      //  said lines are of valid orientations with respect to one another
      if (line1->pointOnLine(intersection) && line2->pointOnLine(intersection)) {
        float adist = angdist(line1->getOrientation(), line2->getOrientation());
        float len1 = line1->getLength();
        float len2 = line2->getLength();
        float dist1 = intersection.distanceFrom(line1->firstPt());
        float dist2 = intersection.distanceFrom(line2->firstPt());
        /*
        std::cout << "CrossData: " << line1->getId() << " " << line2->getId()
                  << " intersect=" << intersection
                  << " adist=" << adist << " len1=" << len1 << " len2=" << len2
                  << " dist1=" << dist1 << " : " << dist1/len1 
                  << " dist2=" << dist2 << " : " << dist2/len2
                  << std::endl;
        */
        if ( adist > M_PI / 9.0 && std::min(len1,len2) / std::max(len1,len2) >= 0.25
             && std::max(len1,len2) < sketch->getWidth()/2
             && dist1 < line1->getLength()*0.7 && dist1 > line1->getLength()*0.3
             && dist2 < line2->getLength()*0.7 && dist2 > line2->getLength()*0.3) {
          CrossData newcross(cs, line1.getData(), line2.getData(), dimensions);
          newcross.setColor(line1->getColor());
          bool matched = false;
          for ( std::vector<Shape<CrossData>>::iterator oldcross_it = crosses.begin();
                oldcross_it != crosses.end(); ) {  // don't bump iterator here; do it at the bottom
            Shape<CrossData>& oldcross = *oldcross_it;
            if ( oldcross->isMatchFor(newcross) ) {
              int oldlen = std::min(oldcross->line1.getLength(), oldcross->line2.getLength());
              int newlen = std::min(newcross.line1.getLength(), newcross.line2.getLength());
              if ( newlen <= oldlen ) {   // old cross is better; discard this new one
                matched = true;
                break;
              }
              else { // new cross is better; replace the old one
                oldcross.deleteShape();
                oldcross_it = crosses.erase(oldcross_it);  // this is why we don't bump the iterator above
                continue; // bumping iterator wouldn't be safe if oldcross_it is now at crosses.end()
              }
            }
            oldcross_it++;  // bumping iterator is safe here
          }
          if ( ! matched ) {
            Shape<CrossData> nc(new CrossData(newcross));
            nc->setViewable(true);
            nc->setParentId(sketch->getViewableId());
            crosses.push_back(nc);
          }
        }
      }
  	} END_ITERATE;
  } END_ITERATE;
  // std:: cout << "Found " << camLines.size() << " lines and recognized "
  //        << crosses.size() << " cross" << (crosses.size()==1 ? "" : "es") << std::endl;
  SHAPEVEC_ITERATE(camLines, LineData, line) {
    line.deleteShape();
	} END_ITERATE;
  return crosses;
}

} // namespace
