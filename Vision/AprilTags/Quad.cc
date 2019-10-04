#include "Vision/AprilTags/FloatImage.h"
#include "Vision/AprilTags/MathUtil.h"
#include "Vision/AprilTags/GLine2D.h"
#include "Vision/AprilTags/Quad.h"
#include "Vision/AprilTags/Segment.h"

namespace AprilTags {
	
const float Quad::maxQuadAspectRatio = 32;

Quad::Quad(const std::vector< std::pair<float,float> >& p, const std::pair<float,float>& opticalCenter)
  : quadPoints(p), segments(), observedPerimeter(), homography(opticalCenter)
#ifdef QUAD_INTERPOLATE
  , p0(fmat::pack(p[0].first, p[0].second)), p3(fmat::pack(p[3].first, p[3].second)) ,p01(), p32()
#endif
 {
#ifdef QUAD_INTERPOLATE
  // borrowed from Michael Kaess' AprilTags library
  const fmat::Column<2,float> p1 = fmat::pack(p[1].first, p[1].second);
  const fmat::Column<2,float> p2 = fmat::pack(p[2].first, p[2].second);
  p01 = p1 - p0;
  p32 = p2 - p3;
#endif
  // Keep homography even if interpolating, since we'll use this to
  // orient the AprilTag in the world map.  But the results might not
  // be very good because we're not using a good algorithm to find the
  // homography.  See Michael Kaess' version of Homography33 that
  // calls cv::findHomography for better results.
  homography.addCorrespondence(-1, -1, quadPoints[0].first, quadPoints[0].second);
  homography.addCorrespondence( 1, -1, quadPoints[1].first, quadPoints[1].second);
  homography.addCorrespondence( 1,  1, quadPoints[2].first, quadPoints[2].second);
  homography.addCorrespondence(-1,  1, quadPoints[3].first, quadPoints[3].second);
	homography.compute();
}

std::pair<float,float> Quad::interpolate(float x, float y) {
#ifdef QUAD_INTERPOLATE
  // borrowed from Michael Kaess' AprilTags library
  fmat::Column<2,float> r1 = p0 + p01 * (x+1.0)/2.0;
  fmat::Column<2,float> r2 = p3 + p32 * (x+1.0)/2.0;
  fmat::Column<2,float> r = r1 + (r2-r1) * (y+1.0)/2.0;
  return std::pair<float,float>(r[0],r[1]);
#else
  return homography.project(x,y);
#endif
}

void Quad::search(const FloatImage& fImage, std::vector<Segment*>& path,
	    Segment& parent, int depth, std::vector<Quad>& quads) {
  // cout << "Searching segment " << parent.getId() << ", depth=" << depth << ", #children=" << parent.children.size() << endl;
  // terminal depth occurs when we've found four segments.
  if (depth == 4) {
    // cout << "Entered terminal depth" << endl; // debug code

    // Is the first segment the same as the last segment (i.e., a loop?)
    if (path[4] == path[0]) {
      // the 4 corners of the quad as computed by the intersection of segments.
      std::vector< std::pair<float,float> > p(4);
      float calculatedPerimeter = 0;
      bool bad = false;
      for (int i = 0; i < 4; i++) {
	// compute intersections between all the lines. This will give us 
	// sub-pixel accuracy for the corners of the quad.
	GLine2D linea(std::make_pair(path[i]->getX0(),path[i]->getY0()),
		      std::make_pair(path[i]->getX1(),path[i]->getY1()));
	GLine2D lineb(std::make_pair(path[i+1]->getX0(),path[i+1]->getY0()),
		      std::make_pair(path[i+1]->getX1(),path[i+1]->getY1()));

	p[i] = linea.intersectionWith(lineb);
	calculatedPerimeter += path[i]->getLength();

	// no intersection? Occurs when the lines are almost parallel.
	if (p[i].first == -1)
	  bad = true;
      }
      // cout << "bad = " << bad << endl;
      // eliminate quads that don't form a simply connected loop, i.e., those 
      // that form an hour glass, or wind the wrong way.
      if (!bad) {
    float t0 = std::atan2(p[1].second-p[0].second, p[1].first-p[0].first);
	float t1 = std::atan2(p[2].second-p[1].second, p[2].first-p[1].first);
	float t2 = std::atan2(p[3].second-p[2].second, p[3].first-p[2].first);
	float t3 = std::atan2(p[0].second-p[3].second, p[0].first-p[3].first);

	//	double ttheta = fmod(t1-t0, 2*M_PI) + fmod(t2-t1, 2*M_PI) +
	//	  fmod(t3-t2, 2*M_PI) + fmod(t0-t3, 2*M_PI);
	float ttheta = MathUtil::mod2pi(t1-t0) + MathUtil::mod2pi(t2-t1) +
	  MathUtil::mod2pi(t3-t2) + MathUtil::mod2pi(t0-t3);
	// cout << "ttheta=" << ttheta << endl;
	// the magic value is -2*PI. It should be exact, 
	// but we allow for (lots of) numeric imprecision.
	if (ttheta < -7 || ttheta > -5)
	  bad = true;
      }

      if (!bad) {
	float d0 = MathUtil::distance2D(p[0], p[1]);
	float d1 = MathUtil::distance2D(p[1], p[2]);
	float d2 = MathUtil::distance2D(p[2], p[3]);
	float d3 = MathUtil::distance2D(p[3], p[0]);
	float d4 = MathUtil::distance2D(p[0], p[2]);
	float d5 = MathUtil::distance2D(p[1], p[3]);

	// check sizes
	if (d0 < Quad::minimumEdgeLength || d1 < Quad::minimumEdgeLength || d2 < Quad::minimumEdgeLength ||
	    d3 < Quad::minimumEdgeLength || d4 < Quad::minimumEdgeLength || d5 < Quad::minimumEdgeLength) {
	  bad = true;
	  // cout << "tagsize too small" << endl;
	}

	// check aspect ratio
	float dmax = max(max(d0,d1), max(d2,d3));
	float dmin = min(min(d0,d1), min(d2,d3));

	if (dmax > dmin*Quad::maxQuadAspectRatio) {
	  bad = true;
	  // cout << "aspect ratio too extreme" << endl;
	}
      }

      if (!bad) {
	std::pair<float,float> opticalCenter(fImage.getWidth()/2, fImage.getHeight()/2);
	Quad q(p, opticalCenter);
	q.segments=path;
	q.observedPerimeter = calculatedPerimeter;
	quads.push_back(q);
      }
    }
    return;
  }

  //  if (depth >= 1) // debug code
  //cout << "depth: " << depth << endl;

  // Not terminal depth. Recurse on any children that obey the correct handedness.
  for (unsigned int i = 0; i < parent.children.size(); i++) {
    Segment &child = *parent.children[i];
    //    cout << "  Child " << child.getId() << ":  ";
    // (handedness was checked when we created the children)
    
    // we could rediscover each quad 4 times (starting from
    // each corner). If we had an arbitrary ordering over
    // points, we can eliminate the redundant detections by
    // requiring that the first corner have the lowest
    // value. We're arbitrarily going to use theta...
    if ( child.getTheta() > path[0]->getTheta() ) {
      // cout << "theta failed: " << child.getTheta() << " > " << path[0]->getTheta() << endl;
      continue;
    }
    path[depth+1] = &child;
    search(fImage, path, child, depth+1, quads);
  }
}

std::ostream& operator<<(std::ostream &os, const Quad &quad) {
  os << "Quad(";
  for (std::vector<std::pair<float,float> >::const_iterator it = quad.quadPoints.begin();
       it != quad.quadPoints.end(); it++) {
    if ( it != quad.quadPoints.begin() )
      os << ", ";
    os << "[" << int(round(it->first)) << "," << int(round(it->second)) << "]";
  }
  os << ")";
  return os;
}

} // namespace
