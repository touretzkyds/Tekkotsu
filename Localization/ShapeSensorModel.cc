#include "ShapeSensorModel.h"

#include "Motion/Kinematics.h"  // for kine variable
#include "Shared/Config.h"  // for config variable
#include "DualCoding/VRmixin.h"
#include "ShapeLandmarks.h"

#include <cmath>
#include <iostream>

using namespace DualCoding;

float const LocalShapeEvaluator::maxDist = 1e10;
float const LocalShapeEvaluator::stdevSq = 150*150; // was 60*60;

LocalShapeEvaluator::LocalShapeEvaluator(ShapeSpace &localShS, ShapeSpace &worldShS) : 
  localLms(), worldLms() {
  PfRoot::loadLms(localShS.allShapes(), false, localLms);
  PfRoot::loadLms(worldShS.allShapes(), true, worldLms);
  std::cout << "LocalShapeEvaluator: " << worldShS.allShapes().size() << " world shapes. "
            << localShS.allShapes().size() << " local shapes =";
  for ( const ShapeRoot& s : localShS.allShapes() )
    std::cout << " " << s->getName();
  std::cout << endl;
  if ( localLms.size()==0 || worldLms.size()==0 ) {
    std::cout << "ParticleFilter::loadLms found " << localLms.size() << " local and "
	      << worldLms.size() << " world landmarks: can't localize!" << std::endl;
  }
}

void LocalShapeEvaluator::evaluate(LocalizationParticle& p) {
  unsigned int const nLocals = localLms.size();
  float particleViewX[nLocals],  particleViewY[nLocals], particleViewX2[nLocals],  particleViewY2[nLocals];
  int localMatches[nLocals];
  float localScores[nLocals];
  evaluateWorkhorse(p, nLocals, particleViewX, particleViewY, particleViewX2, particleViewY2,
		    localMatches, localScores);
  //  for (unsigned int i=0; i<nLocals; i++)
  //    std::cout <<"  lm " << i << ":   " << localMatches[i] <<  " score " << localScores[i] << std::endl;
  //  std::cout << "    oldweight " << p.weight;
  updateWeight(p, localMatches, localScores);
  //  std::cout << "    newweight " << p.weight << std::endl;
}

void LocalShapeEvaluator::evaluateWorkhorse
(LocalizationParticle& p, const unsigned int nLocals,
 float particleViewX[], float particleViewY[], float particleViewX2[], float particleViewY2[],
 int localMatches[], float localScores[]) {
  // determine position of local space landmark in world given the current particle
  float const cosT = std::cos(-p.theta);
  float const sinT = std::sin(-p.theta);
  float const negSinT = -sinT;
	// std::cout << "Particle x=" << p.x << " y=" << p.y << " th=" << p.theta << std::endl;
  for ( unsigned int indexL=0; indexL < nLocals; indexL++ ) {
    PfRoot &landmark = *(localLms[indexL]);
    particleViewX[indexL] = landmark.x * cosT + landmark.y * sinT + p.x; 
    particleViewY[indexL] = landmark.x * negSinT + landmark.y * cosT + p.y;
    if ( landmark.type == lineDataType ) {
      const PfLine &line = static_cast<PfLine&>(landmark);
      particleViewX2[indexL] = line.x2 * cosT + line.y2 * sinT + p.x;
      particleViewY2[indexL] = line.x2 * negSinT + line.y2 * cosT + p.y;
    }
  }
  // Now compute match scores for the particle by finding matches between local landmarks and world landmarks.
  for ( unsigned int indexL = 0; indexL < nLocals; indexL++ ) {
    float distsq = maxDist; // distance > this is treated as a non-match; value should be < 1e10 to avoid underflows when not using log weights
    localMatches[indexL] = -1;  // assume no match unless we find something
    for ( unsigned int indexW=0; indexW<worldLms.size(); indexW++ ) {
      if ( localLms[indexL]->type == worldLms[indexW]->type &&
					 localLms[indexL]->color == worldLms[indexW]->color ) {
				float const lx = particleViewX[indexL];
				float const ly = particleViewY[indexL];
				float const wx = worldLms[indexW]->x;
				float const wy = worldLms[indexW]->y;
				float tempDistsq;

				// dispatch on landmark type because lines and markers require special match handling
				switch ( localLms[indexL]->type ) {
				case lineDataType: {
					PfLine &localLine = *static_cast<PfLine*>(localLms[indexL]);
					PfLine &worldLine = *static_cast<PfLine*>(worldLms[indexW]);
					float tempDistsq1, tempDistsq2;
					// If endpoints are valid, compare distance between endpoints.
					// If not valid, measure perpendicular distance from the local endpoint
					// to the world line segment, if the projection of the endpoint onto the
					// segment occurs within the segment, not beyond it.  Instead of calculating
					// the projection we use a heuristic test: either the x or y endpoint value must
					// lie within the range of the line segment.
					if ( (localLine.valid1 && worldLine.valid1) ||
							 !( (lx >= std::min(worldLine.x,worldLine.x2) && lx <= std::max(worldLine.x,worldLine.x2)) ||
									(ly >= std::min(worldLine.y,worldLine.y2) && ly <= std::max(worldLine.y,worldLine.y2)) ) ) {
						tempDistsq1 = (lx-wx)*(lx-wx) + (ly-wy)*(ly-wy);
					} else {
						float const tempDist1 = distanceFromLine(lx,ly,worldLine);
						tempDistsq1 = tempDist1 * tempDist1;
					}
					float const lx2 = particleViewX2[indexL];
					float const ly2 = particleViewY2[indexL];
					float const wx2 = worldLine.x2;
					float const wy2 = worldLine.y2;
					if ( (localLine.valid2 && worldLine.valid2) ||
							 !( (lx2 >= std::min(worldLine.x,worldLine.x2) && lx2 <= std::max(worldLine.x,worldLine.x2)) ||
									(ly2 >= std::min(worldLine.y,worldLine.y2) && ly2 <= std::max(worldLine.y,worldLine.y2)) ) )
						tempDistsq2 = (lx2-wx2)*(lx2-wx2) + (ly2-wy2)*(ly2-wy2);
					else {
						float const tempDist2 = distanceFromLine(lx2,ly2,worldLine);
						tempDistsq2 = tempDist2 * tempDist2;
					}
					AngPi const localOrient = localLine.orientation + p.theta;
					AngPi odiff = worldLine.orientation - localOrient;
					odiff = std::min<float>(odiff, M_PI - odiff);
					float const odist = 500 * std::sin(odiff);
					float const odistsq = odist * odist;
					tempDistsq = tempDistsq1 + tempDistsq2 + odistsq; // plus orientation match term?
					if ( false && tempDistsq1 <= 10000 && tempDistsq2 <= 10000 ) {
						std::cout << "line " << lx << "," << ly << " : " << wx << "," << wy
											<< " end1valid=" << localLine.valid1 << ":" << worldLine.valid1
											<< " tempDistsq1=" << tempDistsq1
											<< " indexL:W=" << indexL << ":" << indexW << std::endl;
						std::cout << "     " << lx2 << "," << ly2 << " : " << wx2 << "," << wy2
											<< " end2valid=" << localLine.valid2 << ":" << worldLine.valid2
											<< " tempDistsq2=" << tempDistsq2
											<< " odistsq = " << odistsq << " total=" << tempDistsq << std::endl;
					}
				}
					break;
				case ellipseDataType:
				case blobDataType:
				case cylinderDataType:
        case naughtDataType:
        case crossDataType:
          {
					tempDistsq = (lx-wx)*(lx-wx) + (ly-wy)*(ly-wy);
					// *** EXPERIMENTAL ***
					PfRoot &landmark = *(localLms[indexL]);
					AngTwoPi ltheta = atan2(landmark.y,landmark.x);
					float ldist = sqrt(landmark.x * landmark.x + landmark.y * landmark.y);
					AngTwoPi ptheta = atan2(wy-p.y,wx-p.x) - p.theta;
					float pdist = sqrt((wx-p.x)*(wx-p.x) + (wy-p.y)*(wy-p.y));
					float thetadiff = angdist(ptheta,ltheta);
					tempDistsq = (pdist-ldist)*(pdist-ldist) + 5000*thetadiff; // *pdist*pdist;
					if ( false && tempDistsq < 50000 )
						std::cout << "dists " << ldist << " " << pdist << "   thetas " << ltheta << " " << ptheta
											<< "  thetadiff=" << thetadiff << "   tempDistsq=" << tempDistsq << std::endl;
					break;
				}
				case markerDataType: {
					PfMarker &localMarker = *static_cast<PfMarker*>(localLms[indexL]);
					PfMarker &worldMarker = *static_cast<PfMarker*>(worldLms[indexW]);

					// check for marker "equality"
					if (localMarker.data->isMatchingMarker(worldMarker.data))
						tempDistsq = (lx-wx)*(lx-wx) + (ly-wy)*(ly-wy);
					else
						continue;
					break;
				}
				case aprilTagDataType: {
					PfAprilTag &localTag = *static_cast<PfAprilTag*>(localLms[indexL]);
					PfAprilTag &worldTag = *static_cast<PfAprilTag*>(worldLms[indexW]);

					// check for tag "equality"
					if ( localTag.data->getTagID() == worldTag.data->getTagID() )
						tempDistsq = (lx-wx)*(lx-wx) + (ly-wy)*(ly-wy);
					else
						continue;
					break;
				}
				case pointDataType:
					// don't try to match points; they're just placeholders, not landmarks
					tempDistsq = maxDist;
					break;
				default:
					std::cout << "ParticleFilter::computeMatchScore() can't match landmark type "
										<< localLms[indexL]->type << std::endl;
					return;
				}

				// if this world landmark is a closer match, accept it
				if ( tempDistsq < distsq ) {
					distsq = tempDistsq;
					localMatches[indexL] = indexW;
				}
      }
    }

    if ( localMatches[indexL] != -1 || ! localLms[indexL]->mobile )
      localScores[indexL] = distsq;
  }
}

void LocalShapeEvaluator::updateWeight(LocalizationParticle &p, 
																			 int const localMatches[], float const localScores[]) {
  for (unsigned int i=0; i < localLms.size(); i++)
    if ( localMatches[i] != -1 )
      p.weight += -localScores[i]/stdevSq;
}

float LocalShapeEvaluator::distanceFromLine(coordinate_t x0, coordinate_t y0, PfLine &wline) {
  float const &x1 = wline.x;
  float const &y1 = wline.y;
  float const &x2 = wline.x2;
  float const &y2 = wline.y2;
  float const &length = wline.length;
  float const result = std::fabs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / length;
  return result;
}

bool projectShapeToCamera(const LocalizationParticle &particle, const ShapeRoot *shp, float& px, float& py)
{
  //    cout << "PROJECT: marker to cam: part " << particle << " marker " << m << endl;

  // get centroid
  BaseData const &bd = shp->getData();
  Point worldPt = bd.getCentroid();

  // transform to local frame
  // lx = (wx - x) * cos(theta) + (wy - y) * sin(theta)
  // ...
  Point localPt;
  worldPt.coords[0] -= particle.x;
  worldPt.coords[1] -= particle.y;

  localPt.coords[0] = worldPt.coords[0] * std::cos(particle.theta) + worldPt.coords[1] * std::sin(particle.theta);
  localPt.coords[1] = worldPt.coords[0] * -std::sin(particle.theta) + worldPt.coords[1] * std::cos(particle.theta);
  localPt.coords[2] = worldPt.coords[2];
  //    cout << "PROJECT: transformed to local: " << localPt << endl;

  // check for shape behind the camera (bad projection)
  if (localPt.coords[0] < 0)
    return false;

  // transform from local to camera coordinates
  Point cameraPt(localPt);
#ifdef TGT_HAS_CAMERA
  cameraPt.applyTransform(kine->baseToLink(CameraFrameOffset));
#endif

  // compute pixels
  float camXnorm, camYnorm;
  config->vision.computePixel(cameraPt.coordX(), cameraPt.coordY(), cameraPt.coordZ(), camXnorm, camYnorm);

  // transform from normalized [-1,+1] coordinates to actual pixel coordinates
  px = (camXnorm + 1)/2*VRmixin::camSkS.getWidth();
  py = (camYnorm + 1)/2*VRmixin::camSkS.getHeight();
  return true;
}

void CameraShapeEvaluator::computeLikelihood(LocalizationParticle &particle)
{
  //    cout << "SENSOR MODEL: Computing likelihood for particle " << particle << endl;

  // look through all markers in sensor reading (image)
  SHAPEROOTVEC_ITERATE(cShS, cs)
    switch (cs->getType()) {
    case markerDataType : {
      BaseData const &bd = cs.getData();
      //cout << "SENSOR MODEL: Searching for match for " << cm << endl;
      fmat::Column<2> original(bd.getCentroid().coords);

      float bestMatch = std::log(pRandom);

      // look for best match in world space
      SHAPEROOTVEC_ITERATE(wShS, wm)

	if (wm->isSameTypeAs(cs)) {
	  //cout << "SENSOR MODEL: Found matching markers: " << cm << " and " << wm << endl;
	  // try and project marker
	  fmat::Column<2> projected;
	  DualCoding::ShapeRoot *targetLandMark(&wm);
	  if (projectShapeToCamera(particle, targetLandMark, projected[0], projected[1])) {
	    float prob = std::log((1- pRandom) * (normpdf(projected[0] - original[0], xvar) *
					     normpdf(projected[1] - original[1], yvar)) + pRandom) * alpha;
	    if (prob > bestMatch) {
	      bestMatch = prob;
	    }
	  }
	}
      END_ITERATE;
      // don't need to normalize here because every particle will have same
      // number of markers in image
      particle.weight += bestMatch;
    }
      break;
    default:
      break;
    }
  END_ITERATE;
}		
