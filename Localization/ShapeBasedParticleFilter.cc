//-*-c++-*-

#include <cmath>
#include <iostream>

#include "Crew/MapBuilder.h" 
#include "DualCoding/ShapeGraphics.h"
#include "DualCoding/ShapeLocalizationParticle.h"
#include "DualCoding/ShapeSpace.h"
#include "DualCoding/ShapeFuns.h"
#include "DualCoding/VRmixin.h"  // for mapBuilder
#include "Localization/ShapeBasedParticleFilter.h"

using namespace std;

namespace DualCoding {
	
void ShapeBasedParticleFilter::resetFilter() {
  const float w = dynamic_cast<const LowVarianceResamplingPolicy*>(resampler)->logWeights ? 0 : 1;
  VRmixin::particleFilter->resetFilter(w);
}

void ShapeBasedParticleFilter::resizeParticles(unsigned int numParticles) {
  ShapeSpace &wShS = sensorModel->getWorldShS();
  unsigned int numDisp = select_type<LocalizationParticleData>(wShS).size();
  if ( numParticles < numDisp )
    wShS.deleteShapes<LocalizationParticleData>();
  ParticleFilter<LocalizationParticle>::resizeParticles(numParticles);
  if ( numParticles < numDisp )
    displayParticles(numDisp);  
}

void ShapeBasedParticleFilter::synchEstimateToAgent() {
  estimate.x = VRmixin::theAgent->getCentroid().coordX();
  estimate.y = VRmixin::theAgent->getCentroid().coordY();
  estimate.theta = VRmixin::theAgent->getOrientation();
}

void ShapeBasedParticleFilter::setPosition(float const x, float const y, AngTwoPi const orientation, float jiggleVariance) {
  LocalizationParticle part(x, y, orientation);
  setPosition(part, jiggleVariance);
}

void ShapeBasedParticleFilter::computeVariance() {
  float sumWeight = 0;
  for (particle_collection::const_iterator p = particles.begin(); p != particles.end(); p++)
		sumWeight += fmax(1e-20, exp(p->weight));
	float meanWeight = sumWeight / particles.size();

	float sumW = 0, sumXYSq = 0, sumCos = 0, sumSin = 0, sumWtSq = 0;
  for (particle_collection::const_iterator p = particles.begin(); p != particles.end(); p++) {
    float w = fmax(1e-20, exp(p->weight));
    sumXYSq += w * ((p->x - estimate.x)*(p->x - estimate.x) + (p->y - estimate.y)*(p->y - estimate.y));
    sumCos += w * cos(p->theta);
    sumSin += w * sin(p->theta);
    sumWtSq += (w - meanWeight) * (w - meanWeight);
		sumW += w;
  }
  variance.x = sumXYSq / sumW;
  sumCos /= sumW;
  sumSin /= sumW;
  float R = sqrt(sumCos*sumCos + sumSin*sumSin);
  variance.theta = (R>=1) ? 0 : sqrt(-2*log(R));  // circular std. dev.
  // variance.theta = 1 - R;  // circular variance (not equal to square of std. dev.; see Wikipedia article on Directional Statistics)
	variance.y = sumWtSq / particles.size();
	// std::cout << " sumWeight=" << sumWeight << " meanWeight=" << meanWeight << " sumWtSq=" << sumWtSq << std::endl;
  varianceValid = true;
}

void ShapeBasedParticleFilter::setWorldBounds(const Shape<PolygonData> &bounds) {
  ShapeParticleDistributionPolicy<LocalizationParticle> *dist =
    dynamic_cast<ShapeParticleDistributionPolicy<LocalizationParticle> *>(&(getResamplingPolicy()->getDistributionPolicy()));
  if ( dist != NULL )
    dist->setWorldBounds(bounds);
  else
    cout << "Error: setWorldBounds found wrong type of DistributionPolicy" << endl;
}

void ShapeBasedParticleFilter::setWorldBounds(float minX, float width, float minY, float height) {
  ShapeParticleDistributionPolicy<LocalizationParticle> *dist =
    dynamic_cast<ShapeParticleDistributionPolicy<LocalizationParticle> *>(&(getResamplingPolicy()->getDistributionPolicy()));
  if ( dist != NULL )
    dist->setWorldBounds(minX, width, minY, height);
  else
    cout << "Error: setWorldBounds found wrong type of DistributionPolicy" << endl;
}


struct compareParticles {
  bool operator ()(ShapeBasedParticleFilter::particle_type const& a, ShapeBasedParticleFilter::particle_type const& b) const {
    return (a.weight > b.weight);
  }
};

void ShapeBasedParticleFilter::deleteParticleDisplay(ShapeSpace &wShS) {
  wShS.deleteShapes<LocalizationParticleData>();
  GET_SHAPE(particles, GraphicsData, wShS);
  if ( particles.isValid() )
    wShS.deleteShape(particles);
}

void ShapeBasedParticleFilter::displayParticles(float const howmany) {
	ShapeSpace &wShS = sensorModel->getWorldShS();
	std::stable_sort(particles.begin(), particles.end(), compareParticles());
	deleteParticleDisplay(wShS);
	if ( howmany <= 0 ) return;
	unsigned int numberOfParticles;
	if ( howmany <= 1.0 )
		numberOfParticles = (unsigned int)ceil(particles.size()*howmany);
	else
		numberOfParticles = min<size_t>((size_t)howmany, particles.size());
	NEW_SHAPE(_particles, GraphicsData, new GraphicsData(wShS));
	_particles->setName("particles"); // avoid shadowing problem
	for (unsigned int i=0; i<numberOfParticles; i++)
	  _particles->add(new GraphicsData::LocalizationParticleElement("pt",particles,i));
}

void ShapeBasedParticleFilter::displayIndividualParticles(float const howmany) {
	ShapeSpace &wShS = sensorModel->getWorldShS();
	std::stable_sort(particles.begin(), particles.end(), compareParticles());
	deleteParticleDisplay(wShS);
	if ( howmany <= 0 ) return;
	unsigned int numberOfParticles;
	if ( howmany <= 1.0 )
		numberOfParticles = (unsigned int)ceil(particles.size()*howmany);
	else
		numberOfParticles = min<size_t>((size_t)howmany, particles.size());
	for (unsigned int i=0; i<numberOfParticles; i++) {
		NEW_SHAPE(pt, LocalizationParticleData, new LocalizationParticleData(wShS, particles, i));
	}
}

} // namespace
