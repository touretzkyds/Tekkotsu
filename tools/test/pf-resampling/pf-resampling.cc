#include "Shared/ParticleFilter.h"
#include "Localization/LocalizationParticle.h"
#include "Shared/zignor.h"
#include <iostream>
#include <cmath>
#include <sys/time.h>

namespace project_get_time {
	unsigned int simulation_time=0;
	unsigned int (*get_time_callback)()=NULL;
}

using namespace std;
using namespace project_get_time;

class Particle : public ParticleBase<Particle> {
public:
	typedef class ParticleDistributionPolicy DistributionPolicy;
	Particle() : ParticleBase<Particle>(), pos() { }
	float sumSqErr(const Particle& p) const { 
		float x=p.pos-pos; return x*x;
	}
	float pos;
};

typedef ParticleFilter<Particle> PF;
typedef PF::LowVarianceResamplingPolicy Resampler;

class ParticleDistributionPolicy : public PF::DistributionPolicy {
	typedef PF::index_t index_t;
	virtual void randomize(particle_type* begin, index_t num) {
		particle_type* end=&begin[num]; 
		cerr << "RANDOMIZED" << endl;
		while(begin!=end)
			(begin++)->pos = (float(random())/(-1U>>1)*M_PI*2);
	}
	virtual void jiggle(float var, particle_type* begin, index_t num) {
		particle_type* end=&begin[num]; 
		while(begin!=end) {
			begin->pos = (begin->pos+DRanNormalZig32()*.1*var);
			++begin;
		}
	}
};

class SinSensorModel : public PF::SensorModel {
	typedef PF::particle_collection particle_collection;
	typedef PF::index_t index_t;
	virtual void evaluate(particle_collection& particles, index_t& bestIndex) {
		for(index_t i=0; i<particles.size(); ++i) {
			if(particles[i].pos<0 || particles[i].pos>M_PI*2)
				particles[i].weight += -FLT_MAX;
			else {
				particles[i].weight += std::log(std::sin(particles[i].pos)/2+.5);
				//particles[i].weight += std::log((std::cos(particles[i].pos)/2+.5)*(1-particles[i].pos/M_PI/10));
				if(particles[i].weight>particles[bestIndex].weight)
					bestIndex=i;
			}
		}
	}
};

void doPlot(unsigned int i, const PF::particle_collection& p) {
	cout << "plot '-' smooth frequency title 'distribution " << i << "' with boxes" << endl;
	for(unsigned int i=0; i<p.size(); i++)
		cout << floor(p[i].pos*10)/10 << " 1" << endl;
	cout << "e" << endl << "pause 1" << endl;
}

int main(int argc, char** argv) {
	struct timeval tp;
	gettimeofday(&tp,NULL);
	tp.tv_sec+=tp.tv_usec;
	cerr << "seeds are " << tp.tv_sec << ", " << tp.tv_usec << endl;
	RanNormalSetSeedZig32((int*)&tp.tv_sec,tp.tv_usec);
	
	PF filter(1000);
	cout << "# output can be piped into gnuplot to produce plots" << endl;
	cout << "set yrange [0:]" << endl;
	cout << "set xrange [0:3*pi]" << endl;
	doPlot(0,filter.getParticles());
	SinSensorModel eval;
	Resampler& r = dynamic_cast<Resampler&>(*filter.getResamplingPolicy());
	for(unsigned int i=1; i<=15; i++) {
		for(unsigned int j=0; j<1; j++) {
			r.varianceScale=std::pow(0.75f,(float)j);
			filter.updateSensors(eval);
		}
		doPlot(i,filter.getParticles());
	}
	cerr << "\nPipe output to gnuplot to see graphs for each iteration.\n\n";
	return 0;
}
