#include "Physics.h"
#include "Client.h"
#include "EnvConfig.h"

using namespace std; 

Physics * Physics::inst = NULL;

size_t Physics::update() {
	size_t n;
	Physics& p = singleton();
	const EnvConfig::Environment& env = EnvConfig::singleton();
	if(p.physicsFPS==0) {
		const float t = env.timeScale/env.fps/p.stepsPerFrame;
		for(unsigned int i=0; i<p.stepsPerFrame; ++i) {
			p.step(t);
		}
		n = p.stepsPerFrame;
	} else {
		TimeET now;
		static TimeET last = now;
		float t = static_cast<float>((now - last).Value() * env.timeScale);
		// interpret negative physicsFPS to not scale time step...
		// scaling time steps keeps motion smooth, but changes physics outcome (more "accurate" in slow-mo)
		// not scaling step size will use same physics keyframes, will retain identical simulation outcome)
		float d = (p.physicsFPS<0) ? 1.0f/-p.physicsFPS : env.timeScale/p.physicsFPS;
		n = p.step(t,p.stepsPerFrame,d);
		last = now;
	}
	Client::updateGraphics();
	return n;
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
