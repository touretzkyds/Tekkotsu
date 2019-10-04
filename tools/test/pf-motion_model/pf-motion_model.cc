#include <iostream>
#include <cmath>
#include "Localization/HolonomicMotionModel.h"

namespace project_get_time {
	unsigned int simulation_time=0;
	unsigned int (*get_time_callback)()=NULL;
}

using namespace std;
using namespace project_get_time;

int main(int argc, char** argv) {
	const float speed=10;
	const float bearing=M_PI/3;
	const float avel=M_PI;
	const float time=1000;
	const float dt=50;
	
	unsigned int& t=simulation_time;
	float x=0, y=0, a=0;
	HolonomicMotionModel<LocalizationParticle> mm;
	
	cout.precision(3);
	cout.setf(ios::fixed);
	
	// straight line (blue segment in output.png)
	mm.setVelocity((speed/avel)*cos(bearing-M_PI/2), (speed/avel)*sin(bearing-M_PI/2), 0, t);
	for(; t<time*2; t+=dt*2) {
		mm.getPosition(x,y,a,t);
		cout << x << ' ' << y << ' ' << a << ' ' << t << ' ' << 0 << endl;
	}
    mm.getPosition(x,y,a,t);
    cout << x << ' ' << y << ' ' << a << ' ' << t << ' ' << 0 << endl;
	
	// half circle curving left/counter-clockwise (pink segment in output.png)
	mm.setVelocity(speed*cos(bearing), speed*sin(bearing), avel, t);
	for(; t<time*3; t+=dt) {
		mm.getPosition(x,y,a,t);
		cout << x << ' ' << y << ' ' << a << ' ' << t << ' ' << 1 << endl;
	}
    mm.getPosition(x,y,a,t);
    cout << x << ' ' << y << ' ' << a << ' ' << t << ' ' << 1 << endl;
    
    // full circle turning right/clockwise (red segment)
	mm.setVelocity(speed*cos(bearing), speed*sin(bearing), -avel, t);
	for(; t<time*5; t+=dt) {
		mm.getPosition(x,y,a,t);
		cout << x << ' ' << y << ' ' << a << ' ' << t << ' ' << 2 << endl;
	}
    mm.getPosition(x,y,a,t);
    cout << x << ' ' << y << ' ' << a << ' ' << t << ' ' << 2 << endl;
	
	// half circle turning left/counter-clockwise (green segment)
	mm.setVelocity(speed*cos(bearing), speed*sin(bearing), avel, t);
	for(; t<time*6; t+=dt) {
		mm.getPosition(x,y,a,t);
		cout << x << ' ' << y << ' ' << a << ' ' << t << ' ' << 3 << endl;
	}
    mm.getPosition(x,y,a,t);
    cout << x << ' ' << y << ' ' << a << ' ' << t << ' ' << 3 << endl;
	
	return 0;
}
