#include "Shared/Measures.h"
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <limits>

using namespace std;

const int RANGE=9*16;
const int DIV=16;

int main(int argc, const char* argv[]) {
	for(int i=-RANGE; i<RANGE; i++) {
		float x = M_PI*i/DIV;
		AngPi a(x);
		AngTwoPi a2(x);
		AngSignPi as(x);
		float rnd = std::fmod(x,static_cast<float>(M_PI));
		cout << x << ' ' << a << ' ' << a2 << ' ' << as << ' ' << rnd << '\n';
	}
	
	return EXIT_SUCCESS;
}