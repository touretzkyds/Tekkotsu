#include "Shared/Measures.h"

PlaneEquation::PlaneEquation(const fmat::Column<3> &dir, float disp) : direction(dir), displacement(disp) {
	float const n = direction.norm();
	displacement /= n;
	direction /= n;
}

PlaneEquation::PlaneEquation(float a, float b, float c, float d) : direction(fmat::pack(a,b,c)), displacement(d) {
	float const n = direction.norm();
	displacement /= n;
	direction /= n;
}

std::ostream& operator<<(std::ostream &os, const PlaneEquation &p) {
	os << "PlaneEquation{" << p.getDirection() << " = " << p.getDisplacement() << "}";
	return os;
}

//================================================================

void AngPi::normalize() {
	if ( value >= Pi) {
		if ( value < TwoPi ) {
			value -= Pi;
		} else { // use fmod only if necessary
			value = std::fmod(value,Pi);
		}
	} else if ( value < 0 ) {
		if ( value > -Pi ) {
			value += Pi;
		} else { // use fmod only if necessary
			value = std::fmod(value,Pi);
			if(value<0) // this is almost always true except when value==0, leave it there
				value += Pi;
		}
	}
}


AngPi angdist(AngPi const &arg1, AngPi const &arg2) {
	AngPi diff = arg1.value - arg2.value;
	if ( diff > Pi/2 )
		diff = Pi - diff;
	return diff;
}

void AngTwoPi::normalize() {
	if ( value >= TwoPi) {
		if ( value < 2*TwoPi ) {
			value -= TwoPi;
		} else { // use fmod only if necessary
			value = std::fmod(value,TwoPi);
		}
	} else if ( value < 0 ) {
		if ( value > -TwoPi ) {
			value += TwoPi;
		} else { // use fmod only if necessary
			value = std::fmod(value,TwoPi);
			if(value<0) // this is almost always true except when value==0, leave it there
				value += TwoPi;
		}
	}
}		     

AngPi angdist(AngTwoPi const &arg1, AngTwoPi const &arg2) {
	AngTwoPi diff = arg1.value - arg2.value;
	if ( diff > Pi )
		diff = TwoPi - diff;
	return AngPi(diff);
}

void AngSignPi::normalize() {
	if ( value > Pi ) {
		if ( value <= TwoPi ) {
			value -= TwoPi;
		} else { // use fmod only if necessary
			value = std::fmod(value,TwoPi);
			if( value > Pi )
				value -= TwoPi;
		}
	} else if ( value <= -Pi ) {
		if ( value >= -TwoPi ) {
			value += TwoPi;
		} else { // use fmod only if necessary
			value = std::fmod(value,TwoPi);
			if( value <= -Pi )
				value += TwoPi;
		}
	}
}		     

AngPi angdist(AngSignPi const &arg1, AngSignPi const &arg2) {
	AngSignPi diff = arg1.value - arg2.value;
	if ( diff > Pi )
		diff = TwoPi - diff;
	return AngPi(diff);
}

void AngSignTwoPi::normalize() {
	if ( value >= TwoPi) {
		if ( value < 2*TwoPi )
			value -= TwoPi;
		else // use fmod only if necessary
			value = std::fmod(value,TwoPi);
	} else if ( value < -TwoPi ) {
		if ( value > -2*TwoPi )
			value += TwoPi;
		else  // use fmod only if necessary
			value = std::fmod(value,TwoPi);
	}
}
