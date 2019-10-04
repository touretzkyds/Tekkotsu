#ifdef HAVE_BULLET

#include "MotorControllers.h"

void ProportionalMotorController::updateControllerResponse(float dt) {
	float x = constraint.getValue(dt);
	float response = 0;
	if(dt>0) {
		float err = tgt-x; // find error for PID
		response = p * err;
	}
	constraint.applyResponse(dt, response);
}

void PIDMotorController::updateControllerResponse(float dt) {
	float x = constraint.getValue(dt);
	if(velocity && dt>0)
		x = (x-lastX)/dt;
	float scTgt = tgt*scale; // scale target domain (e.g. convert wheel rim dist/s to axle rad/s)
	
	float response = 0;
	
	if(verbose.size()>0)
		std::cout << verbose << " sctgt=" << scTgt << " cur=" << x;
	
	// PID calculation
	if(dt>0) {
		float err = scTgt-x; // find error for PID
		
		if(i==0) { // if i is 0, don't track sumErr
			sumErr = 0;
		} else {
			sumErr += (err+lastErr)/2 * dt;
			response += i * sumErr;
		}
		
		if(d==0) { // if d is 0, don't track derr and avgDerr
			avgDerr = 0;
		} else {
			float derr = (err - lastErr)/dt;
			avgDerr = avgDerr*derrGamma + derr*(1-derrGamma);
			response += d * avgDerr;
		}
		
		response = p * ( err + response ); // proportional control, proportional also applies to accumulated 'id' error correction (PID "standard form")
		lastErr = err;
		if(verbose.size()>0)
			std::cout << " err=" << err << " sumErr=" << sumErr << " avgDerr=" << avgDerr;
	}
	
	response += scTgt * linearff; // apply feed forward term
	
	// response has to clear minResponse, if so apply punch
	if(std::abs(response) < minResponse) {
		response = 0;
	} else {
		if(response>0)
			response = response - minResponse + punch;
		else
			response = response + minResponse - punch;
	}
	
	// response is clipped to maxResponse
	if(maxResponse<0) {
		// no-op
	} else if(std::abs(response) > maxResponse) {
		if(response>0)
			response = maxResponse;
		else
			response = -maxResponse;
	}
	
	if(friction!=0 && dt>0) {
		float spd = velocity ? x : (x - lastX) / dt;
		response -= spd*friction;
		if(verbose.size()>0)
			std::cout << " spd="<<spd;
	}
	
	if(verbose.size()>0)
		std::cout << " response="<<response <<std::endl;
	lastX = x;
	constraint.applyResponse(dt, response);
}

#endif


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
