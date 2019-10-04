#include "HolonomicMotionModel.h"

void computeHolonomicMotion(float xvel, float yvel, float avel, float time, float& xpos, float& ypos, float& apos) {
  float speed = std::sqrt(xvel*xvel + yvel*yvel);
  if (std::fabs(avel)*1e6 <= speed) {
    // straight line motion
    float distmovx = time * xvel;
    float distmovy = time * yvel;
    float c=std::cos(apos);
    float s=std::sin(apos);
    xpos += distmovx*c - distmovy*s;
    ypos += distmovx*s + distmovy*c;
    //cout << "Dist Moved X: " << distmovx << endl;  // .f floating point
    //cout << "Dist Moved Y: " << distmovy << endl;  // .f floating point
  } else {
    float radius = speed / avel; // when walking while turning, circle created...radius of that circle
    float anglturn = time * avel; // amount turned
    float heading = apos + std::atan2(yvel,xvel); // direction of instantaneous motion
    heading += anglturn/2; // plus the offset that will occur due to curvature over time
    float distmov = 2 * radius * std::sin(anglturn/2); // displacement that will result along heading
		
    //cout << "radius: " << radius << " Angle Turned: " << anglturn << " Dist Moved: " << distmov << " Heading: " << heading << endl;
    xpos += distmov*std::cos(heading);
    ypos += distmov*std::sin(heading);
    apos += anglturn;
  }
}
