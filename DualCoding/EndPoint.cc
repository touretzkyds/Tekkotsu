#include "EndPoint.h"

namespace DualCoding {

void EndPoint::checkValidity(int width, int height, int edge_thresh) {
  const bool v = ( coordX() > edge_thresh &&
									 coordX() < (width-1-edge_thresh) &&
									 coordY() > edge_thresh &&
									 coordY() < (height-1-edge_thresh) );
  setValid(v);
}

void EndPoint::updateParams(const EndPoint& other){
  ++nsamples;
  //  cout << "conf (this,other): " << nsamples << " " << other.nsamples << endl;
  //  cout << "old coords: " << coordX() << " " << coordY() << endl;
  //  cout << "other coords: " << other.coordX() << " " << other.coordY() << endl;
  coords = (coords*nsamples + other.coords*other.nsamples)/(nsamples+other.nsamples);
  rendering_valid = false;
  //  cout << "new coords: " << coordX() << " " << coordY() << endl;
}

void EndPoint::updateParams(const EndPoint& other, unsigned int num_updates){
  nsamples += num_updates-1;
  updateParams(other);
}

} // namespace
