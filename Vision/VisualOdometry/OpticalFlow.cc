#include "OpticalFlow.h"
#include "DualCoding/DualCoding.h"

using namespace DualCoding;

OpticalFlow::OpticalFlow() : integratedFlow(0.0f), flowVectors(NUM_FLOW_VECTORS),
			     flowVectorVisuals(NUM_FLOW_VECTORS), pastTranslations(NUM_FRAMES),
			     pyramid1(), pyramid2(), currPyramid(&pyramid1), prevPyramid(NULL) {}

bool OpticalFlow::myFlowComp(const FlowVector &v1, const FlowVector &v2) {
  return (v1.flow(0,0) < v2.flow(0,0));
}

bool OpticalFlow::myFlowComp2(const FlowVector &v1, const FlowVector &v2) {
  return (v1.relevanceScore < v2.relevanceScore);
}


void OpticalFlow::computeRelevanceScores() {
  for(unsigned int i = 0; i < NUM_FLOW_VECTORS; i++) {
    float score = 0;
    for(unsigned int j = 0; j < NUM_FRAMES; j++) {
      score += (1.0f/((float)(j+1))) * fabs(flowVectors[i].flow(0,0) - pastTranslations[j]);
    }
    flowVectors[i].relevanceScore = score;
  }
}

void OpticalFlow::updateFlow() {
  currPyramid->loadFromRawY();
  if(prevPyramid == NULL) {
    swapPyramids();
    return;
  }
  
  initializePositions();
  for(unsigned int L = RawImagePyramid::NUM_PYRAMID_LAYERS; L > 0; L--) {
    for(unsigned int i = 0; i < NUM_FLOW_VECTORS; i++) {
      fmat::Column<2> flow = iterativeLucasKanade(flowVectors[i].position/pow(2,L-1),
						  flowVectors[i].flow,
						  (*prevPyramid)[L-1],
						  (*currPyramid)[L-1],
						  FLOW_WINDOW);
      if(L != 0)
	flowVectors[i].flow = 2 * (flowVectors[i].flow + flow);
      else
	flowVectors[i].flow = (flowVectors[i].flow + flow);
    }
  }

  computeRelevanceScores();
  sort(flowVectors.begin(), flowVectors.end(), myFlowComp2);
  sort(flowVectors.begin(), flowVectors.begin()+(2*NUM_FLOW_VECTORS/3), myFlowComp);

  float trans = flowVectors[2*NUM_FLOW_VECTORS/6].flow(0,0);
  integratedFlow += trans;
  
  for(int i = NUM_FRAMES-1; i>=1; i--)
    pastTranslations[i] = pastTranslations[i-1];
  pastTranslations[0] = trans;
  // drawFlow();
  swapPyramids();
}

void OpticalFlow::swapPyramids() {
  if(currPyramid == &pyramid1) {
    currPyramid = &pyramid2;
    prevPyramid = &pyramid1;
  }
  else {
    currPyramid = &pyramid1;
    prevPyramid = &pyramid2;
  }
}

void OpticalFlow::drawFlow() {
  static Sketch<uchar> rawY(VRmixin::sketchFromRawY(), "rawY", true);

  rawY = VRmixin::sketchFromRawY();
  
  int i = 0;
  SHAPEVEC_ITERATE(flowVectorVisuals, LineData, line) {
    fmat::Column<2> startPt = 2*flowVectors[i].position;
    fmat::Column<2> endPt = 2*flowVectors[i].position + flowVectors[i].flow;
    EndPoint startPt2 = EndPoint(startPt(0,0), startPt(1,0));
    EndPoint endPt2 = EndPoint(endPt(0,0),endPt(1,0));
    if(!line.isValid()) {
      LineData line2(VRmixin::camShS,startPt2,endPt2);
      line = Shape<LineData>(line2);
    } else {
      line->setEndPts(startPt2, endPt2);
    }
    i++;
  }}
}

void OpticalFlow::initializePositions() {
  unsigned int j = 0;
  unsigned int const width = (*prevPyramid)[SCORE_LAYER].getWidth();
  unsigned int const height = (*prevPyramid)[SCORE_LAYER].getHeight();
  unsigned int const xinc = width/CANDIDATE_FEATURE_RESOLUTION;
  unsigned int const yinc = height/CANDIDATE_FEATURE_RESOLUTION;
  unsigned int const numFeatures = (width/xinc) * (height/yinc);
  std::vector<ScorePoint> eigens(numFeatures);

  for(unsigned int x = 0; x < width; x+=xinc) {
    for(unsigned int y = 0; y < height; y+=yinc) {
      if ( j >= numFeatures )
	std::cout << "*** j = " << j << std::endl;
      eigens[j].position(0,0) = x;
      eigens[j].position(1,0) = y;
      eigens[j].score = (*prevPyramid)[SCORE_LAYER].imageScore(x,y,SCORE_WINDOW);
      // std::cout << "  " << j << ":  x=" << x << "  y=" << y << "  score=" << eigens[j].score << std::endl;
      j++;
    }
  }

  sort(eigens.begin(), eigens.end(), scoreComp);
  if(eigens[0].score == 0) {
    cout << "VISUAL ODOMETRY FAILED" << endl;
    return;
  }

  for(unsigned int i = 0; i < NUM_FLOW_VECTORS; i++) {
    flowVectors[i].position = eigens[i].position;
    flowVectors[i].flow = fmat::pack(0,0);
    }
}

fmat::Column<2>
OpticalFlow::iterativeLucasKanade(fmat::Column<2> center, fmat::Column<2> trans,
				  RawImage& img1, RawImage& img2, int window) {
  fmat::Matrix<2,2> G = img1.spatialGradientMatrix(center(0,0),center(1,0),window);
  if(fmat::det(G) == 0)
    return fmat::pack(0,0);
  fmat::Matrix<2,2> Ginv = fmat::invert(G);

  const unsigned int MAX_ITERATIONS = 5;
  const float ACCURACY_THRESHOLD = 0.1f;
  fmat::Column<2> opticalFlow;
  for(unsigned int i = 0; i < MAX_ITERATIONS; i++) {
    fmat::Column<2> b;
    for(float x = center(0,0) - window; x <= center(0,0)+window; x++) {
      for(float y = center(1,0) - window; y <= center(1,0)+window; y++) {
	float delta = img1(x,y) - img2(x + trans(0,0) + opticalFlow(0,0),
				       y + trans(1,0) + opticalFlow(1,0));
	fmat::Column<2> grad = img1.gradient(x,y);

	b += grad * delta;
      }
    }
    
    fmat::Column<2> gamma = Ginv * b;
    opticalFlow += gamma;

    //    if(((gamma(0,0) * gamma(0,0)) + (gamma(1,0)*gamma(1,0))) < ACCURACY_THRESHOLD)
    //      break;
    if( (gamma(0,0) * gamma(0,0)) + (gamma(1,0) * gamma(1,0)) < ACCURACY_THRESHOLD )
      break;
  }

  return opticalFlow;
}

/*   Josh's original version
fmat::Column<2>
OpticalFlow::iterativeLucasKanade(fmat::Column<2> center, fmat::Column<2> trans,
				  RawImage& img1, RawImage& img2, int window) {
  const unsigned int MAX_ITERATIONS = 5;
  const float ACCURACY_THRESHOLD = 0.1f;
  fmat::Column<2> opticalFlow;
  for(unsigned int i = 0; i < MAX_ITERATIONS; i++) {
    fmat::Column<2> b;
    fmat::Matrix<2,2> G = img1.spatialGradientMatrix(center(0,0),center(1,0),window);
    for(float x = center(0,0) - window; x <= center(0,0)+window; x++) {
      for(float y = center(1,0) - window; y <= center(1,0)+window; y++) {
	float delta = img1(x,y) - img2(x + trans(0,0) + opticalFlow(0,0),
				       y + trans(1,0) + opticalFlow(1,0));
	fmat::Column<2> grad = img1.gradient(x,y);

	b += grad * delta;
      }
    }
    
    if(fmat::det(G) == 0) {
      cout << "========Bad Point========" << endl;
      return fmat::pack(0,0);
    }

    G = fmat::invert(G);
    
    fmat::Column<2> gamma = G * b;

    opticalFlow += gamma;

    //    if(((gamma(0,0) * gamma(0,0)) + (gamma(1,0)*gamma(1,0))) < ACCURACY_THRESHOLD)
    //      break;
    if( (gamma(0,0) * gamma(0,0)) + (gamma(1,0) * gamma(1,0)) < ACCURACY_THRESHOLD )
      break;
  }

  return opticalFlow;
}

*/
