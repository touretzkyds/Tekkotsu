#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include "Shared/fmat.h"
#include "DualCoding/ShapeLine.h"

#include "RawImagePyramid.h"

using namespace std;

class ScorePoint {
public:
  ScorePoint() : position(fmat::pack(0,0)), score(0.0f) {}
  
  fmat::Column<2> position;
  float score;
};

class FlowVector {

public:
  FlowVector() : position(fmat::pack(0,0)), flow(fmat::pack(0,0)), relevanceScore(0) {}

  fmat::Column<2> position;
  fmat::Column<2> flow;
  
  float relevanceScore;
};

class OpticalFlow {

public:

  static const unsigned int SCORE_LAYER = 0; //!< Image layer from which we select features to track

  static const unsigned int NUM_FLOW_VECTORS = 50; //!< Number of flow vectors to calculate

  //! Size of width/2, height/2 of the feature when we calculate flow.
  /*! The larger this is, the better the tracking.  Large values will severely decrease framerate, however.
   */
  static const unsigned int FLOW_WINDOW = 2;

  //! Size of width/2, height/2 of the feature when we select them for flow calculation.
  /*! It is typically safe to keep this low to save computation time.
   */
  static const unsigned int SCORE_WINDOW = 1;

  //! Number of frames we remember about the history of the robot's motion.  Used for removing outliers.
  static const unsigned int NUM_FRAMES = 10;

  //! Number of pixels per grid square when calculating minimum eigenvalues.
  static const unsigned int CANDIDATE_FEATURE_RESOLUTION = 30;

  OpticalFlow();
  OpticalFlow(OpticalFlow &other);

  virtual ~OpticalFlow() {};

  //! Comparison function to sort the flow vectors by horizontal translation.
  static bool myFlowComp(const FlowVector &v1, const FlowVector &v2);

  //! Comparison function in order to sort the flow vectors by relevance score.
  static bool myFlowComp2(const FlowVector &v1, const FlowVector &v2);

  static bool scoreComp(const ScorePoint &p1, const ScorePoint &p2) {
    return p2.score < p1.score;
  }

  //! Update the current flow vectors and increment the integrated angle.
  void updateFlow();

  //! Draw the optical flow vectors in the camera shape space.
  void drawFlow();

  void swapPyramids();

  //! Select the features that we are going to track.
  /*! The features are scored by the magnitude of their minimum eigenvalue.
      Then, the features wpith the largest score are selected.
   */
  void initializePositions();

  float getIntegratedFlow() { return integratedFlow; }

  //! Compute scores for vectors to determine outliers.
  /*! The lower the score, the better.  Uses the idea that the robot's motion is continuous.
      Gives high score to vectors that are far away from the robot's history of motion.
   */
  void computeRelevanceScores();

  //! Lucas-Kanade optic flow algorithm
  static fmat::Column<2> 
  iterativeLucasKanade(fmat::Column<2> center, fmat::Column<2> trans,
		       RawImage& img1, RawImage& img2, int window);

protected:
  float integratedFlow;

private:
  std::vector<FlowVector> flowVectors;
  std::vector<DualCoding::Shape<DualCoding::LineData> > flowVectorVisuals;
  std::vector<float> pastTranslations;

  OpticalFlow &  operator=(const OpticalFlow &other);

  RawImagePyramid pyramid1;
  RawImagePyramid pyramid2;
  RawImagePyramid *currPyramid;
  RawImagePyramid *prevPyramid;
};

/**class OpticalFlowOdometry : public OpticalFlow {
public:
  OpticalFlowOdometry() : scalingFactor(ANGULAR_RESOLUTION), lastAngle(0.0f), OpticalFlow() {}

  void update();
  float getAngle() { return lastAngle; }
  float getIntegratedAngle() { return integratedFlow / scalingFactor; }
private:
  float scalingFactor;
  float lastAngle;
  };*/

#endif
