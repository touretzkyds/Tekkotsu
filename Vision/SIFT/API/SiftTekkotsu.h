#ifndef __SIFTTEKKOTSU_H
#define __SIFTTEKKOTSU_H

#include "Vision/SIFT/SIFTDatabase/KnowledgeBase.h"
#include "Vision/SIFT/ImageHandling/SIFTImage.h"
#include <vector>
#include <string>

class SIFTImage;
class SiftMatch;

namespace DualCoding {
	typedef unsigned char uchar;
	template<typename T> class Sketch;
}

/**
   Main API class
   All API function calls are implemented within this class
   Look at APIExample.cc for an example of how to use the class to perform API calls
**/
class SiftTekkotsu{
private:
  KnowledgeBase kb;
  std::vector<SIFTImage*> imageDatabase;
public:
  SIFTImage testSIFTImage;
private:
  int siftImageMaxID;
		
  char** argv, **argvCopy;
  int    argc;
		
  // Detection of keypoints from image using SIFT++
public:
  void detectKeypoints(ImageBuffer buffer, std::vector<keypoint*>& keys, std::vector< std::vector< std::vector<int> > >& gaussianSpace);
private:
  void detectKeypoints(std::string PGMFileName, ImageBuffer* buffer, 
		       std::vector<keypoint*>& keys, std::vector< std::vector< std::vector<int> > >& gaussianSpace, bool imageProvided);

public:		
  void findInImage(ImageBuffer buffer, std::vector<SiftMatch*>& matchesFound, bool objectSpecified, int wantedObjectID);
  void findInImage(std::vector<keypoint*>& keys, std::vector<SiftMatch*>& matchesFound, bool objectSpecified, int wantedObjectID);
private:
  void clearTestState();
		
  int train_addNewObject(std::vector<keypoint*>& keys);
  int train_removeObject(std::vector<keypoint*>& keys, int oID);
  int train_addToObject(int objectID, std::vector<keypoint*>& keys, object* wantedObject, SiftMatch& matchFound);
  int train_removefromObject(int objectID, std::vector<keypoint*>& keys, object* wantedObject, SiftMatch& matchFound);
  int train_addNewModel(int objectID, std::vector<keypoint*>& keys, object* wantedObject);
  int train_removeModel(int objectID, std::vector<keypoint*>& keys, object* wantedObject, int mID);
  void convertSiftMatchFromModelMatchingInfo(SiftMatch& sMatch, KnowledgeBase::modelMatchingInfo& mminfo);
		
  int convertSiftImageIDToIndex(int siftImageID);
		
public:
  SiftTekkotsu();
  ~SiftTekkotsu();
		
  /// Training API calls
		
  // Create a new object
  // Returns object ID
  int train_addNewObject(std::string PGMFileName);             
  int train_addNewObject(std::string PGMFileName, unsigned int& siftImageID);
  int train_addNewObject(ImageBuffer buffer);
  int train_addNewObject(ImageBuffer buffer, unsigned int& siftImageID);
		
  int train_removeObject(std::string PGMFileName, int oID);             
  int train_removeObject(std::string PGMFileName, unsigned int& siftImageID, int oID);
  int train_removeObject(ImageBuffer buffer, int oID);
  int train_removeObject(ImageBuffer buffer, unsigned int& siftImageID, int oID);
  // Specify object to add image to
  // Return model ID
  int train_addToObject(int objectID, std::string PGMFileName);
  int train_addToObject(int objectID, std::string PGMFileName, unsigned int& siftImageID, SiftMatch& matchFound);
  int train_addToObject(int objectID, ImageBuffer buffer);		
  int train_addToObject(int objectID, ImageBuffer buffer, unsigned int& siftImageID, SiftMatch& matchFound);
		
  int train_removefromObject(int objectID, std::string PGMFileName);
  int train_removefromObject(int objectID, std::string PGMFileName, unsigned int& siftImageID, SiftMatch& matchFound);
  int train_removefromObject(int objectID, ImageBuffer buffer);		
  int train_removefromObject(int objectID, ImageBuffer buffer, unsigned int& siftImageID, SiftMatch& matchFound);
  // Explicitly create new model for a given object
  // Returns model ID
  int train_addNewModel(int objectID, std::string PGMFileName);		
  int train_addNewModel(int objectID, std::string PGMFileName, unsigned int& siftImageID);
  int train_addNewModel(int objectID, ImageBuffer buffer);		
  int train_addNewModel(int objectID, ImageBuffer buffer, unsigned int& siftImageID);
		
  int train_removeModel(int objectID, std::string PGMFileName, int mID);		
  int train_removeModel(int objectID, std::string PGMFileName, unsigned int& siftImageID, int mID);
  int train_removeModel(int objectID, ImageBuffer buffer, int mID);		
  int train_removeModel(int objectID, ImageBuffer buffer, unsigned int& siftImageID, int mID);		
		
  /// Testing API calls
		
  // Detects all instances of a given known object in image
  void findObjectInImage(int objectID, std::string PGMFileName, std::vector<SiftMatch*>& matchesFound);
  void findObjectInImage(int objectID, ImageBuffer buffer, std::vector<SiftMatch*>& matchesFound);
		
  // Detects all instances of all known objects in image
  void findAllObjectsInImage(std::string PGMFileName, std::vector<SiftMatch*>& matchesFound);
  void findAllObjectsInImage(ImageBuffer buffer, std::vector<SiftMatch*>& matchesFound);
		
		
  /// Naming calls
		
  // Object name
  bool   setObjectName(int objectID, std::string Name);
  std::string getObjectName(int objectID);
  int    getObjectID  (std::string Name);
		
  // Model name
  bool   setModelName(int objectID, int modelID, std::string Name);
  std::string getModelName(int objectID, int modelID);
  int    getModelID  (int objectID, std::string Name);
		
		
  /// Parameter calls
		
  void   setParameter(const char* paramName, double paramVal);
  double getParameter(const char* paramName);
		
  /// List of parameters
  /*
    probOfMatch     - a possible transformation is rejected if the probability of matching (computed by Bayesian analysis) is below probofMatch
    errorThreshold  - a possible transformation is rejected if the error of match is above errorThreshold
  */
		
		
  /// File input and output
  void saveToFile(const std::string &filename, bool saveImages);
  void loadFile(const std::string &filename);
  static ImageBuffer sketchToBuffer(const DualCoding::Sketch<DualCoding::uchar>& sk);

  /**
     Advanced image database functions
     SiftTekkotsu maintains a database of all the training images and of the most recent testing image.
     The information stored for each image include:
     1. the image itself (stored as ImageBuffer)
     2. Gaussian space generated by SIFT++
     3. keypoints detected in the image by SIFT++, and used for matching by KnowledgeBase
     The below set of functions allow the user to access the information in the image database.
  **/
		
		
  /// Retrieve training image information
  // getImageBuffer assumes buffer.byteArray is not allocated, and will allocate memory that callee should free
  bool trainImage_getImageBuffer(  unsigned int siftImageID, ImageBuffer& buffer);
  bool trainImage_getGaussianSpace(unsigned int siftImageID, std::vector< std::vector< std::vector<int> > >& gaussianSpace);
  bool trainImage_getKeypoints(    unsigned int siftImageID, std::vector<keypoint*>& keypoints);
		
  /// Retrieve testing image information
  // getImageBuffer assumes buffer.byteArray is not allocated, and will allocate memory that callee should free
  void testImage_getImageBuffer(  ImageBuffer& buffer);
  void testImage_getGaussianSpace(std::vector< std::vector< std::vector<int> > >& gaussianSpace);
  void testImage_getKeypoints(    std::vector<keypoint*>& keypoints);
		
  //A test function that opens a file. To be deleted as soon as possible
  bool loadFile1(const char* filename);

private:
  SiftTekkotsu(const SiftTekkotsu&);
  SiftTekkotsu& operator=(const SiftTekkotsu&);
};

#endif
