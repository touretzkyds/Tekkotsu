#include "SiftTekkotsu.h"
#include "DualCoding/Sketch.h"
#include "Shared/newmat/newmat.h"
#include "Shared/mathutils.h" // for isnan fix

#include "SiftMatch.h"
#include "Vision/SIFT/SIFTDatabase/keypoint.h"
#include "Vision/SIFT/SIFTDatabase/keypointpair.h"
#include "Vision/SIFT/SIFTDatabase/model.h"
#include "Vision/SIFT/SIFTDatabase/object.h"
#include "Vision/SIFT/SIFTPP/sift.hpp"
#include "Vision/SIFT/SIFTPP/sift-driver.hpp"

#include <cstring>

using namespace std;
using namespace DualCoding;

SiftTekkotsu::SiftTekkotsu() : kb(), imageDatabase(), testSIFTImage(), siftImageMaxID(0),
			       argv(new char*[20]), argvCopy(argv), argc(7) {
  argv[0] = new char[1024];
  argv[1] = new char[1024];
  argv[2] = new char[1024];
  argv[3] = new char[1024];
  argv[4] = new char[1024];
  argv[5] = new char[1024];
  argv[6] = new char[1024];
}

SiftTekkotsu::~SiftTekkotsu(){
  argv = argvCopy;
  for (int i = 0; i < argc; i++)
    delete [] argv[i];
  //  free(argv);
  delete [] argv;
	
  for (unsigned int i = 0; i < imageDatabase.size(); i++){
    imageDatabase[i]->clearImage();
    delete imageDatabase[i];
  }
	
  clearTestState();
}

void SiftTekkotsu::detectKeypoints(ImageBuffer buffer, vector<keypoint*>& keys, vector< vector< vector<int> > >& gaussianSpace){
  // Call SIFT++ detection
	
  vector<double> keyValues;
  int numKeypoints;
	
  numKeypoints = 0;
	
  strcpy(argv[0], "siftpp");
  //strcpy(argv[1], PGMFileName.c_str());
  strcpy(argv[1], "bogusImage");
  strcpy(argv[2], "-o");
  strcpy(argv[3], "/tmp/siftTekkotsu.key");
  strcpy(argv[4], "--image-provided");
  strcpy(argv[5], "--verbose");
  strcpy(argv[6], "--save-gss");
	
  VL::pixel_t* tempPixelArray = (VL::pixel_t*)malloc(sizeof(float) * buffer.width * buffer.height);
  int tempPixelArrayPtr = 0;
  for (int i = 0; i < buffer.height; i++){
    for (int j = 0; j < buffer.width; j++){
      tempPixelArray[tempPixelArrayPtr] = buffer.byteArray[tempPixelArrayPtr] / 255.f;
      tempPixelArrayPtr++;
    }
  }
  VL::Sift* sift =  siftdriver(5, argv, &numKeypoints, &keyValues, gaussianSpace, tempPixelArray, buffer.width, buffer.height);
  free(tempPixelArray);
	
  delete sift;
	
	
  // Convert raw values into keypoints
  for (int i = 0; i < numKeypoints; i++){
    int j = i * (128+6);
    keypoint* key = new keypoint();
    // 		key->valid = true;
    key->imageX           = key->modelX           = keyValues[j++];
    key->imageY           = key->modelY           = keyValues[j++];
    key->imageScale       = key->modelScale       = keyValues[j++];
    key->imageOrientation = key->modelOrientation = keyValues[j++];
    key->imageIntScale = (int)keyValues[j++];
    key->imageIntOctave = (int)keyValues[j++];
    for (int jj = 0; jj < 128; jj++){
      key->desc.push_back(keyValues[j+jj]);
    }
    keys.push_back(key);
  }
	
}

void SiftTekkotsu::findInImage(ImageBuffer buffer, vector<SiftMatch*>& matchesFound, bool objectSpecified, int wantedObjectID){
  clearTestState();
  testSIFTImage.buffer.width = buffer.width;
  testSIFTImage.buffer.height = buffer.height;
  testSIFTImage.buffer.byteArray = (unsigned char*)malloc(buffer.width*buffer.height*sizeof(unsigned char));
  memcpy(testSIFTImage.buffer.byteArray, buffer.byteArray, buffer.width*buffer.height*sizeof(unsigned char));
  detectKeypoints(buffer, testSIFTImage.keypoints, testSIFTImage.gaussianSpace);
  findInImage(testSIFTImage.keypoints, matchesFound, objectSpecified, wantedObjectID);
}

void SiftTekkotsu::findInImage(vector<keypoint*>& keys, vector<SiftMatch*>& matchesFound, bool objectSpecified, int wantedObjectID){
  // Match keypoints
  size_t objectsSize = kb.objects.size();
  size_t maxNumModels1 = kb.maxNumModels+1;
  vector< vector< vector<keypoint*> > > matches;
  matches.resize(objectsSize);
  for (size_t i = 0; i < objectsSize; i++){
    matches[i].resize(maxNumModels1);
  }
  vector< vector< vector<keypoint*> > > imageKey;
  imageKey.resize(objectsSize);
  for (size_t i = 0; i < objectsSize; i++){
    imageKey[i].resize(maxNumModels1);
  }
  kb.keypointMatching(&keys, matches, imageKey, objectSpecified, wantedObjectID);
	
  // Match to models
  vector<KnowledgeBase::modelMatchingInfo*> mminfo;
  kb.modelMatching(keys.size(), matches, imageKey, mminfo);
	
  if (mminfo.size() > 0){
    for (int i = 0; i < (int)mminfo[0]->inliers->size(); i++){
      (*(mminfo[0]->inliers))[i]->getKey1()->finalMatch = (*(mminfo[0]->inliers))[i]->getKey2();
      if ((*(mminfo[0]->inliers))[i]->getKey2() != NULL) (*(mminfo[0]->inliers))[i]->getKey2()->matches.push_back((*(mminfo[0]->inliers))[i]->getKey1());
    }
  }
	
  // Convert mminfo to SiftMatch
  for (unsigned int i = 0; i < mminfo.size(); i++){
    SiftMatch *sMatch = new SiftMatch;
    convertSiftMatchFromModelMatchingInfo(*sMatch, *(mminfo[i]));
    matchesFound.push_back(sMatch);
  }
	
  // Previously, used to clean up keys here
  // But now leave this to clearTestState() instead
  // Clean up keys
  // 	for (unsigned int i = 0; i < keys.size(); i++){
  // 		delete keys[i];
  // 	}
	
  // Clean up mminfo
  for (unsigned int i = 0; i < mminfo.size(); i++){
    delete mminfo[i];
  }
	
}

void SiftTekkotsu::clearTestState(){
  testSIFTImage.clearImage();
  for (unsigned int i = 0; i < testSIFTImage.keypoints.size(); i++){
    if (testSIFTImage.keypoints[i]->finalMatch != NULL){
      for (vector<keypoint*>::iterator iter = testSIFTImage.keypoints[i]->finalMatch->matches.begin(); iter != testSIFTImage.keypoints[i]->finalMatch->matches.end(); iter++){
	if (*iter == testSIFTImage.keypoints[i]){
	  testSIFTImage.keypoints[i]->finalMatch->matches.erase(iter);
	  break;
	}
      }
    }
    delete testSIFTImage.keypoints[i];
  }
  testSIFTImage.keypoints.clear();
  testSIFTImage.gaussianSpace.clear();
  testSIFTImage.clearImage();
}

void SiftTekkotsu::convertSiftMatchFromModelMatchingInfo(SiftMatch& sMatch, KnowledgeBase::modelMatchingInfo& mminfo){
  NEWMAT::Matrix* transform = mminfo.solution;
  // recover theta from the rotation components of the transform,
  // using the fact that sin^2 + cos^2 = 1
  double scale = sqrt((*transform)(1,1) * (*transform)(1,1) + (*transform)(2,1) * (*transform)(2,1));
  double recoveredCos = (*transform)(1,1) / scale;
  double theta = acos(max(-1.0, min(1.0, recoveredCos)));
  double tx = (*transform)(3,1);
  double ty = (*transform)(4,1);
			
  if (isnan(theta)) theta = 0.0;
  if ((*transform)(2,1) / scale < 0.0) theta = -theta;
	
  // 	cout << "Conversion in progress\n";
  // 	cout << mminfo.matchedModel << endl;
  // 	cout << mminfo.matchedModel->O << endl;
	
  sMatch.objectID          = mminfo.matchedModel->O->getID();
  sMatch.objectName        = mminfo.matchedModel->O->getName();
  sMatch.modelID           = mminfo.matchedModel->getID();
  sMatch.modelName         = mminfo.matchedModel->getName();
  sMatch.probOfMatch       = mminfo.probOfMatch;
  sMatch.error             = mminfo.error;
  sMatch.scale             = scale;
  sMatch.orientation       = theta;
  sMatch.columnTranslation = tx;
  sMatch.rowTranslation    = ty;
  for (vector<keypointPair*>::const_iterator it = mminfo.inliers->begin();
       it != mminfo.inliers->end(); it++ )
    sMatch.inliers.push_back(*(*it));
	
  sMatch.computeBoundingBox();
}




/// Training API calls

// Create a new object
// Returns object ID
int SiftTekkotsu::train_addNewObject(string PGMFileName){
  unsigned int siftImageID;
  return train_addNewObject(PGMFileName, siftImageID);
}
int SiftTekkotsu::train_removeObject(string PGMFileName, int oID){
  unsigned int siftImageID;
  return train_removeObject(PGMFileName, siftImageID, oID);
}
int SiftTekkotsu::train_addNewObject(string PGMFileName, unsigned int& siftImageID){
  ImageBuffer buffer;
  PGMImg pgmImg;
  pgmImg.fromFile(PGMFileName);
  convertPGMImgToImageBuffer(pgmImg, buffer);
	
  int returnVal = train_addNewObject(buffer, siftImageID);
  free(buffer.byteArray);
  return returnVal;
}
int SiftTekkotsu::train_removeObject(string PGMFileName, unsigned int& siftImageID, int oID){
  ImageBuffer buffer;
  PGMImg pgmImg;
  pgmImg.fromFile(PGMFileName);
  convertPGMImgToImageBuffer(pgmImg, buffer);
	
  int returnVal = train_removeObject(buffer, siftImageID, oID);
  free(buffer.byteArray);
  return returnVal;
}
int SiftTekkotsu::train_addNewObject(ImageBuffer buffer){
  unsigned int siftImageID;
  return train_addNewObject(buffer, siftImageID);
}
int SiftTekkotsu::train_removeObject(ImageBuffer buffer, int oID){
  unsigned int siftImageID;
  return train_removeObject(buffer, siftImageID, oID);
}
//void KnowledgeBase::unlearn_fromModel(vector<keypoint*>* newKeys, model* bestModel, NEWMAT::Matrix* transform, double& s, double& theta, double& tx, double& ty)
int SiftTekkotsu::train_addNewObject(ImageBuffer buffer, unsigned int& siftImageID){
  // Detect keypoints from image
  SIFTImage* sImage = new SIFTImage(siftImageMaxID++);
	
  siftImageID = siftImageMaxID-1;
  imageDatabase.push_back(sImage);
  sImage->buffer.width = buffer.width;
  sImage->buffer.height = buffer.height;
  sImage->buffer.byteArray = (unsigned char*)malloc(buffer.width*buffer.height*sizeof(unsigned char));
  memcpy(sImage->buffer.byteArray, buffer.byteArray, buffer.width*buffer.height*sizeof(unsigned char));
  detectKeypoints(buffer, sImage->keypoints, sImage->gaussianSpace);
  return train_addNewObject(sImage->keypoints);
}
int SiftTekkotsu::train_removeObject(ImageBuffer buffer, unsigned int& siftImageID, int oID){
  // Detect keypoints from image
  SIFTImage* sImage = new SIFTImage(siftImageMaxID++);
	
  siftImageID = siftImageMaxID-1;
  imageDatabase.push_back(sImage);
  sImage->buffer.width = buffer.width;
  sImage->buffer.height = buffer.height;
  sImage->buffer.byteArray = (unsigned char*)malloc(buffer.width*buffer.height*sizeof(unsigned char));
  memcpy(sImage->buffer.byteArray, buffer.byteArray, buffer.width*buffer.height*sizeof(unsigned char));
  detectKeypoints(buffer, sImage->keypoints, sImage->gaussianSpace);
  return train_removeObject(sImage->keypoints, oID);
}
int SiftTekkotsu::train_addNewObject(vector<keypoint*>& keys){
  return (kb.learn_newObject(&keys))->getID();
}
int SiftTekkotsu::train_removeObject(vector<keypoint*>& keys, int oID){
  return (kb.unlearn_Object(&keys, oID))->getID();
}

// Specify object to add image to
// Return model ID
int SiftTekkotsu::train_addToObject(int objectID, string PGMFileName){
  unsigned int siftImageID;
  SiftMatch matchFound;
  return train_addToObject(objectID, PGMFileName, siftImageID, matchFound);
}
int SiftTekkotsu::train_removefromObject(int objectID, string PGMFileName) {
  cout << "Call recieved by siftTekkotsu...";	
  unsigned int siftImageID;
  SiftMatch matchFound;
  return train_removefromObject(objectID, PGMFileName, siftImageID, matchFound);
}	
int SiftTekkotsu::train_addToObject(int objectID, string PGMFileName, unsigned int& siftImageID, SiftMatch& matchFound){
  ImageBuffer buffer;
  PGMImg pgmImg;
  pgmImg.fromFile(PGMFileName);
  convertPGMImgToImageBuffer(pgmImg, buffer);
	
  int returnVal = train_addToObject(objectID, buffer, siftImageID, matchFound);
  free(buffer.byteArray);	return returnVal;
}
int SiftTekkotsu::train_removefromObject(int objectID, string PGMFileName, unsigned int& siftImageID, SiftMatch& matchFound){
  ImageBuffer buffer;
  PGMImg pgmImg;
  pgmImg.fromFile(PGMFileName);
  convertPGMImgToImageBuffer(pgmImg, buffer);
	
  int returnVal = train_removefromObject(objectID, buffer, siftImageID, matchFound);
  free(buffer.byteArray);	return returnVal;
}
int SiftTekkotsu::train_addToObject(int objectID, ImageBuffer buffer){
  unsigned int siftImageID;
  SiftMatch matchFound;
  return train_addToObject(objectID, buffer, siftImageID, matchFound);
}
int SiftTekkotsu::train_removefromObject(int objectID, ImageBuffer buffer){
  unsigned int siftImageID;
  SiftMatch matchFound;
  return train_removefromObject(objectID, buffer, siftImageID, matchFound);
}

int SiftTekkotsu::train_addToObject(int objectID, ImageBuffer buffer, unsigned int& siftImageID, SiftMatch& matchFound){
  object* wantedObject = kb.objectExists(objectID);
  if (!wantedObject) return -1;
	
  // Detect keypoints from image
  SIFTImage* sImage = new SIFTImage(siftImageMaxID++);
	
  siftImageID = siftImageMaxID-1;
  imageDatabase.push_back(sImage);
  sImage->buffer.width = buffer.width;
  sImage->buffer.height = buffer.height;
  sImage->buffer.byteArray = (unsigned char*)malloc(buffer.width*buffer.height*sizeof(unsigned char));
  memcpy(sImage->buffer.byteArray, buffer.byteArray, buffer.width*buffer.height*sizeof(unsigned char));
  detectKeypoints(buffer, sImage->keypoints, sImage->gaussianSpace);
  return train_addToObject(objectID, sImage->keypoints, wantedObject, matchFound);
}
int SiftTekkotsu::train_removefromObject(int objectID, ImageBuffer buffer, unsigned int& siftImageID, SiftMatch& matchFound){
  object* wantedObject = kb.objectExists(objectID);
  if (!wantedObject) return -1;
	
  // Detect keypoints from image
  SIFTImage* sImage = new SIFTImage(siftImageMaxID++);
	
  siftImageID = siftImageMaxID-1;
  imageDatabase.push_back(sImage);
  sImage->buffer.width = buffer.width;
  sImage->buffer.height = buffer.height;
  sImage->buffer.byteArray = (unsigned char*)malloc(buffer.width*buffer.height*sizeof(unsigned char));
  memcpy(sImage->buffer.byteArray, buffer.byteArray, buffer.width*buffer.height*sizeof(unsigned char));
  detectKeypoints(buffer, sImage->keypoints, sImage->gaussianSpace);
  return train_removefromObject(objectID, sImage->keypoints, wantedObject, matchFound);;
}
int SiftTekkotsu::train_addToObject(int /*objectID*/, vector<keypoint*>& keys, object* wantedObject, SiftMatch& matchFound){
  // Match keypoints
  size_t objectsSize = kb.objects.size();
  size_t maxNumModels1 = kb.maxNumModels+1;
  vector< vector< vector<keypoint*> > > matches;
  matches.resize(objectsSize);
  for (size_t i = 0; i < objectsSize; i++){
    matches[i].resize(maxNumModels1);
  }
  vector< vector< vector<keypoint*> > > imageKey;
  imageKey.resize(objectsSize);
  for (size_t i = 0; i < objectsSize; i++){
    imageKey[i].resize(maxNumModels1);
  }
  kb.keypointMatching(&keys, matches, imageKey, false, -1);
	
  // Match to models
  vector<KnowledgeBase::modelMatchingInfo*> mminfo;
  kb.modelMatching(keys.size(), matches, imageKey, mminfo);

  // Find best match to given object
  int bestModelIndex = -1;
  model* bestModel = NULL;
  double bestError = numeric_limits<double>::infinity();
  for (unsigned int i = 0; i < mminfo.size(); i++){
    if (mminfo[i]->matchedModel->O == wantedObject){
      bestModelIndex = (int)i;
      bestModel = mminfo[i]->matchedModel;
      bestError = mminfo[i]->error;
      break;
    }
  }
  if (bestModelIndex != -1){
    for (int i = 0; i < (int)mminfo[bestModelIndex]->inliers->size(); i++){
      (*(mminfo[bestModelIndex]->inliers))[i]->getKey1()->finalMatch = (*(mminfo[bestModelIndex]->inliers))[i]->getKey2();
      if ((*(mminfo[bestModelIndex]->inliers))[i]->getKey2() != NULL) (*(mminfo[bestModelIndex]->inliers))[i]->getKey2()->matches.push_back((*(mminfo[bestModelIndex]->inliers))[i]->getKey1());
    }
  }
	
  double threshold = 0.01 * MAXIMAGEDIM;
	
  if (bestModel == NULL || bestError > threshold ){
    // Create new model
    bestModel = kb.learn_newModel(&keys, wantedObject, mminfo);
    matchFound.objectID          = bestModel->O->getID();
    matchFound.modelID           = bestModel->getID();
    matchFound.probOfMatch       = 1.0;
    matchFound.error             = 0;
    matchFound.scale             = 1.0;
    matchFound.orientation       = 0;
    matchFound.columnTranslation = 0;
    matchFound.rowTranslation    = 0;
  }else{
    // Add to old model
    kb.learn_toModel(&keys, bestModel, mminfo[bestModelIndex]->solution);
    // Copy information to SiftMatch
    convertSiftMatchFromModelMatchingInfo(matchFound, *(mminfo[bestModelIndex]));
  }
	
  // Clean up
  for (unsigned int i = 0; i < mminfo.size(); i++){
    delete mminfo[i];
  }
	
  return bestModel->getID();
}
int SiftTekkotsu::train_removefromObject(int /*objectID*/, vector<keypoint*>& keys, object* wantedObject, SiftMatch& matchFound){
  // Match keypoints
  cout << "Final siftTekkotsu function called... \n";	
  size_t objectsSize = kb.objects.size();
  size_t maxNumModels1 = kb.maxNumModels+1;
  vector< vector< vector<keypoint*> > > matches;
  matches.resize(objectsSize);
  for (size_t i = 0; i < objectsSize; i++){
    matches[i].resize(maxNumModels1);
  }
  vector< vector< vector<keypoint*> > > imageKey;
  imageKey.resize(objectsSize);
  for (size_t i = 0; i < objectsSize; i++){
    imageKey[i].resize(maxNumModels1);
  }
  kb.keypointMatching(&keys, matches, imageKey, false, -1);
	
  // Match to models
  vector<KnowledgeBase::modelMatchingInfo*> mminfo;
  kb.modelMatching(keys.size(), matches, imageKey, mminfo);

  // Find best match to given object
  int bestModelIndex = -1;
  model* bestModel = NULL;
  double bestError = numeric_limits<double>::infinity();
  for (unsigned int i = 0; i < mminfo.size(); i++){
    if (mminfo[i]->matchedModel->O == wantedObject){
      bestModelIndex = (int)i;
      bestModel = mminfo[i]->matchedModel;
      bestError = mminfo[i]->error;
      break;
    }
  }
  if (bestModelIndex != -1){
    for (int i = 0; i < (int)mminfo[bestModelIndex]->inliers->size(); i++){
      (*(mminfo[bestModelIndex]->inliers))[i]->getKey1()->finalMatch = (*(mminfo[bestModelIndex]->inliers))[i]->getKey2();
      if ((*(mminfo[bestModelIndex]->inliers))[i]->getKey2() != NULL) (*(mminfo[bestModelIndex]->inliers))[i]->getKey2()->matches.push_back((*(mminfo[bestModelIndex]->inliers))[i]->getKey1());
    }
  }
	
  double threshold = 0.01 * MAXIMAGEDIM;
	
  if (bestModel == NULL || bestError > threshold ){
    /* Create new model
       bestModel = kb.learn_newModel(&keys, wantedObject, mminfo);
       matchFound.objectID          = bestModel->O->getID();
       matchFound.modelID           = bestModel->getID();
       matchFound.probOfMatch       = 1.0;
       matchFound.error             = 0;
       matchFound.scale             = 1.0;
       matchFound.orientation       = 0;
       matchFound.columnTranslation = 0;
       matchFound.rowTranslation    = 0;  */
  }else{
    // Add to old model
    cout << "Calling unlearn in Knowledge Base \n";
    kb.unlearn_fromModel(&keys, bestModel, mminfo[bestModelIndex]->solution);
    // Copy information to SiftMatch
    convertSiftMatchFromModelMatchingInfo(matchFound, *(mminfo[bestModelIndex]));
  }
	
  // Clean up
  for (unsigned int i = 0; i < mminfo.size(); i++){
    delete mminfo[i];
  }
  return 0;	
	
}


// Explicitly create new model for a given object
// Returns model ID
int SiftTekkotsu::train_addNewModel(int objectID, string PGMFileName){
  unsigned int siftImageID;
  return train_addNewModel(objectID, PGMFileName, siftImageID);
}
int SiftTekkotsu::train_removeModel(int objectID, string PGMFileName, int mID){
  unsigned int siftImageID;
  return train_removeModel(objectID, PGMFileName, siftImageID, mID);
}
int SiftTekkotsu::train_addNewModel(int objectID, string PGMFileName, unsigned int& siftImageID){
  ImageBuffer buffer;
  PGMImg pgmImg;
  pgmImg.fromFile(PGMFileName);
  convertPGMImgToImageBuffer(pgmImg, buffer);
	
  int returnVal = train_addNewModel(objectID, buffer, siftImageID);
  free(buffer.byteArray);
  return returnVal;
}
int SiftTekkotsu::train_removeModel(int objectID, string PGMFileName, unsigned int& siftImageID, int mID){
  ImageBuffer buffer;
  PGMImg pgmImg;
  pgmImg.fromFile(PGMFileName);
  convertPGMImgToImageBuffer(pgmImg, buffer);
	
  int returnVal = train_removeModel(objectID, buffer, siftImageID, mID);
  free(buffer.byteArray);
  return returnVal;
}
int SiftTekkotsu::train_addNewModel(int objectID, ImageBuffer buffer){
  unsigned int siftImageID;
  return train_addNewModel(objectID, buffer, siftImageID);
}
int SiftTekkotsu::train_removeModel(int objectID, ImageBuffer buffer, int mID){
  unsigned int siftImageID;
  return train_removeModel(objectID, buffer, siftImageID, mID);
}
int SiftTekkotsu::train_addNewModel(int objectID, ImageBuffer buffer, unsigned int& siftImageID){
  object* wantedObject = kb.objectExists(objectID);
  if (!wantedObject) return -1;
	
  // Detect keypoints from image
  SIFTImage* sImage = new SIFTImage(siftImageMaxID++);
	
  siftImageID = siftImageMaxID-1;
  imageDatabase.push_back(sImage);
  sImage->buffer.width = buffer.width;
  sImage->buffer.height = buffer.height;
  sImage->buffer.byteArray = (unsigned char*)malloc(buffer.width*buffer.height*sizeof(unsigned char));
  memcpy(sImage->buffer.byteArray, buffer.byteArray, buffer.width*buffer.height*sizeof(unsigned char));
  detectKeypoints(buffer, sImage->keypoints, sImage->gaussianSpace);
  return train_addNewModel(objectID, sImage->keypoints, wantedObject);
}
int SiftTekkotsu::train_removeModel(int objectID, ImageBuffer buffer, unsigned int& siftImageID, int mID){
  object* wantedObject = kb.objectExists(objectID);
  if (!wantedObject) return -1;
	
  // Detect keypoints from image
  SIFTImage* sImage = new SIFTImage(siftImageMaxID++);
	
  siftImageID = siftImageMaxID-1;
  imageDatabase.push_back(sImage);
  sImage->buffer.width = buffer.width;
  sImage->buffer.height = buffer.height;
  sImage->buffer.byteArray = (unsigned char*)malloc(buffer.width*buffer.height*sizeof(unsigned char));
  memcpy(sImage->buffer.byteArray, buffer.byteArray, buffer.width*buffer.height*sizeof(unsigned char));
  detectKeypoints(buffer, sImage->keypoints, sImage->gaussianSpace);
  return train_removeModel(objectID, sImage->keypoints, wantedObject, mID);
}
int SiftTekkotsu::train_addNewModel(int /*objectID*/, vector<keypoint*>& keys, object* wantedObject){
  // Match keypoints
  size_t objectsSize = kb.objects.size();
  size_t maxNumModels1 = kb.maxNumModels+1;
  vector< vector< vector<keypoint*> > > matches;
  matches.resize(objectsSize);
  for (size_t i = 0; i < objectsSize; i++){
    matches[i].resize(maxNumModels1);
  }
  vector< vector< vector<keypoint*> > > imageKey;
  imageKey.resize(objectsSize);
  for (size_t i = 0; i < objectsSize; i++){
    imageKey[i].resize(maxNumModels1);
  }
  kb.keypointMatching(&keys, matches, imageKey, false, -1);
	
  // Match to models
  // This step is necessary to get best matches so that we can create keygroups
  vector<KnowledgeBase::modelMatchingInfo*> mminfo;
  kb.modelMatching(keys.size(), matches, imageKey, mminfo);
	
  // Ignore matching, create new model
  int modelID = (kb.learn_newModel(&keys, wantedObject, mminfo))->getID();
	
  // Clean up
  for (unsigned int i = 0; i < mminfo.size(); i++){
    delete mminfo[i];
  }
	
  return modelID;
	
}
int SiftTekkotsu::train_removeModel(int /*objectID*/, vector<keypoint*>& keys, object* wantedObject, int mID){
  // Match keypoints
  cout << "kill me \n";	
  size_t objectsSize = kb.objects.size();
  size_t maxNumModels1 = kb.maxNumModels+1;
  vector< vector< vector<keypoint*> > > matches;
  matches.resize(objectsSize);
  for (size_t i = 0; i < objectsSize; i++){
    matches[i].resize(maxNumModels1);
  }
  vector< vector< vector<keypoint*> > > imageKey;
  imageKey.resize(objectsSize);
  for (size_t i = 0; i < objectsSize; i++){
    imageKey[i].resize(maxNumModels1);
  }
  kb.keypointMatching(&keys, matches, imageKey, false, -1);
	
  // Match to models
  // This step is necessary to get best matches so that we can create keygroups
  vector<KnowledgeBase::modelMatchingInfo*> mminfo;
  kb.modelMatching(keys.size(), matches, imageKey, mminfo);
	
  // Ignore matching, create new model
  cout << "Removing model...";	
  int modelID = (kb.unlearn_Model(&keys, wantedObject, mminfo, mID))->getID();
	
  // Clean up
  for (unsigned int i = 0; i < mminfo.size(); i++){
    delete mminfo[i];
  }
	
  return modelID;
	
}


/// Testing API calls

// Detects all instances of a given known object in image
void SiftTekkotsu::findObjectInImage(int objectID, string PGMFileName, vector<SiftMatch*>& matchesFound){
  ImageBuffer buffer;
  PGMImg pgmImg;
  pgmImg.fromFile(PGMFileName);
  convertPGMImgToImageBuffer(pgmImg, buffer);
	
  findObjectInImage(objectID, buffer, matchesFound);
  free(buffer.byteArray);
}
void SiftTekkotsu::findObjectInImage(int objectID, ImageBuffer buffer, vector<SiftMatch*>& matchesFound){
  if (!kb.objectExists(objectID)) return;
	
  findInImage(buffer, matchesFound, true, objectID);
}


// Detects all instances of all known objects in image
void SiftTekkotsu::findAllObjectsInImage(string PGMFileName, vector<SiftMatch*>& matchesFound){
  ImageBuffer buffer;
  PGMImg pgmImg;
  pgmImg.fromFile(PGMFileName);
  convertPGMImgToImageBuffer(pgmImg, buffer);
	
  findAllObjectsInImage(buffer, matchesFound);
  free(buffer.byteArray);
}

void SiftTekkotsu::findAllObjectsInImage(ImageBuffer buffer, vector<SiftMatch*>& matchesFound){
  findInImage(buffer, matchesFound, false, -1);
}



/// Naming calls

// Object name
bool SiftTekkotsu::setObjectName(int objectID, string Name){
  // Find object
  object* O = kb.objectExists(objectID);
  if (!O) return false;
	
  // Ensure uniqueness
  if (getObjectID(Name) != -1) return false;
	
  O->setName(Name);
	
  return true;
}

string SiftTekkotsu::getObjectName(int objectID){
  // Find object
  object* O = kb.objectExists(objectID);
  if (!O) {
    return "";
  }
	
  return O->getName();
}

int SiftTekkotsu::getObjectID(string Name){
  for (unsigned int i = 0; i < kb.objects.size(); i++){
    if (kb.objects[i]->getName().compare(Name) == 0){
      return (int)i;
    }
  }
  return -1;
}

// Model name
bool SiftTekkotsu::setModelName(int objectID, int modelID, string Name){
  // Find object
  object* O = kb.objectExists(objectID);
  if (!O) return false;
	
  // Find model
  model* M = O->modelExists(modelID);
  if (!M) return false;
	
  // Ensure uniqueness
  if (getModelID(objectID, Name) != -1) return false;
	
  M->setName(Name);
	
  return true;
}

string SiftTekkotsu::getModelName(int objectID, int modelID){
  // Find object
  object* O = kb.objectExists(objectID);
  if (!O) {
    return "";
  }
	
  // Find model
  model* M = O->modelExists(modelID);
  if (!M){
    return "" ;
  }
	
  return M->getName();
}

int SiftTekkotsu::getModelID(int objectID, string Name){
  // Find object
  object* O = kb.objectExists(objectID);
  if (!O) {
    return -1;
  }
	
  for (unsigned int i = 0; i < O->models.size(); i++){
    if (O->models[i]->getName().compare(Name) == 0){
      return (int)i;
    }
  }
	
  return -1;
}



/// Parameter calls

void  SiftTekkotsu::setParameter(const char* paramName, double paramVal){
  kb.setParameter(paramName, paramVal);
}

double SiftTekkotsu::getParameter(const char* paramName){
  return kb.getParameter(paramName);
}

void SiftTekkotsu::saveToFile(const std::string &filename, bool saveImages){
  clearTestState();
	
  ofstream outfile;
  outfile.open(filename.c_str());
  outfile.precision(8);
  outfile.setf(ios::fixed);
  outfile.setf(ios::showpoint);
	
  outfile << saveImages << endl;
	
  if (saveImages){
    // Save siftImageMaxID
    outfile << siftImageMaxID << endl;
    // Save imageDatabase
    outfile << imageDatabase.size() << endl;
    for (unsigned int i = 0; i < imageDatabase.size(); i++){
      // Save id
      outfile << imageDatabase[i]->id << endl;
      // Save ImageBuffer
      outfile << imageDatabase[i]->buffer.width << " " << imageDatabase[i]->buffer.height << endl;
      for (int j = 0; j < imageDatabase[i]->buffer.width*imageDatabase[i]->buffer.height; j++){
	outfile << (unsigned int)(imageDatabase[i]->buffer.byteArray[j]) << " ";
      }
      outfile << endl;
      // Save Gaussian Space
      outfile << imageDatabase[i]->gaussianSpace.size() << endl;
      for (int ii = 0; ii < (int)imageDatabase[i]->gaussianSpace.size(); ii++){
	outfile << imageDatabase[i]->gaussianSpace[ii].size() << endl;
	for (int j = 0; j < (int)imageDatabase[i]->gaussianSpace[ii].size(); j++){
	  outfile << imageDatabase[i]->gaussianSpace[ii][j].size() << endl;
	  for (int k = 0; k < (int)imageDatabase[i]->gaussianSpace[ii][j].size(); k++){
	    outfile << imageDatabase[i]->gaussianSpace[ii][j][k] << " ";
	  }
	}
	outfile << endl;
      }
      // Save keypoint IDs
      outfile << imageDatabase[i]->keypoints.size() << endl;
      for (unsigned int j = 0; j < imageDatabase[i]->keypoints.size(); j++){
	outfile << imageDatabase[i]->keypoints[j]->getID() << " ";
      }
      outfile << endl;
    }
  }
	
  // Save kb
  kb.saveToFile(outfile);
	
  outfile.close();
}

void SiftTekkotsu::loadFile(const std::string &filename){
  bool verbose = true;
	
  if (verbose) cout << "Loading SiftTekkotsu database from " << filename << endl;
	
  // Clean up state
  for (unsigned int i = 0; i < imageDatabase.size(); i++){
    imageDatabase[i]->clearImage();
    delete imageDatabase[i];
  }
  imageDatabase.clear();
  clearTestState();
	
  ifstream infile;
  infile.open(filename.c_str());
	
  // Read saveImages
  bool saveImages;
  infile >> saveImages;
	
  vector< vector<int> > keypointIDs;
  unsigned int imageDatabaseSize;
  if (saveImages){
    infile >> siftImageMaxID;
    // Read imageDatabase
    infile >> imageDatabaseSize;
    if (verbose) cout << "Number of images: " << imageDatabaseSize << endl;
    for (unsigned int i = 0; i < imageDatabaseSize; i++){
      if (verbose) cout << "Reading SIFTImage #" << i << endl;
      int id;
      infile >> id;
      SIFTImage* sImage = new SIFTImage(id);
      if (verbose) cout << "\tReading ImageBuffer\n";
      // Read ImageBuffer
      infile >> sImage->buffer.width >> sImage->buffer.height;
      if (verbose) cout << "\t\t" << sImage->buffer.height << "x" << sImage->buffer.width << endl;
      sImage->buffer.byteArray = (unsigned char*)malloc(sImage->buffer.width * sImage->buffer.height * sizeof(unsigned char));
      for (int j = 0; j < sImage->buffer.width*sImage->buffer.height; j++){
	unsigned int temp;
	infile >> temp;
	sImage->buffer.byteArray[j] = (unsigned char)temp;
      }
      if (verbose) cout << "\tReading Gaussian Space\n";
      // Read Gaussian Space
      int numOctaves;
      infile >> numOctaves;
      if (verbose) cout << "\t\tnumOctaves = " << numOctaves << endl;
      for (int ii = 0; ii < numOctaves; ii++){
	vector< vector<int> > octaveGaussianSpace;
	int numLevels;
	infile >> numLevels;
	// 			if (verbose) cout << "\t\t\tnumLevels = " << numLevels << endl;
	for (int j = 0; j < numLevels; j++){
	  vector<int> levelSpace;
	  int numVals;
	  infile >> numVals;
	  // 				if (verbose) cout << "\t\t\tnumVals = " << numVals << endl;
	  for (int k = 0; k < numVals; k++){
	    int val;
	    infile >> val;
	    levelSpace.push_back(val);
	  }
	  octaveGaussianSpace.push_back(levelSpace);
	}
	sImage->gaussianSpace.push_back(octaveGaussianSpace);
      }
      if (verbose) cout << "\tReading keypoint IDs\n";
      // Read keypoint IDs
      int numKeys;
      infile >> numKeys;
      vector<int> keys;
      for (int j = 0; j < numKeys; j++){
	int keyID;
	infile >> keyID;
	keys.push_back(keyID);
      }
      keypointIDs.push_back(keys);
      imageDatabase.push_back(sImage);
    }
  }
	
  // Read kb
  if (verbose) cout << "Reading Knowledgebase\n";
  kb.readFromFile(infile);
	
  if (verbose) cout << "Check: imageDatabase.size() == " << imageDatabase.size() << "; keypointIDs.size() == " << keypointIDs.size() << endl;
	
  // Link imageDatabase keypoints back to kb
  for (unsigned int i = 0; i < imageDatabase.size(); i++){
    if (verbose) cout << "Linking imageDatabase[" << i << "]\n";
    for (unsigned int j = 0; j < keypointIDs[i].size(); j++){
      for (unsigned int k = 0; k < kb.keys.size(); k++){
	if (keypointIDs[i][j] == kb.keys[k]->getID()){
	  // 					if (verbose) cout << "Success! " << keypointIDs[i][j] << "\n";
	  imageDatabase[i]->keypoints.push_back(kb.keys[k]);
	  break;
	}
      }
    }
  }
	
  infile.close();
	
}

/**
   Advanced image database functions
   SiftTekkotsu maintains a database of all the training images and of the most recent testing image.
   The information stored for each image include:
   1. the image itself (stored as ImageBuffer)
   2. Gaussian space generated by SIFT++
   3. keypoints detected in the image by SIFT++, and used for matching by KnowledgeBase
   The below set of functions allow the user to access the information in the image database.
**/

int SiftTekkotsu::convertSiftImageIDToIndex(int siftImageID){
  for (unsigned int i = 0; i < imageDatabase.size(); i++){
    if (imageDatabase[i]->id == siftImageID) return (int)i;
  }
  return -1;
}

/// Retrieve training image information

// getImageBuffer assumes buffer.byteArray is not allocated, and will allocate memory that callee should free
bool SiftTekkotsu::trainImage_getImageBuffer(  unsigned int siftImageID, ImageBuffer& buffer){
  int index = convertSiftImageIDToIndex(siftImageID);
  if (index == -1) return false;
	
  buffer.width  = imageDatabase[index]->buffer.width;
  buffer.height = imageDatabase[index]->buffer.height;
  buffer.byteArray = (unsigned char*)malloc(buffer.width*buffer.height*sizeof(unsigned char));
  memcpy(buffer.byteArray, imageDatabase[index]->buffer.byteArray, buffer.width*buffer.height*sizeof(unsigned char));
	
  return true;
}

bool SiftTekkotsu::trainImage_getGaussianSpace(unsigned int siftImageID, vector< vector< vector<int> > >& gaussianSpace){
  int index = convertSiftImageIDToIndex(siftImageID);
  if (index == -1) return false;
	
  gaussianSpace.clear();
  for (unsigned int i = 0; i < imageDatabase[index]->gaussianSpace.size(); i++){
    vector< vector<int> > octaveSpace;
    for (unsigned int j = 0; j < imageDatabase[index]->gaussianSpace[i].size(); j++){
      vector<int> levelSpace;
      for (unsigned int k = 0; k < imageDatabase[index]->gaussianSpace[i][j].size(); k++){
	levelSpace.push_back(imageDatabase[index]->gaussianSpace[i][j][k]);
      }
      octaveSpace.push_back(levelSpace);
    }
    gaussianSpace.push_back(octaveSpace);
  }
	
  return true;
}

bool SiftTekkotsu::trainImage_getKeypoints(    unsigned int siftImageID, vector<keypoint*>& keypoints){
  int index = convertSiftImageIDToIndex(siftImageID);
  if (index == -1) return false;
	
  keypoints.clear();
  for (unsigned int i = 0; i < imageDatabase[index]->keypoints.size(); i++){
    keypoints.push_back(imageDatabase[index]->keypoints[i]);
  }
	
  return true;
}

/// Retrieve testing image information
// getImageBuffer assumes buffer.byteArray is not allocated, and will allocate memory that callee should free
void SiftTekkotsu::testImage_getImageBuffer(  ImageBuffer& buffer){
  buffer.width  = testSIFTImage.buffer.width;
  buffer.height = testSIFTImage.buffer.height;
  buffer.byteArray = (unsigned char*)malloc(buffer.width*buffer.height*sizeof(unsigned char));
  memcpy(buffer.byteArray, testSIFTImage.buffer.byteArray, buffer.width*buffer.height*sizeof(unsigned char));
}

void SiftTekkotsu::testImage_getGaussianSpace(vector< vector< vector<int> > >& gaussianSpace){
  gaussianSpace.clear();
  for (unsigned int i = 0; i < testSIFTImage.gaussianSpace.size(); i++){
    vector< vector<int> > octaveSpace;
    for (unsigned int j = 0; j < testSIFTImage.gaussianSpace[i].size(); j++){
      vector<int> levelSpace;
      for (unsigned int k = 0; k < testSIFTImage.gaussianSpace[i][j].size(); k++){
	levelSpace.push_back(testSIFTImage.gaussianSpace[i][j][k]);
      }
      octaveSpace.push_back(levelSpace);
    }
    gaussianSpace.push_back(octaveSpace);
  }
}

void SiftTekkotsu::testImage_getKeypoints(    vector<keypoint*>& keypoints){
  keypoints.clear();
  for (unsigned int i = 0; i < testSIFTImage.keypoints.size(); i++){
    keypoints.push_back(testSIFTImage.keypoints[i]);
  }
}

ImageBuffer SiftTekkotsu::sketchToBuffer(const DualCoding::Sketch<DualCoding::uchar>& sk) {
  ImageBuffer buffer;
  buffer.height = sk.height;
  buffer.width = sk.width;
  size_t buffsize = sizeof(unsigned char)*buffer.height*buffer.width;
  cout << "buffsize = " << buffsize << endl;
  unsigned char *imgBuf = (unsigned char *)malloc(buffsize);
  sk->savePixels((char *)imgBuf,buffsize);
  buffer.byteArray = imgBuf;
  return buffer;
}
