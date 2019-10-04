#ifndef _Included_FFPlanner_h_
#define _Included_FFPlanner_h_

#include "Behaviors/BehaviorBase.h"

class FFPlanner : public BehaviorBase {
public:
  FFPlanner(const std::string &name, const std::string &_domainFileName) :
  	BehaviorBase(name), domainFileName(_domainFileName), planFileName(), result() {}
  
  void plan(const std::string &problemFileName, const std::string &_planFileName="/tmp/tekkotsu-PlannerOutput.txt");
  
  const std::string getResult() const { return result; }

protected:
  void launchFF(const std::string &problemfile);

  virtual void doEvent();

  std::string domainFileName;
  std::string planFileName;
  std::string result;

};

#endif

