#ifndef Included_FFPlanNode_h_
#define Included_FFPlanNode_h_

#include "Behaviors/StateNode.h"

class FFPlanNode : public StateNode {
public:
  FFPlanNode(const std::string &nodename, const std::string _domainFileName) :
    StateNode(nodename), domainFileName(_domainFileName), problemFileName(), planFileName("/tmp/tekkotsu-Plan.txt"), result() {}
  
  virtual void postStart();
  virtual void doEvent();
  
  void plan();
  const std::string getResult() const { return result; }

protected:
  void launchFF();
  std::string domainFileName;
  std::string problemFileName;
  std::string planFileName;
  std::string result;
};

#endif

