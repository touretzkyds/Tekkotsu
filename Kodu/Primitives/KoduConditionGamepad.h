#ifndef KODU_CONDITION_GAMEPAD_H_
#define KODU_CONDITION_GAMEPAD_H_
#include "Kodu/Primitives/KoduCondition.h"
#include <iostream>

namespace Kodu {
  class KoduWorld;
  class KoduConditionGamepad : public KoduCondition {
  public:
    KoduConditionGamepad(int src,float value)
    : KoduCondition("KoduConditionGamepad", KoduCondition::CT_GAMEPAD),
      input(src),
      position(value)
    { }
     
    ~KoduConditionGamepad() { }

    KoduConditionGamepad& operator=(const KoduConditionGamepad& kCondition){
      if (this != &kCondition) {
				KoduCondition::operator=(kCondition);
				input = kCondition.input;
				position = kCondition.position;
      }
      return *this;
    }
    
  virtual bool evaluate(const KoduWorld&);
  
  private:
    int input;
    float position;
  };
}

#endif
