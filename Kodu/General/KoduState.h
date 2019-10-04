#ifndef KODU_STATE_H_
#define KODU_STATE_H_
#include "Events/EventRouter.h"
#include "Shared/Gamepad.h"
#include <iostream>
#include <vector>

namespace Kodu {


class KoduState {
public:
  struct utterance {
      int hostAddr;
      std::string phrase;
  };

  KoduState() :
    abutton(0.0), bbutton(0.0), xbutton(0.0), ybutton(0.0),
    leftbumper(0.0), rightbumper(0.0), joystickx(0.0), joysticky(0.0), joystick_fullstop(false),
		heard_utt()
    {}

  void addUtterance(utterance uttered) {
      heard_utt.push_back(uttered);
  }

  void clearUtterances() {
      heard_utt.clear();
  }

  void update(int generatoregid, int sourceid, float magnitude);

  // Gamepad State
  float abutton;
  float bbutton;
  float xbutton;
  float ybutton;
  float leftbumper;
  float rightbumper;
  float joystickx;
  float joysticky;

  bool joystick_fullstop;

  std::vector<utterance> heard_utt; //!< Utterances heard since last rule cycle
};

} // namespace

#endif
