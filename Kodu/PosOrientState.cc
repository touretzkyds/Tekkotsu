#include "Kodu/PosOrientState.h"
#include "Shared/Measures.h"

namespace Kodu {
    
    PosOrientState::PosOrientState()
      : position(),
        orientation(0.0f)
    { }

    PosOrientState::PosOrientState(const DualCoding::Point& kPosition, const float& kOrientation)
      : position(kPosition),
        orientation(AngSignPi(kOrientation))
    { }

    PosOrientState::PosOrientState(const PosOrientState& kState)
      : position(kState.position),
        orientation(kState.orientation)
    { }

    PosOrientState::~PosOrientState() {
        // no explicit implementation
    }

    PosOrientState& PosOrientState::operator=(const PosOrientState& kState) {
        if (this != &kState) {
            position = kState.position;
            orientation = kState.orientation;
        }
        return *this;
    }

    PosOrientState PosOrientState::operator-(const PosOrientState& kState) const {
        return PosOrientState(position - kState.position, orientation - kState.orientation);
    }

    bool PosOrientState::operator==(const PosOrientState& kState) const {
        return (position == kState.position && orientation == kState.orientation);
    }

    bool PosOrientState::operator!=(const PosOrientState& kState) const {
        return (!(*this == kState));
    }
}