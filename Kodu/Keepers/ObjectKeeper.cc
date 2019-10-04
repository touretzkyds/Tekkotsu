#include "Kodu/Keepers/ObjectKeeper.h"

namespace Kodu {
	DualCoding::ShapeRoot ObjectKeeper::tempObject;
	bool ObjectKeeper::isValid = false;
	DualCoding::ShapeRoot ObjectKeeper::invalidObject;
}
