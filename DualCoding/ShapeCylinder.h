//-*-c++-*-
#ifndef _SHAPECYLINDER_H_
#define _SHAPECYLINDER_H_

#include "ShapeRoot.h"
#include "CylinderData.h"

namespace DualCoding {
	
	class ShapeSpace;
	
	template<>
	class Shape<CylinderData> : public ShapeRoot {
	public:
		SHAPESTUFF_H(CylinderData);   // defined in ShapeRoot.h
	};
	
} // namespace

#endif // _SHAPECYLINDER_H_
