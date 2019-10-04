#ifndef PERCEPTION_SEARCH_H_
#define PERCEPTION_SEARCH_H_

// Tekkotsu Library
#include "DualCoding/AgentData.h"
#include "DualCoding/ShapeAgent.h"
#include "DualCoding/ShapeCylinder.h"
#include "DualCoding/ShapeFuns.h"
#include "DualCoding/ShapeRoot.h"
#include "DualCoding/VRmixin.h"
#include "Crew/MapBuilder.h"
#include "Shared/Measures.h"
using namespace DualCoding;

// C++ Library
#include <cstdlib>

namespace Kodu {

    //! Specifies the four main directions (north, east, south, and west)
    enum Direction_t {
        DT_EAST         = 0,
        DT_NORTH        = 1L << 1,
        DT_SOUTH        = 1L << 2,
        DT_WEST         = 1L << 3
    };

    //! Logical OR operation
    inline
    Direction_t operator|(Direction_t dt1, Direction_t dt2) {
        return Direction_t(static_cast<int>(dt1) | static_cast<int>(dt2));
    }

    //! Logical AND operation
    inline
    Direction_t operator&(Direction_t dt1, Direction_t dt2) {
        return Direction_t(static_cast<int>(dt1) & static_cast<int>(dt2));
    }

    //! Specifies what region(s) the agent should focus on when using conditions such as see and bump
    enum SearchLocation_t {
        SL_UNRESTRICTED     = 0,
        SL_BEHIND           = 1L << 1,
        SL_IN_FRONT         = 1L << 2,
        SL_TO_LEFT          = 1L << 3,
        SL_TO_RIGHT         = 1L << 4,
        SL_CLOSE_BY         = 1L << 5,
        SL_FAR_AWAY         = 1L << 6
    };
    
    //! Logical OR operation
    inline
    SearchLocation_t operator|(SearchLocation_t rs1, SearchLocation_t rs2) {
        return SearchLocation_t(static_cast<int>(rs1) | static_cast<int>(rs2));
    }

    //! Logicial OR Equals operation
    inline
    SearchLocation_t operator|=(SearchLocation_t rs1, SearchLocation_t rs2) {
        return (rs1 = rs1 | rs2);
    }

    //! Logical AND operation
    inline
    SearchLocation_t operator&(SearchLocation_t rs1, SearchLocation_t rs2) {
        return SearchLocation_t(static_cast<int>(rs1) & static_cast<int>(rs2));
    }

    class HasAprilTagID : public UnaryShapeRootPred {
    public:
        HasAprilTagID(int aprilTagId)
          : UnaryShapeRootPred(),
            id(aprilTagId)
        { }

        ~HasAprilTagID() { }

        bool operator()(const ShapeRoot&) const;

    private:
        int id;
    };

    class IsMatchForTargetObject : public UnaryShapeRootPred {
    public:
        IsMatchForTargetObject(const ShapeRoot& kTargetObject)
          : UnaryShapeRootPred(),
            targetObject(kTargetObject)
        { }

        ~IsMatchForTargetObject() { }

        bool operator()(const ShapeRoot&) const;

    private:
        ShapeRoot targetObject;
    };

    class IsNotWorldShape : public UnaryShapeRootPred {
    public:
        IsNotWorldShape()
          : UnaryShapeRootPred()
        { }

        ~IsNotWorldShape() { }

        bool operator()(const ShapeRoot&) const;
    };

    class IsStar : public UnaryShapeRootPred {
    public:
        IsStar()
          : UnaryShapeRootPred()
        { }

        ~IsStar() { }

        bool operator()(const ShapeRoot&) const;
    };

    class IsShapeOfType : public UnaryShapeRootPred {
    public:
        IsShapeOfType(ShapeType_t shapeType)
          : UnaryShapeRootPred(),
            targetShapeType(shapeType)
        { }

        ~IsShapeOfType() { }

        bool operator()(const ShapeRoot&) const;

    private:
        ShapeType_t targetShapeType;
    };

    class IsNotExcludedShape : public UnaryShapeRootPred {
    public:
        IsNotExcludedShape(const ShapeRoot& kExecption)
          : UnaryShapeRootPred(),
            excludedShape(kExecption)
        { }

        ~IsNotExcludedShape() { }

        bool operator()(const ShapeRoot&) const;

    private:
        ShapeRoot excludedShape;
    };

    class ClearSelfShape : public UnaryShapeRootPred {
        public:
            ClearSelfShape() : UnaryShapeRootPred()
                {}
            ~ClearSelfShape() {}
            bool operator()(const ShapeRoot&) const;
    };
    
    // Assumes the "Agent" data is always valid
#define PERCEPTION_SEARCH(Dir)                                                  \
    class Is##Dir##Agent : public UnaryShapeRootPred {              \
    public:                                                                     \
        Is##Dir##Agent() : UnaryShapeRootPred() { }                 \
                                                                                \
        ~Is##Dir##Agent() { }                                                   \
                                                                                \
        bool operator()(const ShapeRoot&) const;                    \
    };

    // Creates IsLeftOfAgent and IsRightOfAgent
    PERCEPTION_SEARCH(LeftOf);
    PERCEPTION_SEARCH(RightOf);

    // Creates IsInFrontOfAgent and IsBehindAgent
    PERCEPTION_SEARCH(InFront);
    PERCEPTION_SEARCH(Behind);

    // Creates IsCloseByAgent and IsFarAwayFromAgent
    PERCEPTION_SEARCH(CloseBy);
    PERCEPTION_SEARCH(FarAwayFrom);

    //! Calculate the bearing from the agent's position and orientation to a specified point
    float bearingFromAgentToPoint(const Point&);

    //! Calcaulate the bearing from the agent's position and orientation to a specified shape
    float bearingFromAgentToObject(const ShapeRoot&);

    //! Calculates the distance between the agent's and a specified point's centroids
    float distanceFromAgentToPoint(const Point&);

    //! Calculates the distance between the agent's and a specified shape's centroids
    float distanceFromAgentToObject(const ShapeRoot&);

    //! Calculates the length of the open space in between the agent and a specified shape
    float distanceInBetweenAgentAndObject(const ShapeRoot&);

    //! Calculates the perceived radius of a target shape
    float objectRadius(const ShapeRoot&);

    

    //! Returns the closest shape/object to the agent
    ShapeRoot getClosestObject(const std::vector<ShapeRoot>&);

    //! Returns the closest object to a specified point (ASSUMES point and shapes are in SAME shape space)
    ShapeRoot getClosestObjectToPoint(const std::vector<ShapeRoot>&, const Point&);

    //! Returns the objects located in the specified region(s) relative to the agent orientation
		std::vector<ShapeRoot> getObjectsLocated(const std::vector<ShapeRoot>&, SearchLocation_t);

    //! Returns the objects that are a specified color
    std::vector<ShapeRoot> getMatchingObjects(const std::vector<ShapeRoot>& kObjects,
																							const std::string& kColor, const std::string& kType);

    //! Returns the closest object that matches the specified criteria (excluding the specified shape)
		ShapeRoot getClosestObjectMatching(const std::string& color, const std::string& type, SearchLocation_t location,
																			 const ShapeRoot& kExcludedShape = ShapeRoot());
}

#endif // PERCEPTION_SEARCH_H_
