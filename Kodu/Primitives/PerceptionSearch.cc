// INCLUDES
// tekkotsu
#include "DualCoding/AprilTagData.h"
using namespace DualCoding;

// Tekkodu Library
#include "Kodu/Primitives/PerceptionSearch.h"

namespace Kodu {

    bool HasAprilTagID::operator()(const ShapeRoot& kShape) const {
        return (kShape->getType() == aprilTagDataType
            && static_cast<const AprilTagData&>(kShape.getData()).getTagID() == id);
    }

    bool IsMatchForTargetObject::operator()(const ShapeRoot& kShape) const {
        // check if the objects match each other
        return (targetObject->isMatchFor(kShape));
    }

/*
    bool IsNotStar::operator()(const ShapeRoot& kWShape) const {
        return (!(kWShape->getType() && ShapeRootTypeConst(kWShape, AprilTagData)->getTagID() <= 4));
    }
*/
	bool IsNotWorldShape::operator()(const ShapeRoot& kWShape) const {
		switch (kWShape->getType()) {
			// if it's a cylinder, it must have a radius within some range
		case cylinderDataType: {
			const Shape<CylinderData>& kCyl = ShapeRootTypeConst(kWShape, CylinderData);
			if ((kCyl->getRadius() < 40.0f) || (kCyl->getRadius() > 75.0f))
				return true;
			break;
		}

			// if it's an april tag, it must be the north star
		case aprilTagDataType:
			return (!IsStar()(kWShape));

			// the following should not be included in the vector
		case localizationParticleDataType:
		case polygonDataType:
		case agentDataType:
			return false;

		default:
			return true;
		}
		return false;
	}

    bool IsStar::operator()(const ShapeRoot& kWShape) const {
        return (kWShape.isValid() && kWShape->getType() == aprilTagDataType
								&& static_cast<const AprilTagData&>(kWShape.getData()).getTagID() >= 24);  // was <= 4
    }

    bool IsShapeOfType::operator()(const ShapeRoot& kShape) const {
        return (kShape->getType() == targetShapeType);
    }

    bool IsNotExcludedShape::operator()(const ShapeRoot& kShape) const {
			return ( kShape.getId() != excludedShape.getId()  && (kShape->getName() != "AgentData") &&
							 ( kShape->getType() == cylinderDataType || kShape->getType() == agentDataType ) );
    }

    bool ClearSelfShape::operator()(const ShapeRoot& kShape) const {
                        return (kShape->getName() != "AgentData");
    }
    
    bool IsLeftOfAgent::operator()(const ShapeRoot& kShape) const {
        // get the bearing from the agent to the shape and return the result
        return (bearingFromAgentToObject(kShape) > 0.0f ? true : false);
    }

    bool IsRightOfAgent::operator()(const ShapeRoot& kShape) const {
        // is right of agent is simply the opposite of what is left of agent would return
        return (!(IsLeftOfAgent()(kShape)));
    }

    bool IsInFrontAgent::operator()(const ShapeRoot& kShape) const {
        // get the bearing from the agent to the shape
        AngSignPi dtheta = bearingFromAgentToObject(kShape);
        // since the last calculation would produce a value between -pi/2 and +pi/2,
        // add pi/2 to the last calculation. the result will give a position angle
        // if the object is in front the agent
        dtheta += AngSignPi(M_PI / 2.0f);
        // check the value of theta
        return (dtheta > 0.0f ? true : false);
    }

    bool IsBehindAgent::operator()(const ShapeRoot& kShape) const {
        // is behind agent is simply the opposite of what is in front agent would return
        return (!(IsInFrontAgent()(kShape)));
    }

    bool IsCloseByAgent::operator()(const ShapeRoot& kShape) const {
        // get the distance between the shape and the agent
        return (distanceFromAgentToObject(kShape) <= 700.0f);
    }

    bool IsFarAwayFromAgent::operator()(const ShapeRoot& kShape) const {
        // get the distance between the shape and the agent
        return (distanceFromAgentToObject(kShape) >= 1050.0f);
    }

    float bearingFromAgentToPoint(const Point& kPoint) {
        float dtheta = 0.0f;
        switch (kPoint.getRefFrameType()) {
            // take the agent's centroid into consideration if the point is allocentric
            case allocentric:
            {
                // calculate the bearing between some point "kPoint" and the agent's position
                // bearing2ThisPoint = arctan(dy/dx)
                const Point& kAgentPt = VRmixin::theAgent->getCentroid();
                AngSignPi bearing2ThisPoint = (kPoint - kAgentPt).atanYX();
                // subtract the agent's orientation (heading) from the bearing to get the point's angle
                // relative ot the agent
                dtheta = bearing2ThisPoint - AngSignPi(VRmixin::theAgent->getOrientation());
                break;
            }
            // simply calculate the arctan of the point...
            case egocentric:
            {
                dtheta = kPoint.atanYX();
                break;
            }
            // handles all other Reference Frame Types...
            default:
                std::cout << "WARNING: Used unhandled reference frame in bearingFromAgentToPoint(...)\n";
                break;
        }
        // return the angle between the agent and the target point
        return dtheta;
    }

    float bearingFromAgentToObject(const ShapeRoot& kShape) {
        return bearingFromAgentToPoint(kShape->getCentroid());
    }

    float distanceFromAgentToPoint(const Point& kPoint) {
        float dx = kPoint.coordX();
        float dy = kPoint.coordY();
        switch (kPoint.getRefFrameType()) {
            // since the point is allocentric, take the agent's centroid into consideration.
            case allocentric:
            {
                // get the agent's point
                Point agentPoint = VRmixin::theAgent->getCentroid();
                // calculate the differences in the shape's and agent's positions
                dx = dx - agentPoint.coordX();
                dy = dy - agentPoint.coordY();
                break;
            }
            // since the point is egocentric, there is nothing more to calculate (the agent's centroid
            // is { 0, 0 }).
            case egocentric:
                break;

            // this handles all other Reference Frame Types...
            default:
                std::cout << "WARNING: Used unhandled reference frame in distanceFromAgentToPoint(...)\n";
                return (0.0f);
        }
        // return the distance
        return sqrt((dx * dx) + (dy * dy));
    }

    float distanceFromAgentToObject(const ShapeRoot& kShape) {
        return distanceFromAgentToPoint(kShape->getCentroid());
    }

    float distanceInBetweenAgentAndObject(const ShapeRoot& kShape) {
        // approx. measurements based on the iRobot CREATE (measured on/before Dec. 5, 2013)
        static float const kRobotInflatedRadius = 205.0f;
        float distBtwObjects = 0.0f;
        switch(kShape->getType()) {
            case cylinderDataType:
            {
                // get the radius of the cylinder
                float radius = objectRadius(kShape);
                // calculate the distance between the objects
                float distBetweenCentroids = distanceFromAgentToObject(kShape);
                distBtwObjects = distBetweenCentroids - kRobotInflatedRadius - radius;
                break;
            }
            case agentDataType:
            {
                // calculate the distance between the objects
                float distBetweenCentroids = distanceFromAgentToObject(kShape);
                distBtwObjects = distBetweenCentroids - 2*kRobotInflatedRadius;
                break;
            }
            default:
							std::cout << "WARNING: Used unhandled shape type " << kShape << " in "
                          << "distanceInBetweenAgentAndObject(...)\n";
                return (0.0f);
        }
        return (distBtwObjects > 0.0f ? distBtwObjects : 0.0f);
    }

    float objectRadius(const ShapeRoot& kShape) {
        static float const kErrValue = -1.0f;
        float radius = 0.0f;
        switch(kShape->getType()) {
            case cylinderDataType:
                radius = ShapeRootTypeConst(kShape, CylinderData)->getRadius();
                break;

            case agentDataType:
							  radius = 205.0f;  // for iRobot Create
                break;

            default:
                std::cout << "WARNING: Used unhandled shape type in objectRadius(...)\n";
                return kErrValue;
        }
        return radius;
    }

    ShapeRoot getClosestObject(const std::vector<ShapeRoot>& kObjects) {
        const std::size_t kSize = kObjects.size();
        // if the vector's size is zero, return an invalid shape
        if (kSize == 0) {
            return ShapeRoot();
        }

        // else if the vector's size is one, return the first element (the only shape in the vector)
        else if (kSize == 1) {
            return kObjects[0];
        }

        // else iterate over the vector and find the closest shape
        else {
            ShapeRoot nearestObject = kObjects[0];
            float nearestObjectDist = distanceFromAgentToObject(nearestObject);
            
            // iterate over the remainder of the objects
            for (std::size_t index = 1; index < kSize; index++) {
                float currentObjectDist = distanceFromAgentToObject(kObjects[index]);
                if (currentObjectDist < nearestObjectDist) {
                    nearestObjectDist = currentObjectDist;
                    nearestObject = kObjects[index];
                }
            }
            return nearestObject;
        }
    }

    ShapeRoot getClosestObjectToPoint(const std::vector<ShapeRoot>& kShapes,
        const Point& kComparisonPoint)
    {
        const std::size_t kSize = kShapes.size();
        // if the vector is empty, return an invalid shape
        if (kSize == 0) {
            return ShapeRoot();
        }
        // else iterate over the vector and find the closest shape to the comparison point
        else {
            ShapeRoot closestMatch = kShapes[0];
            float minCenDiff = closestMatch->getCentroid().xyDistanceFrom(kComparisonPoint);

            for (std::size_t i = 1; i < kSize; i++) {
                float cenDiff = kShapes[i]->getCentroid().xyDistanceFrom(kComparisonPoint);
                if (cenDiff < minCenDiff) {
                    closestMatch = kShapes[i];
                    minCenDiff = cenDiff;
                }
            }
            return closestMatch;
        }
    }

    std::vector<ShapeRoot> getObjectsLocated(const std::vector<ShapeRoot>& kObjects, SearchLocation_t location) {
        // copy the objects to result
        std::vector<ShapeRoot> result = kObjects;
        std::cout << "Checking map for objects:";
        
        // test if the search location should be limited to the front or back
        if (location & SL_IN_FRONT) {
            std::cout << " [in front]";
            result = subset(result, IsInFrontAgent());
        } else if (location & SL_BEHIND) {
            std::cout << " [behind]";
            result = subset(result, IsBehindAgent());
        }
            
        // test if the search location should be limited to the left or right sides
        if (location & SL_TO_LEFT) {
            std::cout << " [to the left]";
            result = subset(result, IsLeftOfAgent());
        } else if (location & SL_TO_RIGHT) {
            std::cout << " [to the right]";
            result = subset(result, IsRightOfAgent());
        }

        // test if the search location is limited to "close by" or "far away (from)" the agent
        if (location & SL_CLOSE_BY) {
            std::cout << " [close by]";
            result = subset(result, IsCloseByAgent());
        }
        else if (location & SL_FAR_AWAY) {
            std::cout << " [far away]";
            result = subset(result, IsFarAwayFromAgent());
        }
        std::cout << std::endl;
        return result;
    }

	std::vector<ShapeRoot> getMatchingObjects(const std::vector<ShapeRoot>& kObjects,
																						const std::string& kColor, const std::string& kType) {
		std::string rcolor = ""; //TODO when setable colors are introduced this needs to be removed
		if (kType == "apple") {
			rcolor = "red";
		} else if (kType == "rock") {
			rcolor = "blue";
		} else if (kType == "tree") {
			rcolor = "green";
		}
		if (rcolor != "") //Condition for matching apples, trees, and rocks
			return subset(subset(kObjects, IsType(cylinderDataType)), IsColor(rcolor));
		else if(kType == "robot")
			return subset(kObjects, IsType(agentDataType)); //TODO setable colors not implemented
		else
			return subset(kObjects, IsName(kType)); // handle "turtle", "cycle", etc.
		
	}

    ShapeRoot getClosestObjectMatching(const std::string& color, const std::string& type, SearchLocation_t location,
																			 const ShapeRoot& kExcludedShape) {
        // The closest object that matches the specified criteria
        // (if not assigned a value it is invalid)
        ShapeRoot closestMatch;

        // Tekkotsu function. Returns all the objects (formerly only cylinders)
        NEW_SHAPEROOTVEC(matchingObjects, subset(VRmixin::worldShS, ClearSelfShape()));
        if ( ! matchingObjects.empty() && kExcludedShape.isValid() )
            matchingObjects = subset(matchingObjects, IsNotExcludedShape(kExcludedShape));
        
        // get objects with a particular color
        if ( ! matchingObjects.empty() )
					matchingObjects = getMatchingObjects(matchingObjects, color, type);

        // get objects that are located in a particular region (left, right, front, behind) and
        // distance (close by, far away) from the agent
        if ( ! matchingObjects.empty() && (location != SL_UNRESTRICTED) )
					matchingObjects = getObjectsLocated(matchingObjects, location);

        // get the closest object to the agent
        if ( ! matchingObjects.empty() )
					closestMatch = getClosestObject(matchingObjects);

        // return the object that matches the search criteria (may be valid or invalid)
        return closestMatch;
    }
}
