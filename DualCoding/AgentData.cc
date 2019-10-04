//-*-c++-*-
#include <vector>
#include <iostream>
#include <math.h>

#include "Point.h"       // Point data member
#include "Shared/Measures.h"    // coordinate_t; AngPi data member
#include "ShapeTypes.h"  // agentDataType

#include "SketchSpace.h"
#include "Sketch.h"
#include "ShapeSpace.h"  // required by DATASTUFF_CC
#include "ShapeRoot.h"   // required by DATASTUFF_CC

#include "visops.h"
#include "ShapeFuns.h"
#include "ShapeEllipse.h"
#include "Crew/MapBuilder.h"
#include "VRmixin.h"
#include "Motion/Kinematics.h"  // for kine

#include "AgentData.h"
#include "ShapeAgent.h"

using namespace std;

namespace DualCoding {

	DATASTUFF_CC(AgentData);

	AgentData::AgentData(ShapeSpace& _space, const Point &c, AngTwoPi orient) 
		: BaseData(_space,agentDataType), center_pt(c), orientation(orient), hostAddr(0)
	{ mobile = true; }
  
	AgentData::AgentData(const AgentData& otherData)
		: BaseData(otherData), 
			center_pt(otherData.center_pt), 
			orientation(otherData.orientation),
			hostAddr(otherData.hostAddr)
	{ mobile = true; }

	BoundingBox2D AgentData::getBoundingBox() const {
		BoundingBox2D b;
		fmat::Column<3> bl = -RobotInfo::AgentBoundingBoxHalfDims - RobotInfo::AgentBoundingBoxBaseFrameOffset;
		fmat::Column<3> tr =  RobotInfo::AgentBoundingBoxHalfDims - RobotInfo::AgentBoundingBoxBaseFrameOffset;
		fmat::Matrix<3,3> rot = fmat::rotationZN<3,float>(orientation);
		bl = rot*bl + RobotInfo::AgentBoundingBoxBaseFrameOffset + center_pt.getCoords();
		tr = rot*tr + RobotInfo::AgentBoundingBoxBaseFrameOffset + center_pt.getCoords();
		b.expand(bl);
		b.expand(tr);
		return b;
	}

	bool AgentData::isMatchFor(const ShapeRoot&other) const {
        //agent's are considered matching of they overlap in space
        if(!isSameTypeAs(other)) {
            return false;
        }
		const int MAX_CENTER_DIST = 100;
        float center_dist = (center_pt - other->getCentroid()).xyzNorm();
        return center_dist <= MAX_CENTER_DIST;
	}

	//! Print information about this shape. (Virtual in BaseData.)
	void AgentData::printParams() const {
		cout << "Type = " << getTypeName();
		cout << "Shape ID = " << getId() << endl;
		cout << "Parent ID = " << getParentId() << endl;

		// Print critical points.
		cout << endl;
		cout << "center{" << getCentroid().coords[0] << ", " << getCentroid().coords[1] << "}" << endl;
		
		cout << "orientation = " << orientation << endl;
		//cout << "color = " << getColor() << endl;
		printf("color = %d %d %d\n",getColor().red,getColor().green,getColor().blue);

		cout << "mobile = " << getMobile() << endl;
		cout << "viewable = " << isViewable() << endl;
	}


	//! Transformations. (Virtual in BaseData.)
	void AgentData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
		//  cout << "AgentData::applyTransform: " << getId() << endl;
		//  cout << Tmat << endl;
		center_pt.applyTransform(Tmat,newref);
		orientation = orientation - (AngTwoPi)atan2(Tmat(1,2),Tmat(1,1));
		//  updateOrientation();
	}

	//! Functions to set properties.
	void AgentData::setOrientation(AngTwoPi _orientation) {
		orientation = _orientation;
		deleteRendering();
	}

	void AgentData::projectToGround(const fmat::Transform& camToBase,
																	const PlaneEquation& groundplane) {
		center_pt.projectToGround(camToBase,groundplane);
	}

	bool AgentData::updateParams(const ShapeRoot& other, bool force) {
		if (isMatchFor(other) || force) {
			const AgentData& other_agent = ShapeRootTypeConst(other,AgentData).getData();
			int other_conf = other_agent.getConfidence();
			if (other_conf <= 0)
				return true;
			center_pt = (center_pt*confidence + other_agent.getCentroid()*other_conf) / (confidence+other_conf);
			orientation = orientation*((orientation_t)confidence/(confidence+other_conf))
				+ other_agent.getOrientation()*((orientation_t)other_conf/(confidence+other_conf));
			return true;
		}
		return false;
	}

	//! Render into a sketch space and return reference. (Private.)
	Sketch<bool>* AgentData::render() const {
		Sketch<bool>* draw_result =
			new Sketch<bool>(space->getDualSpace(), "render("+getName()+")");
		draw_result = 0;

		// JJW This does not take orientation into account.
		// should probably take principal axis into account later on
		int cx = int(getCentroid().coords[0]);
		int cy = int(getCentroid().coords[1]);
	
		// Sure the agent rendering is terribly inefficient, but it works
		const float a = 2.f;
		const float b = 2.f;
		const float x_skip = std::atan(1/(0.5f*a)); // minimum x-diff w/o gaps 
		for(float x = (cx-a); x<(cx+a); x+=x_skip) {
			float y_y0_sq = (b*b) * (1 - (x-cx)*(x-cx)/(a*a));
			if(y_y0_sq > 0) {
				int y_bot = cy + (int)(sqrt(y_y0_sq));
				int y_top = cy - (int)(sqrt(y_y0_sq));
				(*draw_result)((int)x,y_bot) = true;
				(*draw_result)((int)x,y_top) = true;
			}
		}
		(*draw_result)(cx-(int)a, cy) = true; // fill in "holes" at ends
		(*draw_result)(cx+(int)a, cy) = true;
		return draw_result;
	}

	// Extraction

	int AgentData::getColorIdMatch(yuv pixelColor) {
        // Finds the closest identification color to pixelColor.
        // id colors use line fits with u and v assumed linear wrt y
		int distances[NUM_COLOR_IDS];
		for (int i = 0; i < NUM_COLOR_IDS; ++i) {
            float u_dist = (pixelColor.u - color_ids[i].u_0 - color_ids[i].u_slope*pixelColor.y);
            float v_dist = (pixelColor.v - color_ids[i].v_0 - color_ids[i].v_slope*pixelColor.y);
			distances[i] =  u_dist*u_dist + v_dist*v_dist;
        }

		int min_val = distances[0];
		int min_index = 0;
		for (int i = 1; i < NUM_COLOR_IDS; ++i) {
			if (distances[i] < min_val) {
				if (color_ids[i].y_low >= 0 &&
						(pixelColor.y < color_ids[i].y_low || pixelColor.y > color_ids[i].y_high)) {
					continue;
				}
				min_val = distances[i];
				min_index = i;
			}
		}

		return color_ids[min_index].id;
	}

	int AgentData::getIdFromCenter(const Sketch<yuv> &camFrameYUV, const Point &a, const Point &b, const Point &c) {
        //find the id from the color at the center of the three points
		Point center = (a + b + c)/3;
		yuv pixelColor = camFrameYUV(center.coordX(), center.coordY());
		return getColorIdMatch(pixelColor);
	}

	int AgentData::getIdFromCorner(const Sketch<yuv> &camFrameYUV, const Point &vert_a, const Point &vert_b, const Point &loner) {
        //find the id from the color at the fourth corner of a rectangle
        //where vert_a, vert_b, and vert_c specify three corners of the rectangle
		float vert_dist = vert_b.coordY() - vert_a.coordY();
		yuv pixelColor;
        pixelColor = camFrameYUV(loner.coordX(), loner.coordY() - vert_dist);
		// cout << "pixel color: " << pixelColor << endl;
		return getColorIdMatch(pixelColor);
	}

	bool AgentData::circlesVerticallyAligned(const Shape<EllipseData> &a, const Shape<EllipseData> &b) {
        //tests whether the centers of two ellipses are vertically aligned with a small margin
		const float VLEVEL_THRESHOLD = 0.25f;
		Point a_center = a->getCentroid();
		Point b_center = b->getCentroid();
		float atanValue;
		if (a_center.coordX() >= b_center.coordX())
			atanValue = fabs((a_center - b_center).atanYX());
		else
			atanValue = fabs((b_center - a_center).atanYX());
		return atanValue < VLEVEL_THRESHOLD;
	}

	bool AgentData::circlesHorizontallyAligned(const Shape<EllipseData> &a, const Shape<EllipseData> &b) {
        //tests whether the centers of two ellipses are horizontally aligned with a small margin
		const float HLEVEL_THRESHOLD = 0.45f;
		Point a_center = a->getCentroid();
		Point b_center = b->getCentroid();
		float atanValue;
		if (a_center.coordY() >= b_center.coordY())
			atanValue = fabs((float)(a_center - b_center).atanYX() - M_PI_2);
		else
			atanValue = fabs((float)(b_center - a_center).atanYX() - M_PI_2);
		return atanValue < HLEVEL_THRESHOLD;
	}

	void AgentData::findCirclesManual(const Sketch<uchar> &camFrame, 
																		std::vector<int> &neighbor_density,
																		std::vector<Shape<EllipseData> > &agentIdCircles) {
		// Finds red ellipses in a camera frame.
		// It can find smaller ellipses than the normal MapBuilder method.
		NEW_SKETCH_N(red_stuff, bool, visops::colormask(camFrame, "red"));
		NEW_SKETCH_N(blobs, usint, visops::labelcc(red_stuff));
		NEW_SKETCH_N(labeli, bool, visops::zeros(camFrame));
		NEW_SHAPEVEC(circles, EllipseData, 0);

		// Given connected components in the camera frame, find a bounding
		// box for each component.  Filter out components by aspect ratio
		// (width/height) and actual area to bounding box area ratio.
		for (unsigned int i = 1; i <= blobs->max(); ++i) {
			labeli.bind(blobs == i);
			int count = 0;
			int x_sum = 0;
			int y_sum = 0;
			int min_index = labeli->findMinPlus();
			int xstart = min_index % labeli.width;
			int ystart = min_index / labeli.width;
			int lbound = labeli.width;
			int rbound = xstart;
			int tbound = ystart;
			int bbound = ystart;
			for (int y = ystart; y < labeli.height; ++y) {
				for (int x = 0; x < labeli.width; ++x) {
					if (labeli(x, y)) {
						if (x < lbound)
							lbound = x;
						else if (x > rbound)
							rbound = x;
						if (y > bbound) {
							bbound = y;
						}
						x_sum += x;
						y_sum += y;
						++count;
					}
				}
			}
			if (count == 0) continue;

			float major,  minor;
			if (bbound - tbound > rbound - lbound) {
				major = bbound - tbound;
				minor = rbound - lbound;
			} else {
				major = rbound - lbound;
				minor = bbound - tbound;
			}
			++major;
			++minor;

			//aspect ratio filter
			if (major/minor > 3) {
				// cout << i << ": aspect ratio " << major/minor << endl;
				continue;
			}

			//area filter
			if ((major*minor)/count > 2) {
				// cout << i << ": area " << (major*minor)/count << endl;
				continue;
			}

			minor = major;
			float center_x = x_sum/((float)count);
			float center_y = y_sum/((float)count);
			NEW_SHAPE_N(circle, EllipseData,
									new EllipseData(VRmixin::camShS,
																	Point(center_x, center_y, 0),
																	major/2, minor/2));
			circle->setColor("red");
			circles.push_back(circle);
			agentIdCircles.push_back(circle);
		}

		// Find number of neighbors to each ellipse in a neighborhood
		// of (-deltaX to deltaX, -deltaY to deltaY).  This is used to
		// rule out circles as id markers later to prevent noise from
		// looking like a small rectangular pattern.
		const int deltaX = 60;
		const int deltaY = 60;

		SHAPEVEC_ITERATE(circles, EllipseData, circle) {
			neighbor_density.push_back(0);
			Point centroid = circle->getCentroid();
			float center_x = centroid.coordX();
			float center_y = centroid.coordY();
			float left = max(center_x - deltaX, 0.0f);
			float right = min(center_x + deltaX, (float)blobs.width - 1);
			float top = max(center_y - deltaY, 0.0f);
			float bottom = min(center_y + deltaY, (float)blobs.height - 1);

			//for each circle, check all other circles to see if they are neighbors
			SHAPEVEC_ITERATE(circles, EllipseData, neighbor) {
				Point neighbor_centroid = neighbor->getCentroid();
				float neighbor_x = neighbor_centroid.coordX();
				float neighbor_y = neighbor_centroid.coordY();
				if (neighbor_x >= left && neighbor_x <= right && neighbor_y >= top && neighbor_y <= bottom) {
					++neighbor_density.back();
				}
			} END_ITERATE;
		} END_ITERATE;
	}

	void AgentData::findAgentsBelt(const Sketch<yuv> &camFrameYUV,
																 const std::vector<int> &neighbor_density,
																 const std::vector<Shape<EllipseData> > &circles) {
        // Look for an agent identification marker around the base of
        // a robot.  The marker is four circles marking the corners of
        // a rectangle.  Three of the markers are red and the color of
        // the fourth marker is used to identify the specific robot.
        // The side of the robot the marker is on is given by which of
        // the four circles is not red.
		const float robot_radius = VRmixin::theAgent->getBoundingBoxHalfDims()[1];
		// cout << "num circles: " << circles.size() << endl;

		enum RobotSide robot_side;
		vector<Shape<AgentData> > possible_agents;

		// We project the circles to a plane slightly above the ground to
		// reduce the distance error, since the circles aren't actually in
		// a horizontal plane.
		PlaneEquation above_ground(0, 0, 1, 65); //when on ground
		//PlaneEquation above_ground(0, 0, 1, 65 - 737); //when on table
		const fmat::Transform cam_transform = kine->linkToBase(CameraFrameOffset);

		for (unsigned int i = 0; i < neighbor_density.size(); ++i) {
			// cout << "density: " << circles[i]->getId() << " - " << neighbor_density[i] << endl;
		}

		// For each unique combination of three circles, check whether
		// they can form a rectangle.  If so, compute the center of the
		// robot from the position of the two vertically aligned circles
		// and find its id from the fourth circle.
		const int DENSITY_THRESHOLD = 6;
		for (unsigned int i = 0; i < circles.size(); ++i) {
			if (neighbor_density[i] > DENSITY_THRESHOLD)
				continue;
			for (unsigned int j = i + 1; j < circles.size(); ++j) {
				if (!circlesVerticallyAligned(circles[i], circles[j])) {
					// cout << circles[i]->getId() << " and " << circles[j]->getId() << " not vertically aligned." << endl;
					continue;
				}
				if (!compareCircleSize(circles[i], circles[j])) {
					//cout << circles[i]->getId() << " and " << circles[j]->getId() <<
					//" not similar size." << endl;
					continue;
				}
				for (unsigned int k = 0; k < circles.size(); ++k) {
					if (k == i || k == j) continue;
					unsigned int v, h;
					if (circlesHorizontallyAligned(circles[i], circles[k])) {
						v = i;
						h = j;
					} else if (circlesHorizontallyAligned(circles[j], circles[k])) {
						v = j;
						h = i;
					} else {
						// cout << "neither " << circles[i]->getId() << " nor " << circles[j]->getId() <<
						//	" is horizontally aligned with " << circles[k]->getId() << "." << endl;
						continue;
					}
					if (!compareCircleSize(circles[i], circles[k]) &&
							!compareCircleSize(circles[j], circles[k]))
						continue;
					float radius = max(circles[i]->getSemimajor(),
														 max(circles[j]->getSemimajor(),
																 circles[k]->getSemimajor()));

					if (fabs(circles[i]->getCentroid().coordX() - circles[j]->getCentroid().coordX()) >= 7*radius) {
						//cout << circles[i]->getId() << " too far in x from " << circles[j]->getId() << endl;
						continue;
					}
					if (fabs(circles[v]->getCentroid().coordY() - circles[k]->getCentroid().coordY()) >= 7*radius) {
						//cout << circles[v]->getId() << " too far in y from " << circles[k]->getId() << endl;
						continue;
					}

					Point pi = circles[i]->getCentroid();
					Point pj = circles[j]->getCentroid();
					Point pk = circles[k]->getCentroid();

					if (!pi.projectToGround(cam_transform, above_ground)) {
						cout << "project to ground failed." << endl;
						break;
					}
					if (!pj.projectToGround(cam_transform, above_ground)) {
						cout << "project to ground failed." << endl;
						break;
					}
					if (!pk.projectToGround(cam_transform, above_ground)) {
						cout << "project to ground failed." << endl;
						continue;
					}
					Point pv = circles[v]->getCentroid();
					Point ph = circles[h]->getCentroid();

					if (!compareCircleDist(pi, pj) || !compareCircleDist(pj, pk)) {
						// cout << circles[i]->getCentroid() << endl;;
						// cout << pi << endl;
						// cout << circles[j]->getCentroid() << endl;;
						// cout << pj << endl;
						continue;
					}

                    // cout << "red dot colors: " <<
                    //     camFrameYUV(circles[i]->getCentroid().coordX(), circles[i]->getCentroid().coordY()) << " " <<
                    //     camFrameYUV(circles[j]->getCentroid().coordX(), circles[j]->getCentroid().coordY()) << " " <<
                    //     camFrameYUV(circles[k]->getCentroid().coordX(), circles[k]->getCentroid().coordY()) << endl;

					// cout << "v: " << circles[v]->getId() << endl;
					// cout << "h: " << circles[h]->getId() << endl;
					// cout << "k: " << circles[k]->getId() << endl;

					int robot_id = getIdFromCorner(camFrameYUV,
																				 circles[k]->getCentroid(),
																				 circles[v]->getCentroid(),
																				 circles[h]->getCentroid());
					// cout << "robot id: " << robot_id << endl;

					if (circles[k]->getCentroid().coordY() < pv.coordY()) {
						if (pv.coordX() < ph.coordX()) // top right missing
							robot_side = AgentData::FRONT;
						else // top left missing
							robot_side = AgentData::LEFT;
					} else {
						if (pv.coordX() < ph.coordX()) // bottom right missing
							robot_side = AgentData::BACK;
						else // bottom left missing
							robot_side = AgentData::RIGHT;
					}

                    //pi and pj are the vertically aligned circles
                    //knowing the radius of the robot we can construct a perpendicular
                    //bisector of the line between pi and pj which has length equal to
                    //the robot's radius
                    //this should give a good estimate of the robot's position
					Point diff = (pi - pj);
					Point mid = pi + diff/2;
					fmat::Matrix<3,3,float> rotate = fmat::rotationZ(-copysign(1.0f,diff.coordY())*M_PI/2);
					fmat::Column<3,float> robot_center = robot_radius*rotate*diff.unitVector().getCoords();
					Point center(0, 0, 0, egocentric);
					center.setCoords(robot_center);
					AngTwoPi orient_to_dots((float)center.atanYX());
					center += mid;

					// The pattern of the three red circles gives us the side of
					// the robot we're looking at, so we can adjust the
					// orientation.
					AngTwoPi orient;
					cout << "Saw agent '" << agent_names[robot_id] << "' ";
					switch (robot_side) {
					case AgentData::FRONT:
						cout << "front";
						orient = (float)orient_to_dots + M_PI;
						break;
					case AgentData::BACK:
						cout << "back";
						orient = (float)orient_to_dots;
						break;
					case AgentData::LEFT:
						cout << "left";
						orient = (float)orient_to_dots + M_PI/2;
						break;
					case AgentData::RIGHT:
						cout << "right";
						orient = (float)orient_to_dots + 3*M_PI/2;
						break;
					}
					cout << ", orientation " << float(orient)*180/M_PI << " deg." << endl;

					// Set the name based on the id color and add it to
					// localShS.  These will be filtered by matchSrcToDst in the
					// MapBuilder which relies on isMatchFor() and
					// updateParams() of AgentData.
					Shape<AgentData> new_agent(VRmixin::localShS, center);
					new_agent->setOrientation(orient);
					new_agent->setName(agent_names[robot_id]);
				}
			}
		}
	}

	void AgentData::extractAgents(const Sketch<uchar> &camFrame, const Sketch<yuv> &camFrameYUV,
																const std::set<color_index> &objcolors) {
		std::vector<int> neighbor_density;
		std::vector<Shape<EllipseData> > agentIdCircles;
		findCirclesManual(camFrame, neighbor_density, agentIdCircles);
		findAgentsBelt(camFrameYUV, neighbor_density, agentIdCircles);
		VRmixin::camShS.deleteShapes(agentIdCircles);
	}

	const char* AgentData::agent_names[NUM_COLOR_IDS] = {
		"octopus", // blue
		"turtle", // green
		"cycle", // yellow
		"kodu", // black
	};

	AgentData::ColorIdTarget AgentData::color_ids[NUM_COLOR_IDS] = {
		{ 0, -1, -1, 115, 1.086f, 135, -0.638f}, // blue
		{ 1, -1, -1, 110, 0, 130, -0.186}, // green
		{ 2, -1, -1, 90, 0, 152, -0.121f},  // yellow
		{ 3, 0, 16, 128, 0, 128, 0},  // black
	};

} // namespace
