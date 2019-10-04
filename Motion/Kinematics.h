//-*-c++-*-
#ifndef INCLUDED_Kinematics_h_
#define INCLUDED_Kinematics_h_

#include "Shared/RobotInfo.h"
#include "Shared/fmat.h"
#include "Shared/Measures.h"
#include "Motion/KinematicJoint.h"

#include <string>
#include <vector>

class VisionObjectEvent;

//! Forward and inverse kinematics calculations using Tekkotsu output indices.
/*! 
 *  You should read the <a
 *  href="http://www.tekkotsu.org/Kinematics.html">Kinematics
 *  tutorial</a> to get a general understanding of the math involved
 *  and diagrams for usage with supported robots.
 *
 *  This class involves all aspects of the forward kinematics:
 *  calculations concerning locations and orientations in space given
 *  a known set of joint configurations.  There is a global
 *  instantiation of Kinematics named ::kine, which can be used to
 *  perform these calculations regarding the joint positions currently
 *  in ::state.
 *
 *  To perform kinematics on a hypothetical set of joint values,
 *  use PostureEngine or one of its subclasses.  PostureEngine also
 *  adds inverse kinematic functions, which will allow you to
 *  determine joint angles in order to reach a given point.
 *  
 *  Wherever a reference frame index is requested, you can
 *  simply supply one of the output indexes in the usual manner:
 *  @code kine->linkToBase(HeadOffset+TiltOffset); @endcode
 *
 *  However, there are also a number of points on the body which are
 *  not joints, but should have their own reference frames, such as
 *  the base frame, or the camera.  These frames have their own
 *  indices, listed in the robot info file for the model in question
 *  (such as ERS7Info.h), with names ending in @c FrameOffset.
 *  @code kine->linkToBase(CameraFrameOffset); @endcode
 *
 *  Example code:
 *  @code
 *  // Find the ray from the camera to whatever the near-field IR is hitting:
 *  fmat::Transform T = kine->linkToLink(NearIRFrameOffset,CameraFrameOffset);
 *  fmat::Column<3> camera_ray = T*fmat::pack(0,0,state->sensors[NearIRDistOffset]);
 *  float x; // x will be in the range ±1 for resolution layer independence
 *  float y; // y ranges ±y_dim/x_dim (i.e. ±1/aspectRatio)
 *  config->vision.computePixel(camera_ray[0],camera_ray[1],camera_ray[2],x,y);
 *  @endcode
 *  
 *  Finally, for each model we have created a database of "interest points" --
 *  locations of notable interest on the body of the robot.  These may be of
 *  use to people attempting to use the limbs to manipulate objects.
 *  To access these interest points, call #getInterestPoint
 *  with the name of the interest point, obtained
 *  from the <a href="http://www.tekkotsu.org/Kinematics.html">diagrams</a>.
 *
 *  Note that you can pass a comma separated list of interest point names
 *  and the result will be the midpoint of those interest points:
 *  @code kine->getInterestPoint(BaseFrameOffset,"LowerInnerFrontLFrShin,LowerOuterFrontLFrShin"); @endcode
 *  
 *  @see PostureEngine for inverse kinematics
 */
class Kinematics {
public:
	//!Constructor, pass the full path to the kinematics configuration file
	Kinematics() : root(), lastUpdateTime(0)
	{
		init();
	}
	//!Copy constructor, everything is either update-before-use or static, copy is normal init
	Kinematics(const Kinematics& k) : root(k.root), lastUpdateTime(0)
	{
		for(unsigned int i=0; i<NumReferenceFrames; ++i)
			jointMaps[i]=NULL;
		root.buildChildMap(jointMaps,0,NumReferenceFrames);
	}
	//!Assignment operator, everything is either update-before-use or static, assignment is no-op
	Kinematics& operator=(const Kinematics& k) {
		root=k.root;
		for(unsigned int i=0; i<NumReferenceFrames; ++i)
			jointMaps[i]=NULL;
		root.buildChildMap(jointMaps,0,NumReferenceFrames);
		return *this;
	}

	//!Destructor
	virtual ~Kinematics();

    //! Returns a pointer to the root of the kinematic tree
    const KinematicJoint* getRoot() { return &root; }

	//! returns the KinematicJoint structure for the specified Tekkotsu output or reference frame offset
	const KinematicJoint* getKinematicJoint(unsigned int idx) const { update(); return jointMaps[idx]; }
	
	fmat::Column<3> getPosition(unsigned int idx) const {
		if(jointMaps[idx]==NULL)
			throw std::runtime_error("Kinematics::getPosition: kinematics unspecified for requested offset");
		update();
		return jointMaps[idx]->getFullT().translation();
	}
		
	//! Returns a matrix for transforming from link frame @a j to base frame
	/*! @param[in]  link the output offset, see class notes for values */
	fmat::Transform linkToBase(unsigned int link) {
		if(jointMaps[link]==NULL)
			throw std::runtime_error("Kinematics::linkToBase: kinematics unspecified for requested offset");
		update();
		return jointMaps[link]->getFullT();
	}

	//! Returns a matrix for transforming from the base frame to link @a j frame
	/*! @param[in]  link the output offset, see class notes for values */
	fmat::Transform baseToLink(unsigned int link) { return linkToBase(link).inverse(); }

	//! Returns a matrix for transforming from link @a iL to link @a oL
	/*! @param[in]  iL the output offset to convert from, see class notes for values
	 *  @param[in]  oL the output offset to convert to, see class notes for values */
	fmat::Transform linkToLink(unsigned int iL, unsigned int oL) {
		if(jointMaps[iL]==NULL || jointMaps[oL]==NULL)
			throw std::runtime_error("Kinematics::linkToLink: kinematics unspecified for requested offset");
		update();
		return jointMaps[iL]->getT(*jointMaps[oL]);
	}
	
	//! returns a transformation to account for standing pose, where the origin of the "local" space is the projection of the base frame origin along the ground plane normal
	fmat::Transform baseToLocal() { return baseToLocal(calculateGroundPlane()); }
	
	//! returns a transformation to account for standing pose, where the origin of the "local" space is the projection of the base frame origin along the ground plane normal
	fmat::Transform baseToLocal(const PlaneEquation& plane);
	
	//! returns a transformation to account for standing pose, where the origin of the "local" space is the projection of the base frame origin along the ground plane normal
	fmat::Transform localToBase() { return baseToLocal(calculateGroundPlane()).inverse(); }

	//! returns a transformation to account for standing pose, where the origin of the "local" space is the projection of the base frame origin along the ground plane normal
	fmat::Transform localToBase(const PlaneEquation& plane) { return localToBase(plane).inverse(); }
	
	
	//! Returns the location of a named point and the link it is attached to
	/*! @param[in]  name   the name of the interest point; varies by model, <a href="http://www.tekkotsu.org/Kinematics.html">see the diagrams</a> for your model.
	 *  @param[out] link   on exit, offset of the link, or -1U if not found
	 *  @param[out] ip     on exit, the requested point, relative to the link frame returned in @a link
	 *  @param[in] convertToJoint if true and @a name is relative to a "pure" reference frame (i.e. NumOutputs <= link < NumReferenceFrames), then @a link and @a ip will be transformed to the parent joint (if it exists)
	 *
	 *  If @a name is not found, link will be -1 and ip will be all 0's. */
	void getInterestPoint(const std::string& name, unsigned int& link, fmat::Column<3>& ip, bool convertToJoint=false);

	//! Returns the location of a named point, relative to any desired reference frame
	/*! @param[in]  link   the desired link reference frame to give results in
	 *  @param[in]  name   the name of the interest point; varies by model, <a href="http://www.tekkotsu.org/Kinematics.html">see the diagrams</a> for your model.
	 *
	 *  You can pass a comma separated list of interest point names and the result will be the midpoint of those IPs.
	 *
	 *  If an interest point is not found, a std::runtime_error is thrown. */
	fmat::Column<3> getInterestPoint(unsigned int link, const std::string& name);



#ifdef TGT_HAS_LEGS
	//! Calculate the leg heights along a given "down" vector (0 is level with base frame)
	/*! This can be based on either the gravity vector from accelerometer readings,
	 *  or if that may be unreliable due to being in motion, you could do some basic
	 *  balance modeling and pass a predicted vector.  This uses the interest point database
	 *  to find the lowest interest point for each leg
	 *  @note on Aibo platforms, if packing accelerometer readings, don't forget to negate the "left" accelerometer! */
	void calcLegHeights(const fmat::Column<3>& down, float heights[NumLegs]);
	
	//! Find the leg which is in least contact with ground
	/*! @see calcLegHeights()
	 *  @note on Aibo platforms, if packing accelerometer readings, don't forget to negate the "left" accelerometer!
	 *  @return index of leg which is highest in reference to gravity vector */
	LegOrder_t findUnusedLeg(const fmat::Column<3>& down);
#endif

	//! Find the ground plane by fitting a plane to the lowest 3 interest points
	/*! This function merely calls the other version of calculateGroundPlane with the current
	 *  gravity vector as the "down" vector.
	 *  @return vector of the form @f$p_1x + p_2y + p_3z = p_4@f$, relative to the base frame */
	PlaneEquation calculateGroundPlane();

	//! Find the ground plane by fitting a plane to the lowest 3 interest points
	/*! @note on Aibo platforms, if packing accelerometer readings, don't forget to negate the "left" accelerometer!
	 *  @return vector of the form @f$p_1x + p_2y + p_3z = p_4@f$, relative to the base frame */
	PlaneEquation calculateGroundPlane(const fmat::Column<3>& down);

	//! Find the point of intersection between a ray and a plane
	/*! @param j is the link number the ray is relative to
	 *  @param r_j is the ray through the origin of link @a j
	 *  @param b is the link number the plane is relative to (probably BaseFrameOffset)
	 *  @param p_b represents the plane to be intersected
	 *  @param f is the link number the results should be relative to
	 *  @param objCentroidHeight the height of the object's centroid above the ground
	 *
	 *  @a p_b should be of the form @f$p_1x + p_2y + p_3z = p_4@f$
	 *  @return homogeneous coordinate of intersection (may be infinity)
	 *
	 *  For projecting to the ground plane, one of the specialized projectToGround()
	 *  functions may be more convenient. */
	fmat::Column<4> projectToPlane(unsigned int j, const fmat::Column<3>& r_j,
				       unsigned int b, const PlaneEquation& p_b,
				       unsigned int f, float objCentroidHeight=0);
	
	//! Find the location of an object on the ground from an arbitrary ray @a r_j in reference frame @a j (probably CameraFrameOffset)
	/*! @param visObj the vision object to project
	 *  @param objCentroidHeight the height of the object's centroid above the ground
	 *  Uses the default calculateGroundPlane(), otherwise call projectToPlane() to specify a custom plane. */
	fmat::Column<4> projectToGround(unsigned int j, const fmat::Column<3>& r_j, float objCentroidHeight=0) {
		return projectToPlane(j,r_j, BaseFrameOffset, calculateGroundPlane(), BaseFrameOffset,objCentroidHeight);
	}
	
	//! Find the location of an object on the ground (the easy way from a vision object event (i.e. EventBase::visObjEGID))
	/*! @param visObj the vision object to project
	 *  @param objCentroidHeight the height of the object's centroid above the ground
	 *  Uses the default calculateGroundPlane(), otherwise call the other projectToGround() to specify a custom plane. */
	fmat::Column<4> projectToGround(const VisionObjectEvent& visObj, float objCentroidHeight=0) {
		return projectToGround(visObj, calculateGroundPlane(), objCentroidHeight);
	}

	//! Find the location of an object on the ground with a custom ground plane specification
	/*! @a gndPlane must be specified relative to the base frame, in
	 *  the form @f$p_1x + p_2y + p_3z = p_4@f$,
	 *  @see projectToPlane() for more control over projection and results */
	fmat::Column<4> projectToGround(const VisionObjectEvent& visObj, const PlaneEquation& gndPlane, float objCentroidHeight=0);
	
	//! A simple utility function, converts x,y,z,h to a fmat::Column<3>
	/*! @param[in]  x the value for the first element
	 *  @param[in]  y the value for the second element
	 *  @param[in]  z the value for the third element
	 *  @return @f$ \left[\begin{array}{c} x\\y\\z\\ \end{array}\right] @f$ */
	static fmat::Column<3> pack(float x, float y, float z) { return fmat::pack(x,y,z); }
	
	//! A simple utility function, converts x,y,z,h to a fmat::Column<4>
	/*! @param[in]  x the value for the first element
	 *  @param[in]  y the value for the second element
	 *  @param[in]  z the value for the third element
	 *  @param[in]  h the value for the fourth element
	 *  @return @f$ \left[\begin{array}{c} x\\y\\z\\h\\ \end{array}\right] @f$ */
	static fmat::Column<4> pack(float x, float y, float z, float h) { return fmat::pack(x,y,z,h); }
	
	//! A simple utility function, pulls the first 3 rows of the first column, divides each by the fourth row, and stores into ox, oy, and oz
	/*! @param[in]  m  the column to unpack from, applying scaling factor
	 *  @param[out] ox set to the first element of @a m, divided by fourth element
	 *  @param[out] oy set to the second element of @a m, divided by fourth element
	 *  @param[out] oz set to the third row element of @a m, divided by fourth element */
	static void unpack(const fmat::SubVector<4, const float>& m, float& ox, float& oy, float& oz) {
		ox=m[0]/m[3]; oy=m[1]/m[3]; oz=m[2]/m[3];
	}
	
protected:
	//! Called by constructors to do basic setup - first call will read Config::motion_config::kinematics from disk, future initializes reuse static roconfig
	void init();
	
	//! initializes static variables -- only call if not #staticsInited
	static void initStatics();
	
	//! checks that statics have been initialized, and calls initStatics if they are missing
	static void checkStatics() { if(!staticsInited) initStatics(); }
	
public:
	//! refresh the joint settings in #root from WorldState::outputs
	virtual void update() const;
	
	//! holds the position and attached link of a given interest point
	struct InterestPoint {
		InterestPoint() : p(), output(-1U) {} //!< constructor
		InterestPoint(float x, float y, float z, unsigned int output_)
			: p(fmat::pack(x,y,z)), output(output_) {} //!< constructor
		fmat::Column<3> p; //!< position
		unsigned int output; //!< output offset for link that this is relative to
	};
	
protected:
	//! the root of the kinematic tree
	KinematicJoint root;
	
	//! determine if the joints are up to date (compare to WorldState::lastSensorUpdateTime)
	mutable unsigned int lastUpdateTime;
	
	//! holds mapping from tekkotsu output index to chain and link indicies
	KinematicJoint* jointMaps[NumReferenceFrames];
	
	//! initially false, set to true after first Kinematics is initialized
	static bool staticsInited;
	
	//! we'll be using the hash_map to store named interest points
	typedef std::map<std::string,InterestPoint> InterestPointMap;
	//! these interest points are shared by all Kinematics classes (i.e. all PostureEngines)
	/*! this is to reduce initialization time, but does mean one robot can't do
	 *  interest point calculations regarding a different model robot...  */
	static InterestPointMap ips;
};

//! a global instance of Kinematics, joint values reference those of WorldState so users can easily query the current spatial locations of joints
extern Kinematics * kine;

/*! @file
 * @brief Describes Kinematics, which provides access to the mathematical functionality of the roboop package using Tekkotsu data structures
 * @author ejt (Creator)
 */

#endif
