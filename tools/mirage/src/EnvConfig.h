//-*-c++-*-
#ifndef INCLUDED_EnvConfig_h_
#define INCLUDED_EnvConfig_h_

#include "Shared/plist.h"
#include "Motion/KinematicJoint.h"
#include "Physics.h"

namespace EnvConfig {
	const std::string& getCurrentLoadingFile(); //!< returns EnvConfig::singleton().getCurrentLoadingFile()
	
	class Object : virtual public plist::Dictionary {
	public:
		Object() : plist::Dictionary(), location(), orientation(), pointAt() {
			addEntry("Location",location,"Controls position of object");
			addEntry("Orientation",orientation,"The (x,y,z) components of a quaternion controlling object orientation, except if PointAt is set.");
			addEntry("PointAt",pointAt,"If this is a length-3 array, will override Orientation to point the z axis at the specified point.");
			setLoadSavePolicy(FIXED,SYNC);
		}
		plist::Point location;
		plist::Point orientation;
		#ifdef DOCDUMP
		plist::Point pointAt;
		#else
		plist::ArrayOf<plist::Primitive<float> > pointAt;
		#endif
	};
	
	class Camera : public Object {
	public:
		Camera() : Object(), autoFollow() {
			addEntry("AutoFollow",autoFollow,"Name of an object to track, specify as clientName#path/to/joint, e.g. default value RobotID-1#BaseFrame");
		}
		plist::Primitive<std::string> autoFollow; //!< name of the client and joint being tracked so the client can disconnect and come back and we'll resume tracking
	};
	
	class Light : virtual public plist::Dictionary, public Object {
	public:
		Light() : plist::Dictionary(), Object(), color(0.375f,0.375f,0.375f),
			attenuateRange(30000), attenuateConst(0.35), attenuateLinear(0), attenuateQuad(0)
		{
			addEntry("Color",color,"Color of the light, by default #606060.");
			addEntry("AttenuateRange",attenuateRange,"The maximum range the light will reach before being cut off, default 30000");
			addEntry("AttenuateConst",attenuateConst,"The constant attenuation factor c: light = 1/(c + l·dist + q·dist²), default 0.35");
			addEntry("AttenuateLinear",attenuateLinear,"The linear attenuation factor l: light = 1/(c + l·dist + q·dist²), default 0");
			addEntry("AttenuateQuad",attenuateQuad,"The quadratic attenuation factor q: light = 1/(c + l·dist + q·dist²), default 0");
			setLoadSavePolicy(FIXED,SYNC);
		}
		
		plist::RGBColor<float> color;
		plist::Primitive<float> attenuateRange;
		plist::Primitive<float> attenuateConst;
		plist::Primitive<float> attenuateLinear;
		plist::Primitive<float> attenuateQuad;
	};
	
	class PhysicalObject : virtual public plist::Dictionary, public Object, public LinkComponent {
	public:
		PhysicalObject() : plist::Dictionary(), Object(), LinkComponent(),
			kinematics(), components(), frictionForce(0.5f), anistropicFrictionRatio(1,1,1),
			sourceFile(getCurrentLoadingFile())
		{
			addEntry("Kinematics",kinematics,"Pass the path of a .kin kinematics file to load for this object.");
			addEntry("Components",components,"Directly define a series of LinkComponents to construct the object.");
			addEntry("FrictionForce",frictionForce,"Conversion from velocity to friction force (default 0.5)");
			addEntry("AnistropicFrictionRatio",anistropicFrictionRatio,"Direction sensitivity of friction, '1' in all directions means it is not direction sensitive");
			setLoadSavePolicy(FIXED,SYNC);
		}
		PhysicalObject& operator=(const PhysicalObject& o) {
			Object::operator=(o);
			LinkComponent::operator=(o);
			kinematics=o.kinematics;
			components=o.components;
			frictionForce=o.frictionForce;
			anistropicFrictionRatio=o.anistropicFrictionRatio;
			sourceFile=o.sourceFile;
			return *this;
		}
		plist::Primitive<std::string> kinematics; //!< path to a .kin file, absolute paths are first checked relative to #sourceFile, then current working directory
		plist::ArrayOf<LinkComponent> components; //!< a list of components to be assembled as a compound collision object
		plist::Primitive<float> frictionForce; //!< conversion from velocity to friction force (default 0.5)
		plist::Point anistropicFrictionRatio; //!< direction sensitivity of friction, '1' in all directions means it is not direction sensitive
		std::string sourceFile; //!< file from which this object was loaded, to aid in searching for #kinematics
	private:
		PhysicalObject(const PhysicalObject&); //!< don't copy
	};
	
	class Background : virtual public plist::Dictionary {
	public:
		enum model_t { NONE, PLANE, BOX, DOME };
		static const char* const modelNames[5];
		
		Background() : plist::Dictionary(), model(NONE,modelNames), color(), material("CloudySky"), curvature(-100), tiling(5), distance(5000) {
			model.setPreferredNameForVal("None",NONE);
			addEntry("Model",model,"Type of background to use (default None)\n"+model.getDescription());
			addEntry("Color",color,"Solid color if no Material is specified (default black)");
			addEntry("Material",material,"An Ogre material name (default CloudySky)");
			addEntry("Curvature",curvature,"Curvature used for Plane and Dome models (default -100)");
			addEntry("Tiling",tiling,"Tiling factor for Plane and Dome models (default 5)");
			addEntry("Distance",distance,"The distance parameter for Box and Dome models (default 5000)");
			setLoadSavePolicy(FIXED,SYNC);
		}
		
		plist::NamedEnumeration<model_t> model;
		plist::RGBColor<float> color;
		plist::Primitive<std::string> material;
		plist::Primitive<float> curvature;
		plist::Primitive<unsigned int> tiling;
		plist::Primitive<float> distance;
	};
	
	class Shadows : virtual public plist::Dictionary {
	public:
		Shadows() : plist::Dictionary(),
#ifdef __linux__
		enabled(false), // linux drivers aren't generally as good about supporting this
#else
		enabled(true), // other platforms are probably capable
#endif
		stencil(true), modulative(true), color(0.75f,0.75f,0.75f) {
			addEntry("Enabled",enabled,"Must be set to true for any of the other parameters to apply (default true)");
			addEntry("Stencil",stencil,"If true uses stencil shadow generation (CPU based), otherwise uses texture shadow generation (more GPU based); default true");
			addEntry("Modulative",modulative,"If true uses simpler method where shadows are subtracted from final scene, otherwise uses more accurate additive model where lit areas are rendered for each light and added together.");
			addEntry("Color",color,"If Modulative is set, this controls the color of the shadow.");
			setLoadSavePolicy(FIXED,SYNC);
		}
		plist::Primitive<bool> enabled;
		plist::Primitive<bool> stencil;
		plist::Primitive<bool> modulative;
		plist::RGBColor<float> color;
	};
	
	//! description of EnvConfig
	class Environment : virtual public plist::Dictionary {
		friend Environment& singleton();
	public:
		plist::Primitive<std::string> resourceRoot;
		plist::Primitive<float> fps;
		plist::Primitive<float> timeScale;
		plist::Primitive<float> massDisplayScale;
		plist::Primitive<float> inertiaDisplayScale;
		Background background;
		plist::RGBColor<float> ambientLight;
		plist::DictionaryOf<Light> lights;
		Camera camera;
		plist::DictionaryOf<PhysicalObject> objects;
		Shadows shadows;
		
		enum MoCapLevel {
			MOCAP_NONE,
			MOCAP_ROOT,
			MOCAP_LEAVES,
			MOCAP_ALL
		};
		static const char* const mocapLevelNames[5];
		plist::NamedEnumeration<MoCapLevel> mocapPos;
		plist::NamedEnumeration<MoCapLevel> mocapOri;
		
		//! default scene setup, lights, camera, and scenery
		void resetDefaults();
		//! default objects: ground and reference frame:
		void resetScenery();
		
		virtual unsigned int loadFile(const char* filename);
		const std::string& getCurrentLoadingFile() const { return curLoadingFile; }
		
	protected:
		//! constructor, use getEnvironment()
		Environment()
			: plist::Dictionary(), resourceRoot(), fps(25), timeScale(1), massDisplayScale(40), inertiaDisplayScale(0), background(),
			ambientLight(.5f,.5f,.5f), lights(), camera(), objects(), shadows(), mocapPos(MOCAP_ROOT,mocapLevelNames), mocapOri(MOCAP_ROOT,mocapLevelNames)
		{
			addEntry("RenderFPS",fps,"Frames per second for the user window.");
			addEntry("TimeScale",timeScale,"Scales time used for physics simulation, e.g. a value of 0.5 runs in slow-motion.");
			addEntry("MassDisplayScale",massDisplayScale,"Scale used for displaying mass and inertia, factor to convert mass to spatial units.");
			addEntry("InertiaDisplayScale",inertiaDisplayScale,"Scale used for displaying mass and inertia, factor to convert inertia to spatial units.");
			addEntry("Background",background,"Parameters related to the sky coloring.");
			addEntry("AmbientLight",ambientLight,"Color of unlit areas, see also Shadows.  (default #808080)");
			addEntry("Lights",lights,"A collection of lights.  By default one light is included, 'light1'");
			addEntry("Observer",camera,"Position and orientation of user camera.");
			addEntry("Objects",objects,"Initial objects to populate the scene, including the ground itself.");
			addEntry("Shadows",shadows,"Parameters related to shadow generation.");
			addEntry("Physics",Physics::singleton(),"Parameters related to physics simulation.");
			addEntry("MoCapPos",mocapPos,"Controls which reference frame positions will be sent with each sensor update");
			addEntry("MoCapOri",mocapOri,"Controls which reference frame orientations will be sent with each sensor update");
			setLoadSavePolicy(FIXED,SYNC);
		}
		std::string curLoadingFile; //!< during a call to loadFile, this stores the filename so PhysicalObject can search for relative paths
	};
	
	Environment& singleton();	
}

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
