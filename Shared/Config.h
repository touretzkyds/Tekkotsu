//-*-c++-*-
#ifndef INCLUDED_Config_h
#define INCLUDED_Config_h

#include "Shared/plist.h"
#include "Shared/string_util.h"
#include "Shared/RobotInfo.h"
extern "C" {
#include <jpeglib.h>
}
#ifdef PLATFORM_APERIOS
#  include <OPENR/OPrimitiveControl.h>
#else
typedef unsigned int OSpeakerVolume; //!< provides an OPEN-R type of the same name for compatability, see Config::sound_config::volume
const OSpeakerVolume ospkvolinfdB = 0x8000; //!< 'muted' volume level, see Config::sound_config::volume
const OSpeakerVolume ospkvol25dB  = 0xe700; //!< low volume, see Config::sound_config::volume
const OSpeakerVolume ospkvol18dB  = 0xee00; //!< mid volume, see Config::sound_config::volume
const OSpeakerVolume ospkvol10dB  = 0xf600; //!< high volume, see Config::sound_config::volume
#endif
#include <vector>
#include <string>

//! a subclass of plist::Dictionary which adds support for filtering settings by robot model, each configuration section is based on this
/*! As a slight extension to standard plists, you can specify
 *  model-specific settings by prepending a key with:
 *     Model:MODEL_PATTERN:KEY_NAME
 *  For example, to use different 'thresh' settings on the ERS-2xx
 *  series vs. the ERS-7 model, you would use the keys:
 *   - Model:ERS-2*:thresh
 *   - Model:ERS-7:thresh
 *  You can filter several items as a group by leaving off the second
 *  ':' and name, and providing a dictionary as the value.  If the
 *  model matches, all entries from the dictionary are imported into
 *  the parent dictionary.
 */
class ConfigDictionary : public plist::Dictionary {
public:
	ConfigDictionary() : plist::Dictionary(false)
	{
		if(curModel.size()==0)
			curModel=RobotName;
	}
	
protected:
	
	virtual bool loadXMLNode(const std::string& key, xmlNode* val, const std::string& comment);
	virtual bool saveOverXMLNode(xmlNode* k, xmlNode* val, const std::string& key, std::string comment, const std::string& indentStr, std::set<std::string>& seen) const;

	//! returns true if pattern matches model - pattern may have up to 1 '*', case insensitive
	static bool matchNoCase(const std::string& model, const std::string& pattern);
	
	static const std::string msPrefix; //!< the prefix indicating a model-specific key
	static const std::string msSep; //!< the separator string which splits the key into the prefix, model pattern, and (optionally) a single key name
	//static const std::string msNum;
	static std::string curModel; //!< will be set to the model identifier of the currently running hardware
	static void initCurModel(); //!< function to initialize #curModel
};

//! Stores an item for each joint, can be accessed via joint name or array offset
/*! Two main reasons for this class are to keep outputs in offset order (not alphabetic by name),
 *  and to strip '~' from output names for backward compatability (used to pad output names
 *  to ensure constant string length. */
template<typename T>
class OutputConfig : public ConfigDictionary {
public:
	//! constructor
	OutputConfig(unsigned int firstOffset, unsigned int numOutputsToStore, const T& defValue)
	: ConfigDictionary(), offset(firstOffset), outputs(numOutputsToStore,defValue)
	{
		for(unsigned int i=0; i<numOutputsToStore; ++i)
			addEntry(outputNames[i+firstOffset],outputs[i]);
	}
	//! strips trailing '~' characters from name before doing usual lookup
	virtual ObjectBase& getEntry(const std::string& name) {
		std::string::size_type n = name.find_last_not_of('~')+1;
		return ConfigDictionary::getEntry((n==name.size()) ? name : name.substr(0,n));
	}
	//! return the value at position @a index, which must exist (no range checking)
	T& getEntry(size_t index) { return outputs[index]; }
	//! return the value at position @a index, which must exist (no range checking, equivalent to getEntry(index))
	T& operator[](size_t index) { return outputs[index]; }
	using ConfigDictionary::operator[];
	
	//! small trick -- override to save outputs in indexed order instead of alphabetical order
	void saveXML(xmlNode* node, bool onlyOverwrite, std::set<std::string>& seen) const;
	using ConfigDictionary::saveXML;
	
protected:
	//! strips trailing '~' characters from name before doing usual lookup
	bool loadXMLNode(const std::string& key, xmlNode* val, const std::string& comment) {
		std::string::size_type n = key.find_last_not_of('~')+1;
		return ConfigDictionary::loadXMLNode((n==key.size()) ? key : key.substr(0,n), val, comment);
	}
	
	unsigned int offset; //!< the index offset of the first entry in #outputs
	std::vector<T> outputs; //!< storage of data for each output
};

//! the root configuration object, provides some global functionality like porting pathnames (portPath())
class Config : public ConfigDictionary {
public:
	Config(const std::string& filename="") : ConfigDictionary(),
		behaviors(), wireless(), main(), controller(), vision(), motion(this), sound(this), fsRoot()
	{
		addEntry("wireless",wireless);
		addEntry("vision",vision);
		addEntry("main",main);
		addEntry("behaviors",behaviors);
		addEntry("controller",controller);
		addEntry("motion",motion);
		addEntry("sound",sound);
		if(filename.size()!=0)
			loadFile(filename.c_str());
	}
	
	enum transports { UDP, TCP }; //!< types of network transport protocols supported
	static const unsigned int NUM_TRANSPORTS=2; //!< number of #transports available
	static const char * transport_names[NUM_TRANSPORTS+1]; //!< string names for #transports
	
	void setFileSystemRoot(const std::string& fsr); //!< sets #fsRoot
	const std::string& getFileSystemRoot() const { return fsRoot; } //!< returns #fsRoot
	
	//! returns a portable version of @a path which should be usable on either the simulator or the robot
	std::string portPath(const std::string& path) const;
	
	//!returns an absolute path if @a is relative (to root), otherwise just @a name
	std::string makePath(const std::string& name) const { return searchPath(name,"config"); }

	//! searches for @a name at root/target/name, root/name, and finally just name, returns root/target/name if not found.
	std::string searchPath(const std::string& name, const std::string& root) const;
	
	
	
	// **************************************** //
	//! place for users to put their own configuration
	// **************************************** //
	/*! you can dynamically "link in" external configuration settings by passing them to the addEntry() of the
	 *  plist::Dictionary superclass.  You may want to call writeParseTree() first to flush current settings,
	 *  and then readParseTree() afterward to pull any pre-existing values from the configuration
	 *  file into the instances you've just registered.
	 *
	 *  Of course, you could also just write your values into the configuration file first, and just rely
	 *  on getEntry/setEntry to read/write the value.  This may be more convenient if you use the
	 *  value infrequently and don't need an instance of it sitting around. */
	class behaviors_config : public ConfigDictionary {
	public:
		//! constructor
		behaviors_config() : ConfigDictionary(), flash_bytes(4), flash_on_start(true)
		{
			//users may want to dynamically link in their own settings when running, so don't warn if there are
			// unused items in this section
			setUnusedWarning(false);
			
			addEntry("flash_bytes",flash_bytes,"how many bytes of the address to flash\n"
							 "You probably already know the first 3 bytes for your network,\n"
							 "so you might only want the last byte for brevity.\n"
							 "(valid values are 1 through 4) ");
			addEntry("flash_on_start",flash_on_start,"whether or not to automatically trigger on boot\n"
							 "Will use a priority of kEmergencyPriority+1 in order to override\n"
							 "the emergency stop's status animation ");
		}
		plist::Primitive<unsigned int> flash_bytes; //!< how many bytes of the IP to flash
		plist::Primitive<bool> flash_on_start;      //!< whether or not to trigger flashing when initially started
	} behaviors;

	
	
	// **************************************** //
	//! wireless configuration options
	// **************************************** //
	class wireless_config : public ConfigDictionary {
	public:
		//! constructor
		wireless_config () : ConfigDictionary(), id(1) {
			addEntry("id",id,"id number (in case you have more than one AIBO)");
		}
		plist::Primitive<int> id; //!< id number (in case you have more than one AIBO)
	} wireless;
	
	
	
	// **************************************** //
	//! general configuration options
	// **************************************** //
	class main_config : public ConfigDictionary {
	public:
		//!constructor
		main_config()
			: ConfigDictionary(), seed_rng(true), console_port(10001), consoleMode(CONTROLLER,consoleModeNames), stderr_port(10002),
			wsjoints_port(10031),wspids_port(10032),gamepadControl_port(10041),headControl_port(10052),armControl_port(10054),
			walkControl_port(10050),estopControl_port(10053),stewart_port(10055),aibo3d_port(10051),
			wmmonitor_port(10061), use_VT100(true), worldState_interval(0)
		{
			addEntry("seed_rng",seed_rng,"if true, will call srand with timestamp data, mangled by current sensor data");
			addEntry("console_port",console_port,"port to send/receive \"console\" information on (separate from system console)");
			addEntry("consoleMode",consoleMode,"determines how input on console_port is interpreted\n"+consoleMode.getDescription()+"\n"
							 "  'controller' will pass it as input to the Controller (assumes same syntax as ControllerGUI)\n"
							 "  'textmsg' will broadcast it as a TextMsgEvent (textmsgEGID)\n"
							 "  'auto' is the original mode, which uses 'textmsg' if the ControllerGUI is connected, and 'controller' otherwise ");
			addEntry("stderr_port",stderr_port,"port to send error information to");
			addEntry("wsjoints_port",wsjoints_port,"port to send joint positions on");
			addEntry("wspids_port",wspids_port,"port to send pid info on");
			addEntry("gamepad", gamepadControl_port, "port to receive gamepad data\n");
			addEntry("headControl_port",headControl_port,"port for receiving head commands");
                        addEntry("armControl_port",armControl_port,"port for receiving arm commands");
			addEntry("walkControl_port",walkControl_port,"port for receiving walk commands");
			addEntry("estopControl_port",estopControl_port,"port for receiving walk commands");
			addEntry("stewart_port",stewart_port,"port for running a stewart platform style of control");
			addEntry("aibo3d_port",aibo3d_port,"port for send/receive of joint positions from Aibo 3D GUI");
			addEntry("wmmonitor_port",wmmonitor_port,"port for monitoring Watchable Memory");
			addEntry("use_VT100",use_VT100,"if true, enables VT100 console codes (currently only in Controller menus - 1.3)");
			addEntry("worldState_interval",worldState_interval,"time (in milliseconds) to wait between sending WorldState updates over wireless");
		}
		plist::Primitive<bool> seed_rng;     //!< if true, will call srand with timestamp data, mangled by current sensor data
		plist::Primitive<int> console_port;  //!< port to send/receive "console" information on (separate from system console)
		//! types that #consoleMode can take on
		enum consoleMode_t {
			CONTROLLER, //!< all input is interpreted as Controller commands, using same syntax as the GUI sends
			TEXTMSG, //!< all input is broadcast as a text message event
			AUTO //!< if the GUI is connected, interpret as text messages, otherwise as controller commands.  This was the historical behavior, but seems to be more confusing than helpful.
		};
		static const unsigned int NUM_CONSOLE_MODES=3; //!< the number of consoleMode_t values available
		static const char* consoleModeNames[NUM_CONSOLE_MODES+1]; //!< string names for #consoleMode_t
		//! determines how input on #console_port is interpreted
		plist::NamedEnumeration<consoleMode_t> consoleMode; 
		plist::Primitive<int> stderr_port;   //!< port to send error information to
		plist::Primitive<int> wsjoints_port; //!< port to send joint positions on
		plist::Primitive<int> wspids_port;   //!< port to send pid info on
		plist::Primitive<int> gamepadControl_port; //!< port for gamepad
		plist::Primitive<int> headControl_port;	   //!< port for receiving head commands
                plist::Primitive<int> armControl_port;	   //!< port for receiving head commands
		plist::Primitive<int> walkControl_port;	   //!< port for receiving walk commands
		plist::Primitive<int> estopControl_port;	   //!< port for receiving walk commands
		plist::Primitive<int> stewart_port;  //!< port for running a stewart platform style of control
		plist::Primitive<int> aibo3d_port;   //!< port for send/receive of joint positions from Aibo 3D GUI
		plist::Primitive<int> wmmonitor_port; //!< port for monitoring Watchable Memory
		plist::Primitive<bool> use_VT100;    //!< if true, enables VT100 console codes (currently only in Controller menus - 1.3)
		plist::Primitive<unsigned int> worldState_interval; //!< time (in milliseconds) to wait between sending WorldState updates over wireless
	} main;

	
	
	// **************************************** //
	//! controller information
	// **************************************** //
	class controller_config : public ConfigDictionary {
	public:
		//!constructor
		controller_config() : ConfigDictionary(), gui_port(10020), select_snd(), next_snd(), prev_snd(), read_snd(), cancel_snd(), error_snd()
		{
			addEntry("gui_port",gui_port,"port to listen for the GUI to connect to aibo on");
			addEntry("select_snd",select_snd,"sound file to use for \"select\" action");
			addEntry("next_snd",next_snd,"sound file to use for \"next\" action");
			addEntry("prev_snd",prev_snd,"sound file to use for \"prev\" action");
			addEntry("read_snd",read_snd,"sound file to use for \"read from std-in\" action");
			addEntry("cancel_snd",cancel_snd,"sound file to use for \"cancel\" action");
			addEntry("error_snd",error_snd,"sound file to use to signal errors");
		}
	
		plist::Primitive<int> gui_port;        //!< port to listen for the GUI to connect to aibo on
		plist::Primitive<std::string> select_snd; //!< sound file to use for "select" action
		plist::Primitive<std::string> next_snd;   //!< sound file to use for "next" action
		plist::Primitive<std::string> prev_snd;   //!< sound file to use for "prev" action
		plist::Primitive<std::string> read_snd;   //!< sound file to use for "read from std-in" action
		plist::Primitive<std::string> cancel_snd; //!< sound file to use for "cancel" action
		plist::Primitive<std::string> error_snd; //!< sound file to use to signal errors
	} controller;

	
	
	// **************************************** //
	//! vision configuration options (this is a *big* section, with sub-sections)
	// **************************************** //
	class vision_config : public ConfigDictionary {
	public:
		//!constructor
		vision_config() : ConfigDictionary(), 
				  white_balance(WB_FLUORESCENT), gain(GAIN_MID), shutter_speed(SHUTTER_MID), resolution(1),
			thresh(), colors("config/default.col"), restore_image(true), region_calc_total(true),
			jpeg_dct_method(JDCT_IFAST,dct_method_names), aspectRatio(CameraResolutionX/(float)CameraResolutionY),
			x_range(), y_range(), x_focalLen(), y_focalLen(), // these four values depend on aspectRatio, will be initialized by aspectRatioListener constructor
				  rawcam(), depthcam(), segcam(), regioncam(),
			aspectRatioListener(aspectRatio,*this,&vision_config::aspectRatioChanged)
		{
			white_balance.addNameForVal("indoor",WB_INDOOR);
			white_balance.addNameForVal("outdoor",WB_OUTDOOR);
			white_balance.addNameForVal("fluorescent",WB_FLUORESCENT);
			white_balance.addNameForVal("flourescent",WB_FLUORESCENT); //catch common typo
			addEntry("white_balance",white_balance,"white balance shifts color spectrum in the image\n"+white_balance.getDescription());
			
			gain.addNameForVal("low",GAIN_LOW);
			gain.addNameForVal("mid",GAIN_MID);
			gain.addNameForVal("med",GAIN_MID);
			gain.addNameForVal("medium",GAIN_MID);
			gain.addNameForVal("high",GAIN_HIGH);
			addEntry("gain",gain,"Increasing gain will brighten the image, at the expense of more graininess/noise\n"+gain.getDescription());
			
			shutter_speed.addNameForVal("slow",SHUTTER_SLOW);
			shutter_speed.addNameForVal("low",SHUTTER_SLOW);
			shutter_speed.addNameForVal("mid",SHUTTER_MID);
			shutter_speed.addNameForVal("med",SHUTTER_MID);
			shutter_speed.addNameForVal("medium",SHUTTER_MID);
			shutter_speed.addNameForVal("fast",SHUTTER_FAST);
			shutter_speed.addNameForVal("high",SHUTTER_FAST);
			addEntry("shutter_speed",shutter_speed,"slower shutter will brighten image, but increases motion blur\n"+shutter_speed.getDescription());
			
			addEntry("resolution",resolution,"the resolution that object recognition system will run at.\nThis counts down from the maximum resolution layer, so higher numbers mean lower resolution. ");
			addEntry("thresh",thresh,"Threshold (.tm) files define the mapping from full color to indexed color.\n"
							 "You can uncomment more than one of these - they will be loaded into separate\n"
							 "channels of the segmenter.  The only cost of loading multiple threshold files is\n"
							 "memory - the CPU cost of performing segmentation is only incurred if/when\n"
							 "the channel is actually accessed for the first time for a given frame. ");
			addEntry("colors",colors,"The colors definition (.col) file gives names and a \"typical\" color for display.\n"
							 "The indexes numbers it contains correspond to indexes in the .tm file\n"
							 "This file is common to all .tm files; when doing new color segmentations,\n"
							 "make sure you define colors in the same order as listed here! ");
			addEntry("restore_image",restore_image,"Apparently someone at Sony thought it would be a good idea to replace some\n"
							 "pixels in each camera image with information like the frame number and CDT count.\n"
							 "If non-zero, will replace those pixels with the actual image pixel value in RawCamGenerator ");
			addEntry("region_calc_total",region_calc_total,"When true, this will fill in the CMVision::color_class_state::total_area\n"
							 "field for each color following region labeling.  If false, the total_area\n"
							 "will stay 0 (or whatever the last value was), but you save a little CPU. ");
			addEntry("jpeg_dct_method",jpeg_dct_method,"pick between dct methods for jpeg compression\n"+jpeg_dct_method.getDescription());
			
			/* these are *not* read from configuration file, but set from most recent camera image (via RawCameraGenerator, or RobotInfo namespace values until a camera image is been received):
			 - aspectRatio
			 - x_range
			 - y_range
			 */
			
			addEntry("rawcam",rawcam);
			addEntry("depthcam", depthcam);
			addEntry("segcam",segcam);
			addEntry("regioncam",regioncam);
		}
		
#ifdef PLATFORM_APERIOS
		//! white balance levels supported by the Aibo's camera
		enum white_balance_levels {
			WB_INDOOR=ocamparamWB_INDOOR_MODE,
			WB_OUTDOOR=ocamparamWB_OUTDOOR_MODE,
			WB_FLUORESCENT=ocamparamWB_FL_MODE
		};
#else
		//! white balance levels supported by the Aibo's camera
		enum white_balance_levels { WB_INDOOR=1, WB_OUTDOOR, WB_FLUORESCENT };
#endif
		plist::NamedEnumeration<white_balance_levels> white_balance; //!< white balance shifts color spectrum in the image
		
#ifdef PLATFORM_APERIOS
		//! gain levels supported by the Aibo's camera
		enum gain_levels {
			GAIN_LOW=ocamparamGAIN_LOW,
			GAIN_MID=ocamparamGAIN_MID,
			GAIN_HIGH=ocamparamGAIN_HIGH
		};
#else
		//! gain levels supported by the Aibo's camera
		enum gain_levels { GAIN_LOW=1, GAIN_MID, GAIN_HIGH };
#endif
		plist::NamedEnumeration<gain_levels> gain; //!< Increasing gain will brighten the image, at the expense of more graininess/noise

#ifdef PLATFORM_APERIOS
		//! shutter speeds supported by the Aibo's camera
		enum shutter_speeds {
			SHUTTER_SLOW=ocamparamSHUTTER_SLOW,
			SHUTTER_MID=ocamparamSHUTTER_MID,
			SHUTTER_FAST=ocamparamSHUTTER_FAST
		};
#else
		//! shutter speeds supported by the Aibo's camera
		enum shutter_speeds { SHUTTER_SLOW=1, SHUTTER_MID, SHUTTER_FAST };
#endif
		plist::NamedEnumeration<shutter_speeds> shutter_speed; //!< slower shutter will brighten image, but increases motion blur
		
		enum encoding_t {
			ENCODE_COLOR, //!< send Y, U, and V channels
			ENCODE_SINGLE_CHANNEL, //!< send only a single channel (which channel to send is stored in #channel) This is also used for all seg cam images
			ENCODE_DEPTH,
		};
		static const unsigned int NUM_ENCODINGS=3; //!< number of encodings available
		static const char * encoding_names[NUM_ENCODINGS+1]; //!< string names for #encoding_t
		
		plist::Primitive<int> resolution;       //!< the resolution that object recognition system will run at -- this counts down from the maximum resolution layer, so higher numbers mean lower resolution
		plist::ArrayOf<plist::Primitive<std::string> > thresh;      //!< threshold file names
		plist::Primitive<std::string> colors;      //!< colors definition (.col) file
		plist::Primitive<bool> restore_image;   //!< if true, replaces pixels holding image info with actual image pixels (as much as possible anyway)
		plist::Primitive<bool> region_calc_total; //!< if true, RegionGenerator will calculate total area for each color (has to run through the region list for each color)
		static const char * dct_method_names[]; //!< string names for #J_DCT_METHOD
		plist::NamedEnumeration<J_DCT_METHOD> jpeg_dct_method;  //!< pick between dct methods for jpeg compression
		plist::Primitive<float> aspectRatio;    //!< ratio of width to height (x_res/y_res); this is *not* read from configuration file, but set from most recent camera image (via RawCameraGenerator, or RobotInfo namespace values until a camera image is been received):
		float x_range;        //!< range of values for the x axis when using generalized coordinates; this is *not* read from configuration file, but set from most recent camera image (via RawCameraGenerator, or RobotInfo namespace values until a camera image is been received):
		float y_range;        //!< range of values for the y axis when using generalized coordinates; this is *not* read from configuration file, but set from most recent camera image (via RawCameraGenerator, or RobotInfo namespace values until a camera image is been received):
		float x_focalLen; //!< focal length of x axis: x_range / tan(RobotInfo::CameraHorizFOV); this is *not* read from configuration file, but set from most recent camera image (via RawCameraGenerator, or RobotInfo namespace values until a camera image is been received):
		float y_focalLen; //!< focal length of y axis: y_range / tan(RobotInfo::CameraVertFOV); this is *not* read from configuration file, but set from most recent camera image (via RawCameraGenerator, or RobotInfo namespace values until a camera image is been received):
		
		//! contains settings related to streaming video over the network
		class StreamingConfig : public ConfigDictionary {
		public:
			explicit StreamingConfig(int portNum) : ConfigDictionary(), port(portNum), transport(Config::UDP,transport_names), interval(0)
			{
				addEntry("port",port,"the port number to open for sending data over");
				addEntry("transport",transport,"the IP protocol to use when sending data");
				addEntry("interval",interval,"minimum amount of time (in milliseconds) which must pass between frames\nE.g. 200 yields just under 5 frames per second, 0 is as fast as possible");
			}
			plist::Primitive<unsigned short> port;
			plist::NamedEnumeration<Config::transports> transport;
			plist::Primitive<unsigned int> interval;
		};
		
		//! contains settings specific to the "RawCam" (original camera images) for streaming video over the network
		class RawCamConfig : public StreamingConfig {
		public:
			RawCamConfig() : StreamingConfig(10011),
				encoding(ENCODE_COLOR,encoding_names), channel(0), compression(COMPRESS_JPEG,compression_names), compress_quality(85), y_skip(2), uv_skip(3)
			{
				addEntry("encoding",encoding,"holds whether to send color or single channel\n"+encoding.getDescription());
				addEntry("channel",channel,"if encoding is single channel, this indicates the channel to send");
				addEntry("compression",compression,"the type of compression to apply the image\n"+compression.getDescription());
				addEntry("compress_quality",compress_quality,"0-100, compression quality (currently only used by jpeg)");
				addEntry("y_skip",y_skip,"resolution level to transmit y channel\nAlso used as the resolution level when in single-channel encoding mode ");
				addEntry("uv_skip",uv_skip,"resolution level to transmit uv channel at when using 'color' encoding mode");
			}
			//! type of information to send, stored in #encoding
			
			plist::NamedEnumeration<Config::vision_config::encoding_t> encoding; //!< holds whether to send color or single channel
			plist::Primitive<int> channel;    //!< RawCameraGenerator::channel_id_t, if #encoding is single channel, this indicates the channel to send

			//! compression format to use, stored in Config::vision_config::RawCamConfig::compression
			enum compression_t {
				COMPRESS_NONE, //!< no compression (other than subsampling)
				COMPRESS_JPEG, //!< JPEG compression
				COMPRESS_PNG, //!< PNG compression
			};
			static const unsigned int NUM_COMPRESSIONS=4; //!< number of compression algorithms available
			static const char * compression_names[NUM_COMPRESSIONS+1]; //!< string names for #compression_t
			plist::NamedEnumeration<compression_t> compression;//!< holds whether to send jpeg compression

			plist::Primitive<unsigned int> compress_quality;//!< 0-100, compression quality (currently only used by jpeg)
			plist::Primitive<unsigned int> y_skip;     //!< resolution level to transmit y channel at
			plist::Primitive<unsigned int> uv_skip;    //!< resolution level to transmit uv channel at
		} rawcam;

		class DepthCamConfig : public StreamingConfig {
		public:
			DepthCamConfig() : StreamingConfig(10014), compression(COMPRESS_JPEG, compression_names), compress_quality(85), y0_skip(2), y1_skip(2)
			{
				transport = Config::TCP;
				addEntry("compression",compression,"what compression to use on the segmented image"+compression.getDescription());
				addEntry("compress_quality",compress_quality,"0-100, compression quality (currently only used by jpeg)");
				addEntry("y0_skip",y0_skip,"resolution level to transmit y channel\nAlso used as the resolution level when in single-channel encoding mode ");
				addEntry("y1_skip",y1_skip,"resolution level to transmit uv channel at when using 'color' encoding mode");
			}
			//! compression format to use, stored in Config::vision_config::RawCamConfig::compression
			enum compression_t {
				COMPRESS_NONE, //!< no compression (other than subsampling)
				COMPRESS_JPEG, //!< JPEG compression
				COMPRESS_PNG   //!< PNG compression
			};
			static const unsigned int NUM_COMPRESSIONS=4; //!< number of compression algorithms available
			static const char * compression_names[NUM_COMPRESSIONS+1]; //!< string names for #compression_t
			plist::NamedEnumeration<compression_t> compression;//!< what compression to use on the segmented image
			plist::Primitive<unsigned int> compress_quality;//!< 0-100, compression quality (currently only used by jpeg)
			plist::Primitive<unsigned int> y0_skip;     //!< resolution level to transmit y channel at
			plist::Primitive<unsigned int> y1_skip; 
		} depthcam;
		
		//! contains settings specific to the "SegCam" (segmented color images) for streaming video over the network
		class SegCamConfig : public StreamingConfig {
		public:
			SegCamConfig() : StreamingConfig(10012), skip(1), channel(0), compression(COMPRESS_RLE, compression_names)
			{
				addEntry("skip",skip,"resolution level to transmit segmented images at");
				addEntry("channel",channel,"channel of RLEGenerator to send (i.e. which threshold map to use");
				addEntry("compression",compression,"what compression to use on the segmented image"+compression.getDescription());
			}
			plist::Primitive<unsigned int> skip;       //!< resolution level to transmit segmented images at
			plist::Primitive<unsigned int> channel;    //!< channel of RLEGenerator to send (i.e. which threshold map to use)
			
			//! compression format to use, stored in Config::vision_config::RawCamConfig::compression
			enum compression_t {
				COMPRESS_NONE, //!< no compression (other than subsampling)
				COMPRESS_RLE   //!< RLE compression
			};
			static const unsigned int NUM_COMPRESSIONS=4; //!< number of compression algorithms available
			static const char * compression_names[NUM_COMPRESSIONS+1]; //!< string names for #compression_t
			plist::NamedEnumeration<compression_t> compression;//!< what compression to use on the segmented image
		} segcam;
		
		//! contains settings specific to the "RegionCam" (only display a box for each blob of color) for streaming over the network
		class RegionCamConfig : public StreamingConfig {
		public:
			RegionCamConfig() : StreamingConfig(10013), skip(1) {
				addEntry("skip",skip,"resolution level to extract regions from");
			}
			plist::Primitive<unsigned int> skip; //!< resolution level to transmit segmented images at 
		} regioncam;
		
		//!provides a ray through camera origin from a generalized pixel coordinate
		/*!
		 * Ideally this would take camera calibration into account, e.g. Lens Distortion for Close-Range Photogrammetry -  D.C. Brown, Photometric Engineering, pages 855-866, Vol. 37, No. 8, 1971.
		 * or using 'Camera Calibration Toolbox for Matlab', by Jean-Yves Bouguet: http://www.vision.caltech.edu/bouguetj/calib_doc/
		 *
		 * However, at the moment this does a naïve computation mapping directly from field of view as specified by RobotInfo CameraFOV
		 * 
		 *  @param[in] x x position in range ±x_range (±1 assuming x is the major camera axis)
		 *  @param[in] y y position in range ±y_range (±1/#aspectRatio assuming x is the major camera axis)
		 *  @param[out] r_x x value of the ray
		 *  @param[out] r_y y value of the ray
		 *  @param[out] r_z z value of the ray (always 1) */
		void computeRay(float x, float y, float& r_x, float& r_y, float& r_z);
      
		//!provides a generalized pixel hit in image by a ray going through the camera origin
		/*! @param[in] r_x x value of the ray
		 *  @param[in] r_y y value of the ray
		 *  @param[in] r_z z value of the ray
		 *  @param[out] x x position in range ±x_range (±1 assuming x is the major camera axis)
		 *  @param[out] y y position in range ±y_range (±1/#aspectRatio assuming x is the major camera axis) */
		void computePixel(float r_x, float r_y, float r_z, float& x, float& y);
		//@}
		
		//!provides a generalized pixel hit in image by a ray going through the camera origin that is corrected via the homography
		/*! @param[in] r_x x value of the ray
		 *  @param[in] r_y y value of the ray
		 *  @param[in] r_z z value of the ray
		 *  @param[out] x x position in range ±x_range (±1 assuming x is the major camera axis)
		 *  @param[out] y y position in range ±y_range (±1/#aspectRatio assuming x is the major camera axis) */
		void computePixelCorrected(float r_x, float r_y, float r_z, float& x, float &y);
		
	protected:
		void aspectRatioChanged();
		plist::PrimitiveCallbackMember<vision_config> aspectRatioListener;
				
	} vision;
	
	
	
	//!motion information
	class motion_config : public ConfigDictionary {
	public:
		//!constructor
		motion_config(Config* c)
		: ConfigDictionary(), thisconfig(c), root("data/motion"), walk("walk.prm"), kinematics(std::string(RobotInfo::RobotName).append(".kin")),
			  calibration_scale(PIDJointOffset, NumPIDJoints,1), calibration_offset(PIDJointOffset, NumPIDJoints, 0), 
			  estop_on_snd(), estop_off_snd(), 
			max_head_tilt_speed(0), max_head_pan_speed(0), max_head_roll_speed(0), inf_walk_accel(false),
			console_port(10003), stderr_port(10004)
		{
			addEntry("root",root,"path on memory stick to \"motion\" files - for instance, position (.pos) and motion sequence (.mot)\n"
							 "Any motion related paths which are not absolute (i.e. do not start with '/')\n"
							 "will be assumed to be relative to this directory ");
			addEntry("walk",walk,"the walk parameter file to load by default for new WalkMC's");
			addEntry("kinematics",kinematics,"the ROBOOP format kinematics description file to load");
			addEntry("calibration_scale",calibration_scale,"Multiplier from desired position to command for each PID joint, applied after calibration_offset.");
			addEntry("calibration_offset",calibration_offset,"These values indicate the offset from user specified zero point to the\n"
							 "physical system's zero point.  Added before calibration_scale when\n"
							 "converting from user's desired position to command to send to hardware.");
			addEntry("estop_on_snd",estop_on_snd,"sound file to use when e-stop turned on");
			addEntry("estop_off_snd",estop_off_snd,"sound file to use when e-stop turned off");
			addEntry("max_head_tilt_speed",max_head_tilt_speed,"max speed for the head joints, used by HeadPointerMC; rad/s");
			addEntry("max_head_pan_speed",max_head_pan_speed,"max speed for the head joints, used by HeadPointerMC; rad/s");
			addEntry("max_head_roll_speed",max_head_roll_speed,"max speed for the head joints, used by HeadPointerMC; rad/s");
			addEntry("inf_walk_accel",inf_walk_accel,"if true, walks should attempt to switch directions immediately; otherwise they should do some kind of software acceleration to more smoothly switch direction");
			addEntry("console_port",console_port,"port to send/receive \"console\" information on (separate from system console)");
			addEntry("stderr_port",stderr_port,"port to send error information to");
		}
		
		//!returns an absolute path if @a is relative (to root), otherwise just @a name
		std::string makePath(const std::string& name) { return thisconfig->searchPath(name, root); }
		
		Config* thisconfig;  //!<pointer back to the containing config object
		plist::Primitive<std::string> root;       //!< path on memory stick to "motion" files - for instance, position (.pos) and motion sequence (.mot)
		plist::Primitive<std::string> walk;       //!< the walk parameter file to load by default for new WalkMC's
		plist::Primitive<std::string> kinematics;  //!< the kinematics description file to load
		OutputConfig<plist::Primitive<float> > calibration_scale; //!< Multiplier from desired position to command for each PID joint, applied after calibration_offset.
		OutputConfig<plist::Primitive<float> > calibration_offset; //!< Indicates the offset from user specified zero point to the physical system's zero point.  Added before calibration_scale when converting from user's desired position to command to send to hardware.
		plist::Primitive<std::string> estop_on_snd;  //!< sound file to use when e-stop turned on
		plist::Primitive<std::string> estop_off_snd; //!< sound file to use when e-stop turned off
		plist::Primitive<float> max_head_tilt_speed; //!< max speed for the head joints, used by HeadPointerMC; rad/s
		plist::Primitive<float> max_head_pan_speed; //!< max speed for the head joints, used by HeadPointerMC; rad/s
		plist::Primitive<float> max_head_roll_speed; //!< max speed for the head joints, used by HeadPointerMC; rad/s
		plist::Primitive<bool> inf_walk_accel; //!< if true, walks should attempt to switch directions immediately; otherwise they should do some kind of software acceleration to more smoothly switch direction
		plist::Primitive<int> console_port;  //!< port to send/receive "console" information on (separate from system console)
		plist::Primitive<int> stderr_port;   //!< port to send error information to
		
	private:
		motion_config(const motion_config&); //!< don't call
		motion_config& operator=(const motion_config&); //!< don't call
	} motion;

	
	
	//!sound information
	class sound_config : public ConfigDictionary {
	public:
		//!constructor
		sound_config(Config* c) : ConfigDictionary(), thisconfig(c), root("data/sound"), verbose(0), volume(HIGH), sample_rate(16000),
			sample_bits(16), preload(), pitchConfidenceThreshold(.6f), streaming()
		{
			addEntry("root",root,"path to sound clips");
			addEntry("verbose",verbose,"Controls diagnostic information on the console:\n  0 - none\n  1 - report when sounds begin to play\n  2 - also report when sounds finish playing\n  3- also report when sounds are preloaded and released");
			volume.addNameForVal("mute",MUTE);
			volume.addNameForVal("level_1",LOW);
			volume.addNameForVal("low",LOW);
			volume.addNameForVal("level_2",MID);
			volume.addNameForVal("mid",MID);
			volume.addNameForVal("level_3",HIGH);
			volume.addNameForVal("high",HIGH);
			volume.setStrict(false);
			addEntry("volume",volume,"volume in decibels - the value is interpreted as a signed short, where\n"
					"0x8000 is mute, 0xFFFF is full volume (low=0xE700, mid=0xEE00, high=0xF600)\n"
					"If you directly set the decibel level, be warned sony recommends against going above 0xF600\n"
					"However, I believe the commercial software on the ERS-7 runs at 0xFF00.\n"
					"Going above 0xF800 on a ERS-210 causes distortion (clipping) - full volume on a ERS-7 sounds fine though.\n"+volume.getDescription());
			addEntry("sample_rate",sample_rate,"sample rate to send to system, currently only 8000 or 16000 supported");
			addEntry("sample_bits",sample_bits,"sample bit depth, either 8 or 16");
			addEntry("preload",preload,"list of sounds to preload at boot");
			addEntry("pitchConfidenceThreshold",pitchConfidenceThreshold,"confidence threshold required to generate a pitch event [0-1]");
			addEntry("streaming",streaming);
		}
	
		//!returns an absolute path if @a is relative (to root), otherwise just @a name
		std::string makePath(const std::string& name) { 
			if(name[0]=='/')
				return thisconfig->portPath(name);
			if(root[root.size()-1]=='/')
				return thisconfig->portPath(root+name);
			else
				return thisconfig->portPath(root+"/"+name);
		}
	
		Config* thisconfig;  //!<pointer back to the containing config object
		plist::Primitive<std::string> root;         //!< path to sound clips
		
		//! Controls diagnostic information on the console
		/*! 0 - none\n
		 *  1 - report when sounds begin to play\n
		 *  2 - also report when sounds finish playing\n
		 *  3- also report when sounds are preloaded and released */
		plist::Primitive<int> verbose;
		
		//! Provides some symbolic volume levels, although values are based on actual volume in decibels.
		/*! The value is interpreted as a signed short, where 0 is full volume, 0x8000 is mute */
		enum volume_levels { MUTE=ospkvolinfdB, LOW=ospkvol25dB, MID=ospkvol18dB, HIGH=ospkvol10dB };
		plist::NamedEnumeration<volume_levels> volume;      //!< volume in decibels - the value is interpreted as a signed short, where 0 is full volume, 0x8000 is mute
		
		plist::Primitive<unsigned int> sample_rate; //!< sample rate to send to system, currently only 8000 or 16000 supported
		plist::Primitive<unsigned int> sample_bits; //!< sample bit depth, either 8 or 16
		plist::ArrayOf<plist::Primitive<std::string> > preload; //!< list of sounds to preload at boot
		plist::Primitive<float> pitchConfidenceThreshold; //!< confidence threshold required to generate a pitch event [0-1]
		
		//! audio streaming configuration
		class streaming_config : public ConfigDictionary {
		public:
			//! constructor
			streaming_config() : ConfigDictionary(), mic_port(10070), mic_sample_rate(16000),
				mic_sample_bits(16), mic_stereo(true),
				speaker_port(10071), speaker_frame_length(64),
				speaker_max_delay(1000)
			{
				addEntry("mic_port",mic_port,"port for streaming microphone samples");
				addEntry("mic_sample_rate",mic_sample_rate,"sample rate from the microphone");
				addEntry("mic_sample_bits",mic_sample_bits,"sample bit depth from the microphone (either 8 or 16)");
				addEntry("mic_stereo",mic_stereo,"whether to stream stereo or mono from the microphone");
				
				addEntry("speaker_port",speaker_port,"port for streaming speaker samples");
				addEntry("speaker_frame_length",speaker_frame_length,"length of frame sent to the speaker (ms)");
				addEntry("speaker_max_delay",speaker_max_delay,"maximum delay (ms) during playback");
			}
			plist::Primitive<unsigned int> mic_port;        //!< port for streaming microphone samples
			plist::Primitive<unsigned int> mic_sample_rate; //!< sample rate from the microphone
			plist::Primitive<unsigned int> mic_sample_bits; //!< sample bit depth from the microphone (either 8 or 16)
			plist::Primitive<bool> mic_stereo; //!< whether to stream stereo or mono from the microphone
			
			plist::Primitive<unsigned int> speaker_port;    //!< port for streaming speaker samples
			plist::Primitive<unsigned int> speaker_frame_length; //!< length of frame sent to the speaker (ms)
			plist::Primitive<unsigned int> speaker_max_delay; //!< maximum delay (ms) during playback
		} streaming;
	private:
		sound_config(const sound_config&); //!< don't call
		sound_config& operator=(const sound_config&); //!< don't call
	} sound;

	
	
	virtual void saveXML(xmlNode* node, bool onlyOverwrite, std::set<std::string>& seen) const;
	using ConfigDictionary::saveXML;
	virtual unsigned int loadBuffer(const char buf[], unsigned int len, const char* filename=NULL);
	virtual unsigned int loadFile(const char* filename);
	virtual unsigned int loadFileStream(FILE* f, const char* filename=NULL);
	
	//! pass the section, item name string, item value string - sets the value and returns pointer to the item changed
	/*! this is the older deprecated interface -- use the inherited resolveEntry() instead to support sub-sections */
	void* setValue(const std::string& section, std::string key, const std::string& value);
	
protected:
	static const char * xmlIntro1;
	static const char * xmlIntro2;
	static const char * xmlIntro3;
	
	static const std::locale& curLocale;
	static char localeToLower(char c) { return std::tolower(c,curLocale); };
	
	unsigned int loadOldFormat(const char buf[], unsigned int len);
	unsigned int loadOldFormat(FILE* f);
	void parseLine(const char buf[], unsigned int lineno, std::vector<std::string>& modelStack, bool& ignoring, std::string& section);
	
	//! a prefix representing the file system root, usually indicating the robot's storage root.
	/*! When running in the simulator, this is used to pretend that a subdirectory in the project folder (e.g. 'ms') is the root file system */
	std::string fsRoot;
};

template<typename T>
void OutputConfig<T>::saveXML(xmlNode* node, bool onlyOverwrite, std::set<std::string>& seen) const {
	// first just overwrite what's already there as we normally do
	ConfigDictionary::saveXML(node,true,seen);
	// now if there's anything left...
	if(!onlyOverwrite && seen.size()!=dict.size()) {
		// clear text nodes from end of dictionary back to last entry
		for(xmlNode* cur=xNodeGetLastChild(node); cur!=NULL && xNodeIsText(cur); cur=xNodeGetLastChild(node)) {
			xmlUnlinkNode(cur);
			xmlFreeNode(cur);
		}
		size_t longestKeyLen = getLongestKeyLen(NULL,1);
		std::string indentStr=getIndentationPrefix(node);
		// this is the customized part -- save in "natural" output order instead of alphabetically by key
		for(unsigned int i=0; i<outputs.size(); ++i) {
			if(seen.find(outputNames[i+offset])==seen.end())
				saveXMLNode(node,outputNames[i+offset],&outputs[i],indentStr,longestKeyLen);
		}
		std::string parentIndent;
		if(indentStr.size()>=perIndent().size())
			parentIndent=indentStr.substr(perIndent().size());
		xmlAddChild(node,xmlNewText((const xmlChar*)("\n"+parentIndent).c_str()));
	}
}

//!allows global access to current settings
extern Config* config;

/*! @file
 * @brief Describes Config, which provides global access to system configuration information
 * @author ejt (Creator)
 * @author alokl (Original configuration system)
 */

#endif
