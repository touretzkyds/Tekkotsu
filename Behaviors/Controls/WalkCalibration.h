//-*-c++-*-
#ifndef INCLUDED_WalkCalibration_h_
#define INCLUDED_WalkCalibration_h_

#include "ControlBase.h"
#include "StringInputControl.h"
#include "FileInputControl.h"
#include "ToggleControl.h"
#include "Events/EventRouter.h"

//! allows interactive calibration of a walk engine
/*! 
 *  In retrospect, this might have been a little easier to do as a
 *  ControlBase/StateNode multiple inheritance instead of
 *  ControlBase/EventListener... oh well.
 */
class WalkCalibration : public ControlBase, public EventListener {
public:
	//! constructor
	WalkCalibration();
	
	//! destructor
	~WalkCalibration();

	virtual ControlBase * activate(MC_ID disp_id, Socket * gui);

	virtual void refresh();

	virtual void deactivate() {
		erouter->removeListener(this);
		ControlBase::deactivate();
	}

	virtual ControlBase* doSelect();

	//! listens for locomotion events so we know when to stop recording
	virtual void processEvent(const EventBase& e);

	virtual ControlBase * takeInput(const std::string& msg);
	
	virtual void setHilights(const std::vector<unsigned int>& hi);

	virtual void hilightFirst();

protected:
	enum {
		ROOT,      //!< indicates the root menu is currently displayed (save/load data sets...)
		CHOOSE,    //!< indicates the sample type selection is displayed
		READY,     //!< waiting for user to indicate 0 point
		MOVING,    //!< recording, waiting for the motion stop event
		READING_1, //!< waiting for user to supply the first measurement coordinate
		READING_2, //!< waiting for user to supply the second measurement coordinate
		CLEAR,     //!< clear data confirmation menu
	} st; //!< the currently active state

	//! allows representation of the current sample type
	enum dataSource {
		fs, //!< forward-sideways
		fr, //!< forward-rotate
		sr, //!< sideways-rotate
		br, //!< backward-rotate
		bs, //!< backward-sideways
		r,  //!< pure rotation
		NUM_SRC //!< number of data types
	} curType; //!< the currently selected type of data being recorded	

	static void loadData(const std::string& name, std::vector<float*>& data); //!< does the work of loading data sets
	static void saveData(const std::string& name, const std::vector<float*>& data); //!< does the work of saving data sets
	static void clearData(std::vector<float*>& data); //!< clears current data

	void setupRoot(); //!< sets state to root menu
	void setupChoose(); //!< sets state to choose type menu
	void setupReady(); //!< sets state to ready menu
	void setupMoving(); //!< sets state to moving menu
	void setupReading1(); //!< sets state to enter first measurement menu
	void setupReading2(); //!< sets state to enter second measurement menu
	void setupClear(); //!< sets state to clear confirmation menu

	unsigned int getType(); //!< returns current sample type
	unsigned int getFirstIndex(dataSource t); //!< returns a name index for the first measurement type
	unsigned int getSecondIndex(dataSource t); //!< returns a name index for the second measurement type
	const char * getIndexName(unsigned int t); //!< returns name for measurement type
	const char * getFirstMeasure(dataSource t); //!< returns name for first measurement type
	const char * getSecondMeasure(dataSource t); //!< returns name for second measurement type

	float arclen(float d, float a, float sign); //!< calculates arc distance corresponding to a displacement and angle, negated if necessary to match @a sign
	void addSample(); //!< adds data point corresponding to measurements stored in #firstIn and #secondIn
	void addSample(std::vector<float*>& dat, float x, float y, float a); //!< adds data point to @a dat for a given x,y,a (and #old_x, #old_y, #old_a)
	static void report(unsigned int row, float cmd, float actual); //!< reports data as they are being taken
	static void err(const std::string& str); //!< pops up an error message on the controller and also displays it on the console

	float old_x; //!< the x velocity recorded when we started moving
	float old_y; //!< the y velocity recorded when we started moving
	float old_a; //!< the a velocity recorded when we started moving
	unsigned int startTime; //!< the time recording started
	unsigned int stopTime; //!< the time recording stopped

	ControlBase * help; //!< control holding help info
	FileInputControl * load; //!< control for loading data
	StringInputControl * save; //!< control for saving data
	ControlBase * measure; //!< control for taking measurements
	ControlBase * clear; //!< control for clearing data
	ToggleControl * polar; //!< control for selecting polar measurements for fs/bs
	ToggleControl * rect; //!< control for selecting cartesian measurements for fs/bs
	bool isPolar; //!< true if polar measurements is selected
	std::string lastLoad; //!< name last data file selected (so we know if a new has been selected

	float firstIn; //!< the input given for the first measurement
	float secondIn;  //!< the input given for the second measurement
	std::vector<float*> data[NUM_SRC]; //!< an array of vectors of sample points (one veotor for each of the sample types)
	unsigned int cnts[NUM_SRC]; //!< count of samples for each sample type (cnts[i] should equal data[i].size())
	static const char * datanames[NUM_SRC]; //!< name for each sample type

	std::string status; //!< string to send for ControllerGUI status message (current count of each sample type)
	
private:
	WalkCalibration(const WalkCalibration& ); //!< don't call
	WalkCalibration& operator=(const WalkCalibration& ); //!< don't call
};

/*! @file
 * @brief Describes WalkCalibration, which allows interactive calibration of a walk engine
 * @author ejt (Creator)
 */

#endif
