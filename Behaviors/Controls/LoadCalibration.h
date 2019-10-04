//-*-c++-*-
#ifndef INCLUDED_LoadCalibration_h_
#define INCLUDED_LoadCalibration_h_

#include "FileBrowserControl.h"
#include "Motion/WalkMC.h"

//! will load calibration parameters from a text file into a forward and backward matrix
/*! normally the calibration parameters are stored in the binary walk parameter file itself,
 *  but this will load a new set of calibration parameters from a text file as written by
 *  the tools/walk_calibration/WalkCalibration.m matlab script. */
class LoadCalibration : public FileBrowserControl {
public:

	//!Constructor
	LoadCalibration(WalkMC::CalibrationParam* calp)
		: FileBrowserControl("Load Calibration...","",config->motion.root), cp(calp)
	{ setFilter("*.txt"); }
	//!Constructor
	LoadCalibration(const std::string& n,WalkMC::CalibrationParam* calp)
		: FileBrowserControl(n,"",config->motion.root), cp(calp)
	{ setFilter("*.txt"); }
	//!Constructor
	LoadCalibration(const std::string& n, const std::string& d,WalkMC::CalibrationParam* calp)
		: FileBrowserControl(n,d,config->motion.root), cp(calp)
	{ setFilter("*.txt"); }

protected:
	static const unsigned int ROWS=3;  //!< number of degrees of freedom of movement (probably won't change)
	static const unsigned int COLS=11; //!< number of basis functions (may change)

	//!calls readData() for each of the matrices
	virtual ControlBase* selectedFile(const std::string& f) {
		FILE * file=fopen(config->portPath(f).c_str(),"r");
		if(readMaxs(file))
			if(readData(file,cp->f_calibration))
				readData(file,cp->b_calibration);
		fclose(file);
		FileBrowserControl::selectedFile(f);
		return NULL;
	}

	//!does the actual loading once a file is selected
	bool readData(FILE * f, float mat[ROWS][COLS]) {
		const unsigned int curl=500;
		char curs[curl];
		for(unsigned int i=0; i<ROWS; i++) {
			fgets(curs,curl,f);
			if(feof(f)) {
				serr->printf("ERROR: short read, got %d lines.\n",1+(mat==cp->f_calibration?i:i+ROWS));
				return false;
			} else {
				char *c=curs;
				for(unsigned int col=0; col<COLS; col++)
					mat[i][col]=static_cast<float>(strtod(c,&c));
			}
		}
		return true;
	}

	//!reads the maximum forward, backward, strafe, and rotation velocities
	bool readMaxs(FILE * f) {
		const unsigned int curl=500;
		char curs[curl];
		fgets(curs,curl,f);
		if(feof(f)) {
			serr->printf("ERROR: short read, got %d lines.\n",0);
			return false;
		} else {
			char *c=curs;
			for(unsigned int col=0; col<WalkMC::CalibrationParam::NUM_DIM; col++)
				cp->max_vel[col]=static_cast<float>(strtod(c,&c));
		}
		return true;
	}

	WalkMC::CalibrationParam * cp; //!< pointer to the calibration parameter structure

private:
	LoadCalibration(const LoadCalibration& ); //!< don't call
	LoadCalibration& operator=(const LoadCalibration& ); //!< don't call
};

/*! @file
 * @brief Defines LoadCalibration, which will load calibration parameters from a text file into a forward and backward matrix
 * @author ejt (Creator)
 */

#endif
