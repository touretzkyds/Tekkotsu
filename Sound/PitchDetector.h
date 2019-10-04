//-*-c++-*-
#ifndef INCLUDED_PitchDetector_h_
#define INCLUDED_PitchDetector_h_

#include "Events/EventGeneratorBase.h"
#include "Shared/newmat/newmat.h"
#include <cmath>

//! Generates a PitchEvent whenever a notable frequency is detected using FFT
class PitchDetector : public EventGeneratorBase {
public:
	static const unsigned int fft_frames; //!< number frames to use for fft
	static const unsigned int num_pitches; //!< 5 octaves, 12 pitches per octave
	static const float base_pitch; //!< two octaves below tuning A (440)
	static const float half_step; //!< twelfth root of two: @f$ \sqrt[12]{2} @f$
	static const float sqrt_2_pi; //!< set to @f$ \sqrt{2\pi} @f$
	
	//! constructor
	PitchDetector() : EventGeneratorBase("PitchDetector",EventBase::micPitchEGID,reinterpret_cast<size_t>(this),EventBase::micOSndEGID), 
		left(), right(), iml(), imr(), rel(), rer(), pol(), por(), po(), 
		frame_sz(0), win_sz(0), rate(0), cur_frame(0), local_maxes(0),
		pitch_info(NULL), pitch_bin(NULL), have_fft(false) //,fft_file(0)
	{
		//fft_file = fopen("ms/data/sound/fft.dat", "w");
	} 
	
	//! destructor, asserts that #pitch_info and #pitch_bin have been deleted by doStop()
	~PitchDetector();
	
	virtual void doStart(); //!< allocates and sets up #pitch_info and #pitch_bin, relies on EventGeneratorBase to manage event subscriptions
	virtual void doStop(); //!< deletes #pitch_info and #pitch_bin
	virtual void doEvent();

	static std::string getClassDescription() { return "Generates a PitchEvent whenever a notable frequency is detected using FFT"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	

protected:
	//! stores info about the pitch currently being detected
	struct PitchInfo {
		//! constructor -- sets everything to 0 (additional initialization is done for each #pitch_info entry during doStart())
		PitchInfo() : freq(0), sigma(0), local_max(0), global_max(0), overtone(0), confidence(0), amplitude(0), duration(0) {}
		
		float freq, //!< frequency of this pitch (calculated on instantiation)
		sigma,//!< standard dev to use (sqrt(var)), calc'd on start
		//following set only relevant if the pitch is on
		local_max, //!< [0,1] value: how much stronger it is than neighbors, else zero
		global_max, //!< [0,1] value: how much stronger than mean if global max, else zero
		overtone, //!< value in [0,1] with confidence that it is overtone            
		confidence, //!< value in [0,1] with confidence that this is a pitch, not noise (should confidence have a rolling average as well?)
		amplitude; //!< rolling average amplitude
		unsigned int duration; //!< number of windows it has been on 
	};
	
	//keep temporaries for efficiency
	NEWMAT::ColumnVector left, //!< the waveform of the left channel 
		right, //!< the waveform of the right channel
		iml, //!< imaginary outputs of the FFT for left channel
		imr, //!< imaginary outputs of the FFT for right channel
		rel, //!< real outputs of the FFT for the left channel
		rer, //!< real outputs of the FFT for the right channel
		pol, //!< absolute value (magnitude) of results of FFT for left channel
		por, //!< absolute value (magnitude) of results of FFT for right channel
		po; //!< average of #pol and #por for each bin
	unsigned int frame_sz, //!< number of samples given by system for each frame (assumed that once started, this won't change)
		win_sz, //!< number of samples to be recorded before running FFT (#frame_sz * #fft_frames)
		rate, //!< sampling frequence (Hz)
		cur_frame, //!< the current frame index to be filled in next, up to #fft_frames, when the FFT is run
		local_maxes; //!< number of individual peaks
	PitchInfo *pitch_info; //!< an array of PitchInfos, one for each of #num_pitches, allocated for scope spanning doStart()/doStop()
	float *pitch_bin; //!< array, holds current amplitude for each #num_pitches (mono)
	bool have_fft; //!< set to true after FFT has been computed, differentiates first filling of buffers from subsequent rewrites
	//FILE *fft_file;
	
	//! returns true if the confidence is above a threshold obtained from configuration
	static bool is_pitch(float conf);
	
	//! returns a confidence estimate of a pitch in bin @a p... (actual computation used is not obvious, you'll have to stare at the code /ejt)
	inline float confidence(unsigned int p, float strength) {
		float l = pitch_info[p].local_max,
		g = pitch_info[p].global_max,
		o = pitch_info[p].overtone;
		//XXX this sucks. add variance
		
		if (strength < win_sz * 5.0f)
			return 0.0f;
		
		if (g > 0.0f)
			return 3*g / 4.0f + (1.0f - o) / 8.0f + (1.0f - 2.0f * local_maxes / num_pitches) / 8.0f;
		else
			return l / 3.0f + (1.0f - o) / 4.0f + (1.0f - 2.0f * local_maxes / num_pitches) / 8.0f;
	}
	
	//! returns the value at @a x of a gaussian with the parameters @a mu and @a sigma
	static inline float gaussian_pdf(float mu, float sigma, float x) {
		float dist = x - mu;
		return std::exp(-dist * dist / (2.0f * sigma * sigma)) / (sqrt_2_pi * sigma);
	}
	
	//! returns a string representing the musical note of a given frequency
	static const char *pitch_name(unsigned int i) {
		static const char *pitch_names[12] = {
			"A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#"
		};
		
		return pitch_names[i % 12];
	}
	
	//! unknown calculation, does some kind of rescaling of @a v (/ejt)
	static void hamming(NEWMAT::ColumnVector &v) {
		for (int i = 0; i != v.size(); ++i)
			v.element(i) = v.element(i) * (0.53836f - 0.46164f * cosf(2 * float(M_PI) * i / (v.size() - 1)));
	}
	
private:
	PitchDetector(const PitchDetector&); //!< don't call (copy constructor)
	PitchDetector& operator=(const PitchDetector&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines PitchDetector, which generates a PitchEvent whenever a notable frequency is detected using FFT
 * @author Matus Telgarsky and Jonah Sherman (Creators)
 * @author Ethan Tira-Thompson (imported into framework)
 *
 * Originally written as a part of a final project at Carnegie Mellon (15-494 Cognitive Robotics, Spring 2006)
 */

#endif
