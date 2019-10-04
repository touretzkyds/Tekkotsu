#include "PitchDetector.h"
#include "Events/EventRouter.h"
#include "Events/EventBase.h"
#include "Events/DataEvent.h"
#include "Events/PitchEvent.h"
#include "Shared/newmat/newmatap.h"
#include "Shared/Config.h"
#include "Shared/debuget.h"

#include "Shared/ODataFormats.h"
#ifdef PLATFORM_APERIOS
#  include "OPENR/OPENRAPI.h"
#endif

REGISTER_BEHAVIOR_MENU_OPT(PitchDetector,"Background Behaviors/System Daemons",BEH_NONEXCLUSIVE|BEH_START);

using namespace std; 

const unsigned int PitchDetector::fft_frames = 4; // number frames to use for fft
const unsigned int PitchDetector::num_pitches = 60; // 5 octaves
const float PitchDetector::base_pitch = 110.0f; // two octaves below tuning A (440)
const float PitchDetector::half_step = 1.0594630943593f; // twelfth root of two
const float PitchDetector::sqrt_2_pi = 2.506628274631f; // \sqrt{2\pi}

PitchDetector::~PitchDetector() {
	ASSERT(pitch_info==NULL,"pitch_info wasn't deleted before destructor");
	ASSERT(pitch_bin==NULL,"pitch_bin wasn't deleted before destructor");
}

void PitchDetector::doStart() {
	EventGeneratorBase::doStart(); // do this first (required)
	
	ASSERT(pitch_info==NULL,"pitch_info was already allocated?");
	ASSERT(pitch_bin==NULL,"pitch_bin was already allocated?");
	
	pitch_info = new PitchInfo[num_pitches];
	for (unsigned int i = 0; i != num_pitches; ++i) {
		float freq = base_pitch * powf(half_step, i);
		pitch_info[i].freq = freq;
		pitch_info[i].sigma = sqrtf((freq * half_step - freq) / 0.5f);
		pitch_info[i].duration = 0;
	}

	// doEvent will initialize before use
	pitch_bin = new float[num_pitches];

	cur_frame = 0;
	have_fft = false;
	//printf("writing to file..\n");
	//fprintf(fft_file, "\n======starting fft collection=======\n");
	//printf("done writing to file..\n");
}

void PitchDetector::doStop() {
	//fclose(fft_file);
	if(pitch_info!=NULL) {
		delete [] pitch_info;
		pitch_info=NULL;
	}
	if(pitch_bin!=NULL) {
		delete [] pitch_bin;
		pitch_bin=NULL;
	}
	EventGeneratorBase::doStop(); // do this last (required)
}

void PitchDetector::doEvent() {
	if( event->getGeneratorID() != EventBase::micOSndEGID)
		return;
	
	// Get to the sound buffer
	// getData() is not specified for const data
	unsigned int i, j;
	const DataEvent<const OSoundVectorData*> *de = reinterpret_cast<const DataEvent<const OSoundVectorData*>*>( &event);
	
	OSoundVectorData *svd = const_cast<OSoundVectorData*>(de->getData());
	const short *d = ( const short *)svd->GetData(0);
	
	if ( ! frame_sz ) { /* we need to initialize _everything_ */
		//printf("building vectors for first time..\n");
		frame_sz = svd->GetInfo(0)->frameSize;
		rate = svd->GetInfo(0)->samplingRate;
		//printf("frame_sz %d, rate %d\n",frame_sz,rate);
		win_sz = frame_sz * fft_frames;
		
		left.ReSize(win_sz);
		right.ReSize(win_sz);
		iml.ReSize(win_sz / 2 + 1);
		imr.ReSize(win_sz / 2 + 1);
		rel.ReSize(win_sz / 2 + 1);
		rer.ReSize(win_sz / 2 + 1);
		pol.ReSize(win_sz / 2 + 1);
		por.ReSize(win_sz / 2 + 1);
		po.ReSize(win_sz / 2 + 1);
	}
	
	//printf("saving audio data to vectors [%u]..\n", cur_frame);
	for (i = 0; i != frame_sz; ++i) {
		left ((cur_frame * frame_sz) + i + 1) = d[(i<<1)  ];
		right((cur_frame * frame_sz) + i + 1) = d[(i<<1)+1];
	}
	
	if (++cur_frame == fft_frames) {
		cur_frame = 0;
		
		hamming(left);
		hamming(right);
		
		//printf("calling fft!\n");
		NEWMAT::RealFFT(left, rel, iml);
		NEWMAT::RealFFT(right, rer, imr);
		for (i = 1; i <= win_sz / 2 + 1; ++i) {
			NEWMAT::Real a, b;
			a = rel(i);
			b = iml(i);
			pol(i) = sqrtf(a*a + b*b);
			a = rer(i);
			b = imr(i);
			por(i) = sqrtf(a*a + b*b);
			//based on whether stereo info actually used, remove all
			//stereo separation altogether or merge even later..
			po(i) = (pol(i) + por(i)) / 2;
			
			//fprintf(fft_file, "[frequency %f] pow[%d] = %f\n", rate * i * 1.0 / win_sz, i, po(i));
		}
		have_fft = true;
		
	} else if (cur_frame == 1 && have_fft) { //hack to split processing..
		float mean = 0.0f;
		unsigned int max = 0; //if we see this value twice.. oops
		//turbo slow for now..
		//printf("building pitch bins!\n");
		local_maxes = 0;
		for (i = 0; i != num_pitches; ++i) {
			float sigma = pitch_info[i].sigma;
			float freq = pitch_info[i].freq;
			float bin = 0.0f;
			
			for (j = 1; j <= win_sz / 2 + 1; ++j)
				bin += po(j) * gaussian_pdf(j * rate * 1.f / win_sz, sigma, freq);
			mean += (pitch_bin[i] = bin);
			
			
			//prep for global max check
			max = (bin > pitch_bin[max]) ? i : max;
			
			//check if prev a local max
			pitch_info[i].local_max = pitch_info[i].global_max = 0.0f;
			if (i == 1) {
				float prev = pitch_bin[i-1];
				if (bin < prev) {
					pitch_info[0].local_max = 1.0f - (bin / prev);
					++local_maxes;
				}
			} else if (i > 1) {
				float a = pitch_bin[i-2], b = pitch_bin[i-1], c = pitch_bin[i];
				if (b > a && b > c) {
					pitch_info[i-1].local_max = 1.0f - (a + c) / (2.f * b);
					++local_maxes;
				}
			}
			if (i == num_pitches - 1) { //intentionally not else-if !
				float prev = pitch_bin[i - 1];
				if (bin > prev) {
					pitch_info[i].local_max = 1.0f - (prev / bin);
					++local_maxes;
				}
			}
		}
		mean /= num_pitches;
		
		pitch_info[max].global_max = 1.0f - mean / pitch_bin[max];
		
		//final pass through on whether this is a pitch or not.
		//compute overtone properties and confidence..
		for (i = 0; i < num_pitches; ++i) {
			float c, f = 1.0f;
			if (i % 4 && is_pitch(confidence(i/4, pitch_bin[i/4])))
				f /= 2.0f;
			if (i % 3 && is_pitch(confidence(i/3, pitch_bin[i/3])))
				f /= 2.0f;
			if (i % 3 && is_pitch(confidence(i*2/3, pitch_bin[i*2/3])))
				f /= 2.0f;
			if (i % 2 && is_pitch(confidence(i/2, pitch_bin[i/2])))
				f /= 2.0f;
			pitch_info[i].overtone = 1.0f - f;
			pitch_info[i].confidence = (c = confidence(i, pitch_bin[i]));
			
			if (is_pitch(c)) {
				//printf("pitch number %u, frequency %f, name %s, confidence %f went on\nstrength %f gmax %f lmax %f otone %f lmaxes %u\n", i, pitch_info[i].freq, pitch_name(i), c, pitch_bin[i],pitch_info[i].global_max, pitch_info[i].local_max, pitch_info[i].overtone, local_maxes);
				EventBase::EventTypeID_t type = ( ! pitch_info[i].duration ) ? EventBase::activateETID : EventBase::statusETID;
				pitch_info[i].amplitude = (pitch_info[i].amplitude*pitch_info[i].duration + pitch_bin[i]) / (pitch_info[i].duration + 1);
				++pitch_info[i].duration;
				erouter->postEvent(PitchEvent(reinterpret_cast<size_t>(this), type, pitch_info[i].freq, pitch_name(i), pitch_bin[i], pitch_info[i].duration*win_sz*1000/rate, c));
			} else {
				if (pitch_info[i].duration) {
					//printf("pitch number %u, frequency %f, name %s, confidence %f, duration %u went off \n",i, pitch_info[i].freq, pitch_name(i), c, pitch_info[i].duration);
					erouter->postEvent(PitchEvent(reinterpret_cast<size_t>(this), EventBase::deactivateETID,pitch_info[i].freq, pitch_name(i),pitch_info[i].amplitude,pitch_info[i].duration*win_sz*1000/rate,c));
					pitch_info[i].duration = 0;
					pitch_info[i].amplitude = 0;
				}
			}
			//fprintf(fft_file, "pitch %d freq %f name %s amp %f dur %u\n\tgmax %f lmax %f otone %f lmaxes %u confidence %f\n",i, pitch_info[i].freq, pitch_name(i),pitch_bin[i], pitch_info[i].duration,pitch_info[i].global_max, pitch_info[i].local_max,pitch_info[i].overtone, local_maxes, c);
		}
	}
	//printf("done with mic event in class Pitch\n");
}

bool PitchDetector::is_pitch(float conf) {
	return (conf >= config->sound.pitchConfidenceThreshold);
}



/*! @file
 * @brief Implements PitchDetector, which generates a PitchEvent whenever a notable frequency is detected using FFT
 * @author Matus Telgarsky and Jonah Sherman (Creators)
 * @author Ethan Tira-Thompson (imported into framework)
 *
 * Originally written as a part of a final project at Carnegie Mellon (15-494 Cognitive Robotics, Spring 2006)
 */
