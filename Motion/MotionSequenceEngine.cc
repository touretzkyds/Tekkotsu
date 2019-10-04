#include "MotionSequenceEngine.h"
#include "DynamicMotionSequence.h"
#include "Shared/get_time.h"
#include "Shared/WorldState.h"
#include "Shared/Config.h"
#include <iostream>

using std::cout;
using std::endl;

MotionSequenceEngine::Move_idx_t MotionSequenceEngine::invalid_move=(MotionSequenceEngine::Move_idx_t)-1;

MotionSequenceEngine::MotionSequenceEngine()
: LoadSave(), playtime(1), lasttime(0), endtime(0), playspeed(1.0f),
playing(true), hold(true), loadSaveMode(1)
{
	for(unsigned int i=0; i<NumOutputs; i++)
		curstamps[i]=-1U;
}


int MotionSequenceEngine::updateOutputs() {
	if(isPlaying()) {
		if(lasttime==0)
			play();
		unsigned int curtime=get_time();
		float diff=(curtime-lasttime)*playspeed;
		if(playtime<-diff)
			setTime(0);
		else
			setTime(static_cast<unsigned int>(diff+playtime));
		lasttime=curtime;
		return 1;
	} else {
		lasttime=get_time();
		return 0;
	}
}

const OutputCmd& MotionSequenceEngine::getOutputCmd(unsigned int i) {
	if(curstamps[i]!=playtime) {
		if(nexts[i]!=invalid_move)
			calcOutput(curs[i],playtime,getKeyFrame(prevs[i]),getKeyFrame(nexts[i]));
		else if(hold)
			curs[i]=getKeyFrame(prevs[i]).cmd;
		else
			curs[i].unset();
		curstamps[i]=playtime;
	}
	return curs[i];
}

unsigned int MotionSequenceEngine::getBinSize() const {
	char buf[128];
	unsigned int len=128;
	unsigned int used=strlen("#MSq\n");
	used+=snprintf(buf,len,isSaveRadians()?"radians\n":"degrees\n");
	unsigned int t=0;
	Move_idx_t tprevs[NumOutputs];
	Move_idx_t tnexts[NumOutputs];
	bool hasInitialFrame=false;
	for(unsigned int i=0;i<NumOutputs;i++) {
		tnexts[i]=getKeyFrame(tprevs[i]=starts[i]).next;
		if(getKeyFrame(starts[i]).cmd.weight!=0)
			hasInitialFrame=true;
	}
	if(hasInitialFrame)
		used+=snprintf(buf,len,"setTime\t0\n");
	while(t!=-1U) {
		for(unsigned int i=0; i<NumOutputs; i++) {
			if((t!=0 || getKeyFrame(tprevs[i]).cmd.weight!=0) && getKeyFrame(tprevs[i]).starttime==t) {
				if(getKeyFrame(tprevs[i]).cmd.weight==1)
					used+=snprintf(buf,len,"%s\t%g\n",outputNames[i],getKeyFrame(tprevs[i]).cmd.value/loadSaveMode);
				else
					used+=snprintf(buf,len,"%s\t%g\t%g\n",outputNames[i],getKeyFrame(tprevs[i]).cmd.value/loadSaveMode,getKeyFrame(tprevs[i]).cmd.weight);
			}
		}
		unsigned int last=t;
		t=setNextFrameTime(tprevs,tnexts);
		if(t!=-1U)
			used+=snprintf(buf,len,"advanceTime\t%d\n",t-last);
	}
	used+=strlen("#END\n");
	return used+1;
}

unsigned int MotionSequenceEngine::loadBuffer(const char buf[], unsigned int len, const char* filename) {
	unsigned int origlen=len;
	if(strncmp("#POS",buf,4)==0) {
		// allow inlined loading of posture files
		PostureEngine pose;
		unsigned int used=pose.loadBuffer(buf,len);
		if(used!=0)
			setPose(pose);
		return used;
	}
	if(strncmp("#MSq",buf,4)!=0) {
		// we don't want to display an error here because we may be only testing file type,
		// so it's up to the caller to decide if it's necessarily an error if the file isn't
		// a motion sequence
		//cout << "ERROR MotionSequenceEngine load corrupted - expected #MSq header" << endl;
		return 0;
	}
	unsigned int linenum=1;
	unsigned int lastOutputIdx=0;
	while(len<=origlen && len>0) {
		int written;
		//printf("%d %.9s\n",linenum+1,buf);
		if(buf[0]=='\r') {
			buf++; len--;
			if(buf[0]=='\n') {
				buf++; len--;
			}
			linenum++;
			continue;
		}
		if(buf[0]=='\n') {
			buf++; len--;
			linenum++;
			continue;
		}
		if(buf[0]=='#') {
			if(strncmp("#END\n",buf,5)==0 || strncmp("#END\r",buf,5)==0) {
				return origlen-len+5;
			} else if(strncmp("#END\r\n",buf,6)==0) {
				return origlen-len+6;
			} else {
				while(len>0 && *buf!='\n' && *buf!='\r') {len--;buf++;}
				if(*buf=='\n') { //in case of \r\n
					buf++;
					len--;
				}
				linenum++;
				continue;
			}
		}
		written=-1;
		const unsigned int cmdlen=64, arglen=32;
		char command[cmdlen];
		char arg1[arglen];
		char arg2[arglen];
		written=readWord(buf,&buf[len],command,cmdlen);
		if(!checkInc(written,buf,len,"*** ERROR MotionSequenceEngine load corrupted - line %d\n",linenum)) return 0;
		written=readWord(buf,&buf[len],arg1,arglen);
		if(written>0)
			if(!checkInc(written,buf,len,"*** ERROR MotionSequenceEngine load corrupted - line %d\n",linenum)) return 0;
		written=readWord(buf,&buf[len],arg2,arglen);
		if(written!=0)
			if(!checkInc(written,buf,len,"*** ERROR MotionSequenceEngine load corrupted - line %d\n",linenum)) return 0;
		for(;len>0 && *buf!='\n' && *buf!='\r';buf++,len--) {}
		if(*buf=='\n') { //in case of \r\n
			buf++;
			len--;
		}

		if(strcasecmp(command,"delay")==0 || strcasecmp(command,"advanceTime")==0) {
			char* used;
			int delay = strtol(arg1,&used,0);
			if(*used!='\0') {
				cout << "*** WARNING illegal delay argument: " << arg1 << " - line " << linenum << endl;
			} else {
				setTime(playtime+delay);
			}
		} else if(strcasecmp(command,"settime")==0) {
			char* used;
			int newtime = strtol(arg1,&used,0);
			if(*used!='\0') {
				cout << "*** WARNING illegal settime argument: " << arg1 << " - line " << linenum << endl;
			} else {
				setTime(newtime);
			}
		} else if(strcasecmp(command,"load")==0 || strcasecmp(command,"overlay")==0) {
			PostureEngine pose;
			if(pose.loadFile(arg1)!=0) {
				// it's a posture file
				setPose(pose);
			} else if(overlayMotion(arg1)!=0) {
				// it's a motionsequence file (conditional test did the overlay)
			} else {
				// unknown file, give warning
				cout << "*** WARNING could not read file " << arg1 << " for overlay - line " << linenum << endl;
			}
		} else if(strcasecmp(command,"loadExplicit")==0) {
			PostureEngine pose;
			if(pose.loadFile(arg1)!=0) {
				setExplicitPose(pose);
			} else
				cout << "*** WARNING could not read file " << arg1 << " for load (loadExplicit only accepts posture files) - line " << linenum << endl;
		} else if(strcasecmp(command,"degrees")==0) {
			cout << "*** WARNING 'degrees' is deprecated, please specify all angles as radians" << endl;
			loadSaveMode=(float)M_PI/180; //setSaveDegrees();
		} else if(strcasecmp(command,"radians")==0) {
			setSaveRadians();
		} else {
			lastOutputIdx=getOutputIndex(command,lastOutputIdx+1);
			bool found = (lastOutputIdx!=NumOutputs);
			unsigned int lidx=lastOutputIdx,ridx=NumOutputs;
			if(!found) { // base name not found
				// try symmetric left/right versions
				char tname[cmdlen+1];
				strncpy(tname+1,command,cmdlen);
				tname[0]='L';
				lidx=getOutputIndex(tname,lastOutputIdx+1);
				if(lidx!=NumOutputs) {
					tname[0]='R';
					ridx=getOutputIndex(tname,lidx+1);
					if(ridx!=NumOutputs)
						found=true;
				}
			}
			if (!found) {
				cout << "*** WARNING " << command << " is not a valid joint on this model." << endl;
			} else {
				char* used;
				float value=(float)strtod(arg1,&used), weight=1;
				if(*used!='\0')
					cout << "*** WARNING illegal value argument: " << arg1 << " - line " << linenum << endl;
				else {
					if(arg2[0]!='\0') {
						weight=(float)strtod(arg2,&used);
						if(*used!='\0') {
							 cout << "*** WARNING illegal weight argument: " << arg2 << " - line " << linenum << endl;
							 weight=1;
						}
					}
					if(lastOutputIdx!=NumOutputs) {
						// found exact match earlier
						setOutputCmd(lastOutputIdx,OutputCmd(value*loadSaveMode,weight));
					} else {
						// found symmetric matches earlier
						setOutputCmd(lidx,OutputCmd(value*loadSaveMode,weight));
						setOutputCmd(ridx,OutputCmd(value*loadSaveMode,weight));
						lastOutputIdx=ridx;
					}
				}
			}
		}

		linenum++;

	}
	cout << "*** WARNING MotionSequenceEngine load missing #END" << endl;
	return origlen-len;
}

unsigned int MotionSequenceEngine::saveBuffer(char buf[], unsigned int len) const {
	//std::cout << "SAVEBUFFER..." << std::flush;
	unsigned int origlen=len;
	int written=snprintf(buf,len,"#MSq\n");
	if(!checkInc(written,buf,len,"*** ERROR MotionSequenceEngine save failed on header\n")) return 0;	if(len==0 || len>origlen) {
		cout << "*** ERROR MotionSequenceEngine save overflow on header" << endl;
		return 0;
	}
	written=snprintf(buf,len,isSaveRadians()?"radians\n":"degrees\n");
	if(!checkInc(written,buf,len,"*** ERROR MotionSequenceEngine save failed on mode\n")) return 0;	if(len==0 || len>origlen) {
		cout << "*** ERROR MotionSequenceEngine save overflow" << endl;
		return 0;
	}
	unsigned int t=0;
	Move_idx_t tprevs[NumOutputs];
	Move_idx_t tnexts[NumOutputs];
	bool hasInitialFrame=false;
	for(unsigned int i=0;i<NumOutputs;i++) {
		tnexts[i]=getKeyFrame(tprevs[i]=starts[i]).next;
		if(getKeyFrame(starts[i]).cmd.weight!=0)
			hasInitialFrame=true;
	}
	if(hasInitialFrame) {
		written=snprintf(buf,len,"setTime\t0\n");
		if(!checkInc(written,buf,len,"*** ERROR MotionSequenceEngine save failed on initial frame spec\n")) return 0;
	}
	while(t!=-1U) {
		//std::cout << "t="<<t<<"..."<<std::endl;
		for(unsigned int i=0; i<NumOutputs; i++) {
			//cout << i << ' ' << outputNames[i] << " (" << getKeyFrame(tprevs[i]).cmd.value << ',' << getKeyFrame(tprevs[i]).cmd.weight << ") " << getKeyFrame(tprevs[i]).starttime << " state: " << starts[i] <<' '<< prevs[i] <<' '<< nexts[i] << " cur: " << getKeyFrame(tprevs[i]).prev << ' ' << tprevs[i] << ' ' << getKeyFrame(tprevs[i]).next << endl;
			//first conditional is to skip 0 weighted values in first frame
			if((t!=0 || getKeyFrame(tprevs[i]).cmd.weight!=0) && getKeyFrame(tprevs[i]).starttime==t) {
				if(getKeyFrame(tprevs[i]).cmd.weight==1) {
					written=snprintf(buf,len,"%s\t%g\n",outputNames[i],getKeyFrame(tprevs[i]).cmd.value/loadSaveMode);
					if(!checkInc(written,buf,len,"*** ERROR MotionSequenceEngine save failed\n")) return 0;
				} else {
					written=snprintf(buf,len,"%s\t%g\t%g\n",outputNames[i],getKeyFrame(tprevs[i]).cmd.value/loadSaveMode,getKeyFrame(tprevs[i]).cmd.weight);
					if(!checkInc(written,buf,len,"*** ERROR MotionSequenceEngine save failed\n")) return 0;
				}
				if(len==0 || len>origlen) {
					cout << "*** ERROR MotionSequenceEngine save overflow" << endl;
					return 0;
				}
			}
		}
		unsigned int last=t;
		t=setNextFrameTime(tprevs,tnexts);
		if(t!=-1U) {
			written=snprintf(buf,len,"delay\t%d\n",t-last);
			if(!checkInc(written,buf,len,"*** ERROR MotionSequenceEngine save failed\n")) return 0;
			if(len==0 || len>origlen) {
				cout << "*** ERROR MotionSequenceEngine save overflow" << endl;
				return 0;
			}
		}
	}
	written=snprintf(buf,len,"#END\n");
	if(!checkInc(written,buf,len,"*** ERROR MotionSequenceEngine save failed on #END\n")) return 0;
	if(len==0 || len>origlen) {
		cout << "*** ERROR MotionSequenceEngine save overflow on #END" << endl;
		return 0;
	}
	return origlen-len;
	cout << "SAVE-done!" << endl;
}

unsigned int MotionSequenceEngine::loadFile(const char filename[]) {
	return LoadSave::loadFile(config->motion.makePath(filename).c_str());
}
unsigned int MotionSequenceEngine::saveFile(const char filename[]) const {
	return LoadSave::saveFile(config->motion.makePath(filename).c_str());
}

/*! @deprecated use radians instead (we don't have a good way to know which output values are angles, and which are 'other', e.g. wheel velocities or LED brightness...) */
void MotionSequenceEngine::setSaveDegrees() { loadSaveMode=(float)M_PI/180; }

void MotionSequenceEngine::setTime(unsigned int x) {
	playtime=x;
	const WorldState * st=WorldState::getCurrent();
	for(unsigned int i=0; i<NumOutputs; i++) {
		if(setRange(x,prevs[i],nexts[i])) {
			OutputCmd& cmd=getKeyFrame((playspeed<0)?nexts[i]:prevs[i]).cmd;
			if(cmd.weight<=0)
				cmd.value=st->outputs[i];
		}
	}
}

void MotionSequenceEngine::setOutputCmd(unsigned int i, const OutputCmd& cmd) {
	Move& p_m=getKeyFrame(prevs[i]);
	if(playtime==p_m.starttime) { //edit current keyframe
		p_m.cmd=cmd;
	} else { //add new keyframe
		Move_idx_t x = newKeyFrame();
		if(x==invalid_move) //ran out of memory - newKeyFrame should report error, we can silently die
			return;
		Move& cur=getKeyFrame(x);
		Move& prev_m=getKeyFrame(prevs[i]); //it's important to refresh this - newKeyFrame may have invalidated p_m reference
		cur.cmd=cmd;
		cur.starttime=playtime;
		cur.prev=prevs[i];
		cur.next=nexts[i];
		prev_m.next=x;
		if(nexts[i]!=invalid_move) //in middle
			getKeyFrame(nexts[i]).prev=x;
		else { //at end
			if(playtime>endtime)
				endtime=playtime;
		}
		prevs[i]=x;
//		cout << "set " << i << ' ' << outputNames[i] << ' ' << cur.cmd.value << ' ' << cur.cmd.weight << ' ' << cur.starttime << " state: " << starts[i] <<' '<< prevs[i] <<' '<< prev_m.next <<' '<< nexts[i] << " new: " << cur.prev << ' ' << x << ' ' << cur.next << endl;
	}
	curstamps[i]=-1U;
}

void MotionSequenceEngine::setPose(const PostureEngine& pose) {
	for(unsigned int i=0; i<NumOutputs; i++)
		if(pose.getOutputCmd(i).weight>0)
			setOutputCmd(i,pose.getOutputCmd(i));
}

void MotionSequenceEngine::setExplicitPose(const PostureEngine& pose) {
	for(unsigned int i=0; i<NumOutputs; i++)
		setOutputCmd(i,pose.getOutputCmd(i));
}

PostureEngine MotionSequenceEngine::getPose() {
	PostureEngine pose;
	getPose(pose);
	return pose;
}

void MotionSequenceEngine::getPose(PostureEngine& pose) {
	for(unsigned int i=0; i<NumOutputs; i++)
		pose.setOutputCmd(i,getOutputCmd(i));
}

unsigned int MotionSequenceEngine::overlayMotion(const std::string& msFile) {
	DynamicMotionSequence ms;
	if(ms.loadFile(msFile.c_str())==0)
		return 0;
	overlayMotion(ms);
	return ms.getEndTime();
}

/*! @todo should better handle conflicts with keyframes in original motion
 *  
 *  This is not a particularly well implemented function -- it will interleave
 *  @a ms's keyframes with any pre-existing ones, but will clobber frames which
 *  coincide with @a ms's own.  Probably will not give the desired effect when
 *  the motion is actually overlaying something, but it will work for basic
 *  usage cases. */
void MotionSequenceEngine::overlayMotion(const MotionSequenceEngine& ms) {
	//merge keyframes for each output, advance playhead by ms.endtime at the end
	for(unsigned int i=0; i<NumOutputs; i++) {
		//cout << "Processing " << outputNames[i] << ":" << endl;
		Move_idx_t myPrevIdx=prevs[i];
		unsigned int myPrevTime=getKeyFrame(prevs[i]).starttime;
		Move_idx_t myNextIdx=getKeyFrame(myPrevIdx).next;
		unsigned int myNextTime=-1U;
		if(myNextIdx!=invalid_move)
			myNextTime=getKeyFrame(myNextIdx).starttime;
		for(Move_idx_t curOtherIdx=ms.starts[i]; curOtherIdx!=invalid_move; curOtherIdx=ms.getKeyFrame(curOtherIdx).next) {
			const Move& curOther=ms.getKeyFrame(curOtherIdx);
			while(myNextTime <= curOther.starttime+getTime()) {
				//cout << "Advancing to " << myNextTime << endl;
				myPrevIdx=myNextIdx;
				myPrevTime=myNextTime;
				myNextIdx=getKeyFrame(myPrevIdx).next;
				myNextTime= (myNextIdx==invalid_move ? -1U : getKeyFrame(myNextIdx).starttime);
			}
			if(curOther.cmd.weight > 0) {
				if(curOther.starttime+getTime() == myPrevTime) {
					//replace current 'prev'
					//cout << "replacing frame " << myPrevIdx << " at " << myPrevTime << endl;
					getKeyFrame(myPrevIdx).cmd=curOther.cmd;
				} else {
					//insert new keyframe between 'prev' and 'next'
					Move_idx_t insIdx=newKeyFrame();
					//cout << "adding new frame at " << curOther.starttime+getTime() << " with index " << insIdx << endl;
					if(insIdx==invalid_move)
						return; //newKeyFrame should have reported error, we can silently return
					Move& ins=getKeyFrame(insIdx);
					ins.prev=myPrevIdx;
					ins.next=myNextIdx;
					ins.cmd=curOther.cmd;
					ins.starttime=curOther.starttime+getTime();
					getKeyFrame(myPrevIdx).next=insIdx;
					if(myNextIdx!=invalid_move)
						getKeyFrame(myNextIdx).prev=insIdx;
					if(myPrevIdx==prevs[i]) {
						nexts[i]=insIdx;
						curstamps[i]=-1U;
					}
					myPrevIdx=insIdx; //inserted frame is now 'prev'
					myPrevTime=ins.starttime;
					if(myPrevTime>endtime)
						endtime=myPrevTime;
				}
			}
		}
	}
	advanceTime(ms.getEndTime());
}

void MotionSequenceEngine::compress() {
	for(unsigned int i=0; i<NumOutputs; i++) {
		Move_idx_t prev=getKeyFrame(starts[i]).next;
		if(prev==invalid_move)
			break;
		Move_idx_t cur=getKeyFrame(prev).next;
		if(cur==invalid_move)
			break;
		Move_idx_t next=getKeyFrame(cur).next;
		while(next!=invalid_move) {
			OutputCmd tmp;
			Move& prev_m=getKeyFrame(prev);
			Move& cur_m=getKeyFrame(cur);
			Move& next_m=getKeyFrame(next);
			calcOutput(tmp,cur_m.starttime,prev_m,next_m);
			if(tmp==cur_m.cmd || (tmp.weight==0 && cur_m.cmd.weight==0) ) {
				prev_m.next=next;
				next_m.prev=prev;
				eraseKeyFrame(cur);
			} else
				prev=cur;
			cur=next;
			next=next_m.next;
		}
	}
}

void MotionSequenceEngine::makeSafe(const float vels[NumOutputs], float margin) {
	float comps[NumOutputs];
	for(unsigned int i=0;i<NumOutputs;i++)
		comps[i]=vels[i]*margin;
	unsigned int t=0;
	Move_idx_t tprevs[NumOutputs];
	Move_idx_t tnexts[NumOutputs];
	for(unsigned int i=0;i<NumOutputs;i++) {
		tnexts[i]=getKeyFrame(tprevs[i]=starts[i]).next;
		curstamps[i]=-1U;
	}
	while(t!=-1U) {
		for(unsigned int i=0; i<NumOutputs; i++) {
			//second and third conditionals are to skip transitions between 0 weighted frames
			if(tnexts[i]!=invalid_move && (getKeyFrame(tprevs[i]).cmd.weight!=0 || getKeyFrame(tnexts[i]).cmd.weight!=0) && getKeyFrame(tprevs[i]).starttime==t) {
				float dv=std::abs(getKeyFrame(tprevs[i]).cmd.value-getKeyFrame(tnexts[i]).cmd.value);
				unsigned int dt=getKeyFrame(tnexts[i]).starttime-getKeyFrame(tprevs[i]).starttime;
				if(dv/dt>comps[i]) {
					unsigned int delay=(unsigned int)(dv/comps[i])-dt;
					for(unsigned int j=0; j<NumOutputs; j++)
						for(Move_idx_t c=tnexts[j]; c!=invalid_move; c=getKeyFrame(c).next)
							getKeyFrame(c).starttime+=delay;
				}
			}
		}
		t=setNextFrameTime(tprevs,tnexts);
	}
	
}	

bool MotionSequenceEngine::isPlaying() {
	return playing && ((playspeed>0) ? (playtime<endtime) : (playtime>0)); 
}


void MotionSequenceEngine::play() {
	if(playspeed>0)
		setTime(0);
	else
		setTime(endtime);
	resume();
}

void MotionSequenceEngine::resume() {
	playing=true;
	lasttime=get_time();
	const WorldState * st=WorldState::getCurrent();
	for(unsigned int i=0; i<NumOutputs; i++) {
		Move_idx_t cur=starts[i];
		while(cur!=invalid_move) {
			if(getKeyFrame(cur).cmd.weight!=0) {
				getKeyFrame(starts[i]).cmd.value=st->outputs[i];
				break;
			}
			cur=getKeyFrame(cur).next;
		}
	}
}

unsigned int MotionSequenceEngine::setNextFrameTime(Move_idx_t p[NumOutputs], Move_idx_t n[NumOutputs]) const {
	unsigned int ans=-1U;
	for(unsigned int i=0; i<NumOutputs; i++)
		if(n[i]!=invalid_move && getKeyFrame(n[i]).starttime<ans)
			ans=getKeyFrame(n[i]).starttime;
	if(ans!=-1U) {
		const WorldState * st=WorldState::getCurrent();
		for(unsigned int i=0; i<NumOutputs; i++) {
			if(setRange(ans,p[i],n[i])) {
				const OutputCmd& cmd=getKeyFrame((playspeed<0)?n[i]:p[i]).cmd;
				if(cmd.weight<=0)
					const_cast<OutputCmd&>(cmd).value=st->outputs[i]; //it's ok to assign the value when weight is '0', still "const"... kinda
			}
		}
	}
	return ans;
}

unsigned int MotionSequenceEngine::readWord(const char buf[], const char * const bufend, char wrd[], const unsigned int wordlen) {
	const char* origbuf=buf;
	wrd[0]='\0';
	unsigned int i;
	//skip whitespace
	for(;buf<bufend && isspace(*buf) && *buf!='\n' && *buf!='\r';buf++) {}
	//store wrd
	for(i=0; buf<bufend && !isspace(*buf); buf++)
		if(i<wordlen-1)
			wrd[i++]=*buf;
	wrd[i]='\0';
	if(buf>=bufend)
		return -1U;
	return buf-origbuf;
}

unsigned int MotionSequenceEngine::getOutputIndex(const char name[], unsigned int idx) {
	unsigned int n=strlen(name);
	while(name[n-1]=='~')
		--n;
	if(idx<NumOutputs) {
		unsigned int startidx=idx;
		for(;idx<NumOutputs;idx++)
			if(strncmp(name,outputNames[idx],n)==0)
				return idx;
		for(idx=0;idx<startidx;idx++)
			if(strncmp(name,outputNames[idx],n)==0)
				return idx;
		return NumOutputs;
	} else {
		for(idx=0;idx<NumOutputs;idx++)
			if(strncmp(name,outputNames[idx],n)==0)
				return idx;
		return idx;
	}
}

/*! @file
 * @brief Implements MotionSequenceEngine, abstract code for smoothly transitioning between a sequence of postures
 * @author ejt (Creator)
 */

