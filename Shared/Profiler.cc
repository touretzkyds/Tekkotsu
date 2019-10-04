#include "Profiler.h"
#include "debuget.h"
#include <cmath>

const float Profiler::HistCurve = 4.05f;

float Profiler::buckets[Profiler::HistSize];

unsigned int Profiler::infosOffset=((size_t)(&static_cast<ProfilerOfSize<1>*>(NULL)[1].infos))-((size_t)(&static_cast<ProfilerOfSize<1>*>(NULL)[1].prof));

ProfilerOfSize<20> * mainProfiler=NULL;
ProfilerOfSize<06> * motionProfiler=NULL;
ProfilerOfSize<06> * soundProfiler=NULL;

/*! Tells the profiler that this is now the active timer, so new timers will fit "under" this.\n
 *  Timer isn't actually started here, lets Profiler::setCurrent do that.
 *  @param prof profiler to report results to.  If is NULL, does nothing.
 *  @param id id number for this function.  See Profiler::getNewID() for what you should pass this */
Profiler::Timer::Timer(unsigned int id, Profiler* prof) : _prof(prof), _id(id), _parent(-1U), _t(0L) {
	if(_prof!=NULL)
		_prof->setCurrent(*this);
}

Profiler::Timer::~Timer() {
	if(_prof!=NULL)
		_prof->finished(*this);
}

void Profiler::Timer::setID(unsigned int id, Profiler* prof) {
	_id=id;
	_prof=prof;
	if(_prof!=NULL)
		_prof->setCurrent(*this);
}	

Profiler::SectionInfo::SectionInfo()
	: totalTime(0L),lastTime(0L),totalInterval(0L),childTime(0L),execExpAvg(0),interExpAvg(0),calls(0)
{
	name[0]='\0';
	for(unsigned int j=0; j<HistSize; j++)
		execHist[j]=interHist[j]=0;
}

void Profiler::SectionInfo::reset() {
	totalTime.Set(0L);
	lastTime.Set();
	totalInterval.Set(0L);
	childTime.Set(0L);
	execExpAvg=0;
	interExpAvg=0;
	for(unsigned int j=0; j<HistSize; j++)
		execHist[j]=interHist[j]=0;
	calls=0;
}

void Profiler::initBuckets() {
	float g=HistTime/std::pow(HistSize,HistCurve)/1000;
	for(unsigned int i=1; i<=HistSize; i++)
		buckets[i-1]=g*std::pow(i,HistCurve);
}

unsigned int Profiler::getNewID(const char* name) {
	ASSERTRETVAL(sectionsUsed<maxSections,"Too many sections registered (increase ProfilerOfSize<MaxSections>)",-1U);
	SectionInfo * infos=getInfos();
#ifdef DEBUG
	for(unsigned int i=0; i<sectionsUsed; i++)
		ASSERTRETVAL(strncmp(infos[i].name,name,MaxSectionNameLen-1)!=0,"Already using name " << name,-1U);
#endif
	unsigned int id=sectionsUsed++;
	strncpy(infos[id].name,name,MaxSectionNameLen-1);
	infos[id].name[MaxSectionNameLen-1]='\0'; //guarantees null-termination
	return id;
}

std::string Profiler::report() {
	char tmp[255];
	SectionInfo * infos=getInfos();
	sprintf(tmp,"Profiling information since: %f to %f\n",startTime.Value(),TimeET().Value());
	std::string ans(tmp);
	for(unsigned int i=0; i<sectionsUsed; i++) {
		sprintf(tmp,"%s:\n",infos[i].name); ans+=tmp;
		unsigned int calls=infos[i].calls;
		sprintf(tmp,"\t%d calls\n",calls); ans+=tmp;
		if(calls>0) {
			sprintf(tmp,"\t%f ms avg\n",infos[i].totalTime.Value()/calls*1000); ans+=tmp;
			sprintf(tmp,"\t%f ms exp.avg\n",infos[i].execExpAvg*1000); ans+=tmp;
			sprintf(tmp,"\t%f ms avg child time (%f%%)\n",infos[i].childTime.Value()/calls*1000,((int)(infos[i].childTime.Value()/infos[i].totalTime.Value()*1000))/10.0); ans+=tmp;
			sprintf(tmp,"\t%f ms avg inter (%f fps)\n",infos[i].totalInterval.Value()/calls*1000,calls/infos[i].totalInterval.Value()); ans+=tmp;
			sprintf(tmp,"\t%f ms exp.avg (%f fps)\n",infos[i].interExpAvg*1000,1/infos[i].interExpAvg); ans+=tmp;
			ans+="\tExec: ";
			for(unsigned int j=0; j<HistSize; j++) {
				sprintf(tmp,"%d ",infos[i].execHist[j]);
				ans+=tmp;
			}
			ans+="\n\tInter: ";
			for(unsigned int j=0; j<HistSize; j++) {
				sprintf(tmp,"%d ",infos[i].interHist[j]);
				ans+=tmp;
			}
			ans+="\n";
		}
	}
	ans+="Bucket distribution (in ms):\n\t0";
	for(unsigned int j=0; j<HistSize; j++) {
		sprintf(tmp,"<%.3g, ",buckets[j]*1000);
		ans+=tmp;
	}
	ans+="\n";
	return ans;
}

void Profiler::reset() {
	SectionInfo * infos=getInfos();
	for(unsigned int i=0; i<sectionsUsed; i++)
		infos[i].reset();
	startTime.Set();
}

Profiler::Profiler(unsigned int mx)
	: curSection(-1U), startTime(), gamma(.85f), maxSections(mx), sectionsUsed(0), autoInit()
{ }

unsigned int Profiler::AutoInit::refcount=0;
unsigned int Profiler::AutoInit::totalcount=0;

Profiler::AutoInit::AutoInit() {
	if(totalcount==0)
		Profiler::initBuckets();
	totalcount++;
	refcount++;
}

Profiler::AutoInit::~AutoInit() { refcount--; }

void Profiler::setCurrent(Timer& tr) {
	SectionInfo& info=getInfos()[tr._id];
	tr._parent=curSection;
	curSection=tr._id;
	info.calls++;
	if(info.calls>1) {
		TimeET diff=info.lastTime.Age();
		info.totalInterval+=diff;
		if(info.calls==2)
			info.interExpAvg=(float)diff.Value();
		else
			info.interExpAvg=info.interExpAvg*gamma+(float)diff.Value()*(1-gamma);
		if(diff.Value()*1000>HistTime)
			info.interHist[HistSize-1]++;
		else
			info.interHist[getBucket((float)diff.Value())]++;
	}
	tr._t.Set(); //do last so profiling code won't throw off timing results (but makes childtime's a little bloated)
	info.lastTime=tr._t;
}

void Profiler::finished(Timer& tr) {
	TimeET diff=tr.elapsed(); //do immediately for accuracy
	SectionInfo& info=getInfos()[tr._id];
	info.totalTime+=diff;
	if(tr._parent!=-1U)
		getInfos()[tr._parent].childTime+=diff;
	//	ASSERT(info.calls!=0,"calls is 0 on finished");
	if(info.calls==1)
		info.execExpAvg=(float)diff.Value();
	else
		info.execExpAvg=info.execExpAvg*gamma+(float)diff.Value()*(1-gamma);
	if(diff.Value()*1000>HistTime)
		info.execHist[HistSize-1]++;
	else
		info.execHist[getBucket((float)diff.Value())]++;
	//the old way:
	//info.execHist[mathutils::log2t((unsigned int)(((1U<<31)/HistTime*2)*diff.Value()))]++;
	curSection=tr._parent;
}

/*! @file
 * @brief Implements Profiler, which managers a hierarchy of timers for profiling time spent in code
 * @author ejt (Creator)
 */
