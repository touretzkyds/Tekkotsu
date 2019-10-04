#include "dynamixel_util.h"
#include "IPC/FailsafeThread.h"
#include <list>
#include <fstream>
#include <iterator>
#include <numeric>
#include <cmath>

using namespace std;

int cmd_perf(std::istream& is, std::ostream& os, bool perfSync, const std::string& perfSaveFile, const std::vector<std::pair<servoid,servoid> >& perfRanges);

class PerfTestThread : public Thread {
public:
	PerfTestThread(istream& in, ostream& out, bool syncwrite, const vector<pair<servoid,servoid> >& idranges)
		: Thread(), cycleTimes(), writeReadTimes(), readTimes(), writeTimes(),
		is(in), os(out), sync(syncwrite), ranges(idranges), nservos(0), curServo(),
		testTime(), setupTime(), 
		failsafe(*this,DynamixelProtocol::PingThread::getTimeout()*1000,false)
	{ failsafe.restartFlag=true; start(); }
	virtual ~PerfTestThread(); //!< stop and join on destruction (otherwise our custom cancelled() won't be called!)
	virtual void* run();
	list<double> cycleTimes, writeReadTimes, readTimes, writeTimes;
protected:
	virtual void cancelled();
	template<class T> bool read(servoid sid, T& resp);
	template<class T> bool write(const T& cmd);
	static void stats(const list<double>& d);
	istream& is;
	ostream& os;
	bool sync;
	const vector<pair<servoid,servoid> >& ranges;
	servoid nservos, curServo;
	TimeET testTime, setupTime;
	FailsafeThread failsafe;
}; 

//! this version does the parsing, then calls the other version for execution
int cmd_perf(std::istream& is, std::ostream& os, int argc, const char* argv[]) {
	bool perfSync=false;
	string perfSaveFile;
	vector<pair<servoid,servoid> > perfRanges;
	try {
		int i=0;
		if(++i>argc-1) {
			cerr << "ERROR: '" << argv[0] << "' requires an id or range" << endl;
			return 2;
		}
		if(strcasecmp("sync",argv[i])==0) {
			perfSync=true;
			if(++i>argc-1) {
				cerr << "ERROR: '" << argv[0] << "' requires an id or range" << endl;
				return 2;
			}
		}
		if(strcasecmp("save",argv[i])==0) {
			if(++i>argc-1) {
				cerr << "ERROR: '" << argv[0] << "' save requires an output file name" << endl;
				return 2;
			}
			perfSaveFile=argv[i];
			if(++i>argc-1) {
				cerr << "ERROR: '" << argv[0] << "' requires an id or range" << endl;
				return 2;
			}
		}
		i += readRanges(argc-i,&argv[i],perfRanges,argv[0]);
	} catch(...) {
		return 2;
	}
	return cmd_perf(is, os, perfSync, perfSaveFile, perfRanges);
}


//! this version does the actual execution of the command
int cmd_perf(std::istream& is, std::ostream& os, bool perfSync, const std::string& perfSaveFile, const std::vector<std::pair<servoid,servoid> >& perfRanges) {
	cout << "Gathering statistics..." << endl;
	cout << "Press <enter> or ctrl-D to end test" << endl;
	PerfTestThread perfTest(is, os, perfSync, perfRanges);
	string s;
	getline(cin,s);
	if(perfSaveFile.size()>0) {
		perfTest.stop().join();
		cout << "Saving data to '" << perfSaveFile << "'..." << flush;
		ofstream dout(perfSaveFile.c_str());
		if(!dout) {
			cerr << "ERROR: could not open '" << perfSaveFile << "' for writing." << endl;
			return 1;
		}
		dout << "CYCLE_END_TIMES\t";
		copy(perfTest.cycleTimes.begin(),perfTest.cycleTimes.end(),ostream_iterator<double>(dout,"\t"));
		dout.seekp(-1,ios_base::cur); // overwrite trailing tab character with newline
		dout << "\nREAD_LATENCIES\t";
		copy(perfTest.readTimes.begin(),perfTest.readTimes.end(),ostream_iterator<double>(dout,"\t"));
		dout.seekp(-1,ios_base::cur); // overwrite trailing tab character with newline
		dout << "\nWRITE_LATENCIES\t";
		copy(perfTest.writeTimes.begin(),perfTest.writeTimes.end(),ostream_iterator<double>(dout,"\t"));
		dout.seekp(-1,ios_base::cur); // overwrite trailing tab character with newline
		dout << endl;
		cout << "done!" << endl;
	}
	return EXIT_SUCCESS;
}

PerfTestThread::~PerfTestThread() {
	if(started) {
		stop();
		join();
	}
}

void* PerfTestThread::run() {
	is.sync();
	if(failsafe.isStarted())
		failsafe.stop();
	write(DynamixelProtocol::SetStatusResponseLevelCmd(DynamixelProtocol::RESPOND_ALL));
	map<servoid,int> positions;
	for(vector<pair<servoid,servoid> >::const_iterator it=ranges.begin(); it!=ranges.end(); ++it) {
		using namespace DynamixelProtocol;
		ServoSensorsResponse servoinfo;
		for(servoid sid=it->first; sid<=it->second && os; ++sid) {
			PingThread ping(is,os,sid,-1U,&servoinfo);
			void * model = ping.join();
			testCancel();
			if(model==NULL || model==Thread::CANCELLED) {
				cerr << "Warning: Servo " << (int)sid << " not responding" << endl;
			} else {
				ModelID_t modelID = static_cast<ModelID_t>(ping.response.getModelNumber());
				if(modelID==MODEL_AXS1 || dynamixelModels.find(modelID)==dynamixelModels.end()) {
					cerr << "Warning: Servo " << (int)sid << " is type " << ping.response.getModelString() << " (need AX, RX, or EX servo)" << endl;
				} else {
					positions[sid]=servoinfo.getPosition();
				}
			}
		}
	}
	nservos = positions.size();
	if(nservos==0) {
		cerr << "ERROR: no valid servos were found for performance testing" << endl;
		return NULL;
	}
	testTime+=setupTime.Age();
	if(failsafe.isStarted())
		failsafe.join();
	failsafe.start();
	string progress="-\\|/";
	string::size_type cnt=0;
	while(os) {
		for(map<servoid,int>::iterator it=positions.begin(); it!=positions.end(); ++it) {
			failsafe.progressFlag=true;
			TimeET readTime;
			if(!write(DynamixelProtocol::ReadServoSensorsCmd(it->first)))
				return NULL;
			writeReadTimes.push_back(readTime.Age().Value()*1000);
			DynamixelProtocol::ServoSensorsResponse resp;
			if(!read(it->first, resp))
				return NULL;
			readTimes.push_back(readTime.Age().Value()*1000);
			it->second = resp.getPosition();
			if(!sync) {
				TimeET writeTime;
				write(DynamixelProtocol::SetPosSpeedCmd(it->first, it->second, 0));
				DynamixelProtocol::WriteResponse wrResp;
				if(!read(it->first, wrResp))
					return NULL;
				writeTimes.push_back(writeTime.Age().Value()*1000);
			}
		}
		if(sync) {
			TimeET writeTime;
			DynamixelProtocol::SyncWriteHeader<DynamixelProtocol::SyncWritePosSpeedEntry> header(positions.size());
			unsigned char checksum=DynamixelProtocol::nchecksum(header,sizeof(header));
			os.write(header,sizeof(header));
			for(map<servoid,int>::const_iterator it=positions.begin(); it!=positions.end(); ++it) {
				DynamixelProtocol::SyncWritePosSpeedEntry entry(it->first,it->second,0);
				os.write(entry,sizeof(entry));
				checksum+=DynamixelProtocol::nchecksum(entry,sizeof(entry));
			}
			os.put(~checksum).flush();
			writeTimes.push_back(writeTime.Age().Value()*1000);
		}
		cycleTimes.push_back(testTime.Age().Value()*1000);
		cout << progress[cnt] << '\r' << flush;
		cnt = (cnt+1)%progress.size();
		testCancel();
	}
	return NULL;
}

void PerfTestThread::cancelled() {
	TimeET stopTime = testTime.Age();
	if(failsafe.isEngaged()) {
		cerr << "No response from servo " << curServo << ", resetting test..." << endl;
		cerr << "Results so far: " << endl;
	} else if(failsafe.isStarted()) {
		failsafe.stop();
	}
	if(nservos==0)
		return;
	cout.precision(2);
	cout << fixed;
	cout << "=== " << (int)nservos << " servos, tested for " << stopTime.Value() << " seconds, "
		<< (sync ? "using sync_write commands" : "using per-servo writes") << " ===" << endl;
	cout << readTimes.size() << '/' << writeReadTimes.size() << " reads completed" << endl;
	cout << writeTimes.size() << " writes completed" << endl;
	cout << cycleTimes.size() << " complete cycles" << endl;
	cout << setw(23) << "Read cmd time (ms):  "; stats(writeReadTimes);
	cout << setw(23) << "Read time (ms):  "; stats(readTimes);
	cout << setw(23) << "Write time (ms):  "; stats(writeTimes);
	cout << "Per servo: " << cycleTimes.size()/cycleTimes.back()*1000*nservos << "Hz (" << cycleTimes.back()/cycleTimes.size()/nservos << " ms/cycle period)" << endl;
	cout << "Per cycle: " << cycleTimes.size()/cycleTimes.back()*1000 << "Hz (" << cycleTimes.back()/cycleTimes.size() << " ms/cycle period)" << endl;
}

template<class T> bool PerfTestThread::read(servoid sid, T& resp) {
	curServo=sid;
	setupTime.Set();
	is.read(resp,sizeof(resp));
	if(!is || is.gcount()!=sizeof(resp)) {
		testCancel();
		cerr << "Short read from " << sid << ", resetting test" << endl;
		return false;
	}
	if(!DynamixelProtocol::validate(resp)) {
		cerr << "Bad checksum from " << sid << ", resetting test" << endl;
		return false;
	}
	return true;
}

template<class T> bool PerfTestThread::write(const T& cmd) {
	os.write(cmd,sizeof(cmd)).flush();
	if(!os)
		return false;
	return true;
}

void PerfTestThread::stats(const list<double>& d) {
	vector<double> nth(d.begin(), d.end());
	nth_element(nth.begin(), nth.begin()+nth.size()/2, nth.end());
	double median = (nth.size()==0) ? 0 : *(nth.begin()+nth.size()/2);
	double sum = accumulate(nth.begin(), nth.end(), 0.0);
	double avg = (nth.size()==0) ? 0 : sum / nth.size();
	double var = 0;
	for(vector<double>::const_iterator it=nth.begin(); it!=nth.end(); ++it) {
		double x = avg - *it;
		var += x*x;
	}
	var/=nth.size();
	double stdev = std::sqrt(var);
	const int w = cout.precision()+3;
	cout << "Median " << setw(w) << median << "  Avg " << setw(w) << avg << "  StDev " << setw(w) << stdev << endl;
}

static bool registered = registerCommand("perf",cmd_perf);
