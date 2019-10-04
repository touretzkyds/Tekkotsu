#include "DataSource.h"
#include "Shared/StackTrace.h"
#include "IPC/RCRegion.h"
#include "Shared/get_time.h"


bool DataSource::requiresFirstSensor = true;
const plist::Primitive<float>* DataSource::sensorFramerate=NULL;
SensorState * DataSource::sensorState=NULL;


SensorState::SensorState() : timestamp(0), frameNumber(0), dirty(false), motionOverride(NULL), /*updateSignal(),*/ resourceSync(NULL), lock() {
	for(unsigned int i=0; i<NumOutputs; ++i) {
		providedOutputs[i]=0;
		outputs[i]=0;
	}
	for(unsigned int i=0; i<NumButtons; ++i)
		buttons[i]=0;
	for(unsigned int i=0; i<NumSensors; ++i)
		sensors[i]=0;
	for(unsigned int i=0; i<NumPIDJoints; ++i)
		pids[i][0]=pids[i][1]=pids[i][2] = pidduties[i] = 0;
}

void SensorState::releaseResource(Data& d) {
	Thread::NoCancelScope nc;
	if(!dirty) {
		static_cast<Resource&>(lock).releaseResource(d);
	} else {
		//++frameNumber;  // this is done in Simulator::sendSensor() instead, see DataSource::SensorState::frameNumber docs
		timestamp=get_time();
		if(resourceSync!=NULL)
			resourceSync();
		static_cast<Resource&>(lock).releaseResource(d);
		//updateSignal.broadcast(); 
	}
}


DataSource::~DataSource() {
	std::for_each(regions.begin(),regions.end(),std::mem_fun(&RCRegion::RemoveReference));
	regions.clear();
}

void DataSource::providingOutput(unsigned int i) {
	if(i>=NumOutputs) {
		ASSERT(i==-1U,"DataSource is trying to provide a bad output " << i);
		return;
	}
	++sensorState->providedOutputs[i];
	if(sensorState->providedOutputs[i]>1)
		std::cerr << "WARNING: multiple (" << sensorState->providedOutputs[i] <<") data sources are claiming to provide feedback for " << outputNames[i] << std::endl;
	//std::cout << "PROVIDING " << i << " (" << outputNames[i] << ") now " << sensorState->providedOutputs[i] << std::endl;
}

void DataSource::ignoringOutput(unsigned int i) {
	if(i>=NumOutputs) {
		ASSERT(i==-1U,"DataSource is trying to ignore a bad output " << i);
		return;
	}
	if(sensorState->providedOutputs[i]==0) {
		std::cerr << "ERROR: DataSource output tracking underflow" << std::endl;
		stacktrace::displayCurrentStackTrace();
		return;
	}
	--sensorState->providedOutputs[i];
	if(sensorState->providedOutputs[i]==1)
		std::cerr << "NOTICE: feedback conflict for " << outputNames[i] << " has been resolved." << std::endl;
	//std::cout << "IGNORING " << i << " (" << outputNames[i] << ") now " << sensorState->providedOutputs[i] << std::endl;
}

void DataSource::setImage(const ImageHeader& header, const void * data) {
	const size_t datasize = header.width*header.height*header.components;
	RCRegion * r = getUnusedRegion(sizeof(header) + datasize,0);
	memcpy(r->Base(),&header,sizeof(header));
	memcpy(r->Base()+sizeof(header), data, datasize);
	setImage(r);
}

RCRegion* DataSource::getUnusedRegion(size_t minSize, size_t padding) {
	// find first 'unused' region
	RCRegion* region=NULL;
	for(std::list<RCRegion*>::iterator it=regions.begin();it!=regions.end(); ++it) {
		if((*it)->NumberOfReference()==1) {
			region=*it;
			regions.erase(it);
			break;
		}
	}
	
	if(region==NULL) { // if we didn't find one create one now
		region = new RCRegion(minSize+padding);
		//std::cout << "DataSource::getUnusedRegion() created " << region->ID().key << " size " << region->Size() << std::endl;
	}
	else if(region->Size() < minSize) {
		//too small -- free it and make a bigger one
		//std::cout << "DataSource::getUnusedRegion() released " << region->ID().key << " size " << region->Size() << std::endl;
		region->RemoveReference(); 
		region = new RCRegion(minSize+padding);
		//std::cout << "DataSource::getUnusedRegion() created " << region->ID().key << " size " << region->Size() << std::endl;
	}
	regions.push_back(region);
	return region;
}


/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
