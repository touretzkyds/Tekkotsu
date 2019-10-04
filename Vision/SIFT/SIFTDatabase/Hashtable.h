#ifndef __HASHTABLE_H
#define __HASHTABLE_H

#include <vector>
#include <iostream>

const size_t primes[30] = {2,5,11,23,47,97,197,397,797,1597,3203,6421,12853,25717,51437,102877,205759,411527,823117,1646237,3292489,6584983,13169977,26339969,52679969,105359939,210719881,421439783,842879579,1685759167};
const int numPrimes = 30;

template <class data, class key, size_t (*hashFunc)(key* K), bool (*equals)(key* k1, key* k2)>
class Hashtable{
private:
	int  primeIndex;
	size_t sizeOfTable;
	std::vector<bool>*  isUsed;
	std::vector<key*>*  keys;
	std::vector<data*>* datas;
	size_t numElems;
	size_t numUsedBins;
	
	size_t find(key* k){
		size_t hashVal = (hashFunc(k) % sizeOfTable);
		
		size_t pos = hashVal;
		int i = 1;
		while (1){
			//				if (pos >= (int)isUsed->size()){
			//					cout << "ERROR! " << pos << " " << isUsed->size() << " " << sizeOfTable << endl;
			//				}
			//				if (pos < 0){
			//					cout << "ERROR! " << hashFunc(k) << " " << hashVal << " " << pos << " " << isUsed->size() << " " << sizeOfTable << endl;
			//					HoughKey* hk = (HoughKey*)k;
			//					cout << hk->scale << " " << hk->x << " " << hk->y << " " << hk->orientation << endl;
			//				}
			if (!((*isUsed)[pos])){
				return pos;
			}else if ((*keys)[pos] != NULL && equals((*keys)[pos], k)){
				return pos;
			}
			pos = (hashVal + (i*i)) % sizeOfTable;
			//				cout << pos << endl;
			i++;
		}
		
		return pos;
	}
	
	void resizeTable(){
		using namespace std;
		
		cout << "RESIZING!\n";
		
		vector<bool>*  oldIsUsed = isUsed;
		vector<key*>*  oldKeys   = keys;
		vector<data*>* oldDatas  = datas;
		size_t oldSizeOfTable = sizeOfTable;
		
		numElems = 0;
		numUsedBins = 0;
		
		// Double table size
		// Assumes that hashtable never gets bigger than 1685759167
		primeIndex++;
		sizeOfTable = primes[primeIndex];
		
		isUsed = new vector<bool>(sizeOfTable, false);
		keys   = new vector<key*>(sizeOfTable, NULL);
		datas  = new vector<data*>(sizeOfTable, NULL);
		
		for (size_t i = 0; i < oldSizeOfTable; i++){
			if ((*oldIsUsed)[i] && ((*oldKeys)[i] != NULL)){
				size_t index = find((*oldKeys)[i]);
				(*isUsed)[index] = true;
				numElems++;
				numUsedBins++;
				(*keys)[index] = (*oldKeys)[i];
				(*datas)[index] = (*oldDatas)[i];
			}
		}
		
		delete oldKeys;
		delete oldIsUsed;
		delete oldDatas;
	}
	
public:
	Hashtable() : 
    primeIndex(10), sizeOfTable(primes[primeIndex]),
    isUsed(new std::vector<bool>(sizeOfTable, false)),
    keys(new std::vector<key*>(sizeOfTable, NULL)),
    datas(new std::vector<data*>(sizeOfTable, NULL)),
    numElems(0), numUsedBins(0) {}
	
	~Hashtable(){
		delete keys;
		delete isUsed;
		delete datas;
	}
	
	data* retrieve(key* k){
		size_t index = find(k);
		if (!(*isUsed)[index])
			return NULL;
		return (*datas)[index];
	}
	
	void retrieveAllData(std::vector<data*>* returnVec){
		returnVec->clear();
		
		for (size_t i = 0; i < sizeOfTable; i++){
			if ((*isUsed)[i] && (*keys)[i] != NULL){
				returnVec->push_back((*datas)[i]);
			}
		}
	}
	
	void retrieveAllKeys(std::vector<key*>* returnVec){
		returnVec->clear();
		
		for (size_t i = 0; i < sizeOfTable; i++){
			if ((*isUsed)[i] && (*keys)[i] != NULL){
				returnVec->push_back((*keys)[i]);
			}
		}
	}
	
	bool insert(data* d, key* k){
		size_t index = find(k);
		
		if ((*isUsed)[index]) return false;
		
		// Check if need to resize
		if (numUsedBins+1 >= sizeOfTable / 2){
			resizeTable();
			index = find(k);
		}
		
		numElems++;
		numUsedBins++;
		(*isUsed)[index] = true;
		(*keys)[index] = k;
		(*datas)[index] = d;
		
		return true;
	}
	
	data* deleteData(key* k, key** retrievedKey){
		size_t index = find(k);
		
		if (!(*isUsed)[index]) return NULL;
		
		*retrievedKey = (*keys)[index];
		data* returnVal = (*datas)[index];
		
		(*keys)[index] = NULL;
		(*datas)[index] = NULL;
		
		numElems--;
		
		return returnVal;
	}
	
	size_t numElements(){
		return numElems;
	}
	
	size_t usedCapacity(){
		return numUsedBins;
	}
	
	size_t capacity(){
		return sizeOfTable;
	}
	
private:
	Hashtable(const Hashtable&); // Do not use
	Hashtable& operator=(const Hashtable&); // Do not use
	
};

#endif
