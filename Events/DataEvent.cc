#include "DataEvent.h"

template<> void DataEvent<unsigned char, -1>::getDataFromString(std::stringstream &ss) { ss >> data; }
template<> void DataEvent<unsigned short int, -1>::getDataFromString(std::stringstream &ss) { ss >> data; }
template<> void DataEvent<unsigned int, -1>::getDataFromString(std::stringstream &ss) { ss >> data; }
template<> void DataEvent<int, -1>::getDataFromString(std::stringstream &ss) { ss >> data; }
template<> void DataEvent<float, -1>::getDataFromString(std::stringstream &ss) { ss >> data; }
template<> void DataEvent<double, -1>::getDataFromString(std::stringstream &ss) { ss >> data; }

template<> void DataEvent<unsigned char, -1>::sendDataToString(std::stringstream &ss) const { ss << data; }
template<> void DataEvent<unsigned short int, -1>::sendDataToString(std::stringstream &ss) const { ss << data; }
template<> void DataEvent<unsigned int, -1>::sendDataToString(std::stringstream &ss) const { ss << data; }
template<> void DataEvent<int, -1>::sendDataToString(std::stringstream &ss) const { ss << data; }
template<> void DataEvent<float, -1>::sendDataToString(std::stringstream &ss) const { ss << data; }
template<> void DataEvent<double, -1>::sendDataToString(std::stringstream &ss) const { ss << data; }

