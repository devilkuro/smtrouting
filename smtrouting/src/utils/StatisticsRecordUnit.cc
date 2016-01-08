/*
 * GlobalStaticsUnit.cc
 *
 *  Created on: 1405051354
 *      Author: Fanjing
 */

#include "StatisticsRecordUnit.h"
#include <sstream>

namespace Fanjing {
StatisticsRecordUnit::StatisticsRecordUnit(int size) {
    this->size = size;
    data = new DataUnit[size];
}

StatisticsRecordUnit::~StatisticsRecordUnit() {
    size = 0;
    if (data!=NULL) {
        delete[] data;
        data = NULL;
    }
}

void StatisticsRecordUnit::setData(double data, int index) {
    if(index >= 0 && index < size){
        this->data[index].type = UNIT_TYPE_DOUBLE;
        this->data[index].douData = data;
    }
}

int StatisticsRecordUnit::getSize() const {
    return size;
}

void StatisticsRecordUnit::setData(int data, int index) {
    if(index >= 0 && index < size){
        this->data[index].type = UNIT_TYPE_INT;
        this->data[index].intData = data;
    }
}

void StatisticsRecordUnit::setData(uint32_t data, int index) {
    if(index >= 0 && index < size){
        this->data[index].type = UNIT_TYPE_UINT32;
        this->data[index].uint32Data = data;
    }
}

void StatisticsRecordUnit::setData(uint64_t data, int index) {
    if(index >= 0 && index < size){
        this->data[index].type = UNIT_TYPE_UINT64;
        this->data[index].uint64Data = data;
    }
}

void StatisticsRecordUnit::setData(string data, int index) {
    if(index >= 0 && index < size){
        this->data[index].type = UNIT_TYPE_STRING;
        this->data[index].strData = data;
    }
}

int StatisticsRecordUnit::getDataType(int index) {
    if(index >= 0 && index < size){
        return data[index].type;
    }else{
        return UNIT_TYPE_ERROR;
    }
}

string StatisticsRecordUnit::toString() {
    stringstream ss;
    for(int i = 0; i < size; i++){
        if(i != 0)
            ss << "\t";
        ss << data[i].toString();
    }
    return ss.str();
}

string StatisticsRecordUnit::DataUnit::toString() {
    stringstream ss;
    switch(type){
        case UNIT_TYPE_INT:
            ss << intData;
            break;
        case UNIT_TYPE_UINT32:
            ss << uint32Data;
            break;
        case UNIT_TYPE_UINT64:
            ss << uint64Data;
            break;
        case UNIT_TYPE_DOUBLE:
            ss << douData;
            break;
        case UNIT_TYPE_STRING:
            ss << strData;
            break;
        default:
            break;
    }
    string str = ss.str();
    ss.str( std::string() );
    ss.clear();
    return str;
}
}  // namespace Fanjing

