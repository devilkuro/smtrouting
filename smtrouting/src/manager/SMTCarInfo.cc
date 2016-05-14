/*
 * SMTCarInfo.cc
 *
 *  Created on: Oct 7, 2015
 *      Author: Fanjing-LAB
 */

#include "SMTCarInfo.h"
#include <iostream>

string SMTCarInfo::path = "";
XMLDocument* SMTCarInfo::doc = NULL;
map<string, SMTCarInfo*> SMTCarInfo::defaultVTypeMap;
SMTCarInfo::SMTCarInfo() {
    id = "";
    mobility = NULL;
    inTeleport = false;
    index = 0;
    isMajorType = false;
    type = SMTCARINFO_ROUTETYPE_LAST_TYPE;
    origin = "";
    destination = "";
    loop = "";
    time = -1;
    string vtype = "";

    // type related
    accel = -1;
    decel = -1;
    sigma = -1;
    length = -1;
    minGap = -1;
    maxSpeed = -1;
    color = "";   // useless
}

SMTCarInfo::~SMTCarInfo() {
}

void SMTCarInfo::loadVehicleTypeXML(const string &path) {
    if (doc != NULL) {
        defaultVTypeMap.clear();
    } else {
        doc = new XMLDocument();
    }
    XMLError e = doc->LoadFile(path.c_str());
    if (e == XML_SUCCESS) {
        XMLElement* root = doc->FirstChildElement("routes");
        if (root != NULL) {
            XMLElement* e = root->FirstChildElement("vType");
            SMTCarInfo* car = new SMTCarInfo();
            while (e) {
                if (NULL != e->Attribute("id")) {
                    car->vtype = e->Attribute("id");
                    car->accel = e->DoubleAttribute("accel");
                    car->decel = e->DoubleAttribute("decel");
                    car->sigma = e->DoubleAttribute("sigma");
                    car->length = e->DoubleAttribute("length");
                    car->minGap = e->DoubleAttribute("minGap");
                    car->maxSpeed = e->DoubleAttribute("maxSpeed");
                    car->color = e->Attribute("color");
                    defaultVTypeMap[e->Attribute("id")] = car;
                }
                e = e->NextSiblingElement("vType");
            }
        }
    }
    doc->Clear();
}

SMTCarInfo* SMTCarInfo::getDefaultVehicleTypeInfo(const string &vTypeId) {
    if (defaultVTypeMap.find(vTypeId) != defaultVTypeMap.end()) {
        return defaultVTypeMap[vTypeId];
    }
    cout << "no vehicle type named " << vTypeId << endl;
    return NULL;
}

bool SMTCarInfo::hasInitialized() {
    // FIXME this is a simple method
    return defaultVTypeMap.size() > 0;
}

void SMTCarInfo::release() {
    // FIXME 静态变量doc需要clear().
    if (doc != NULL) {
        doc->Clear();
        doc = NULL;
    }
    for (map<string, SMTCarInfo*>::iterator it = defaultVTypeMap.begin();
            it != defaultVTypeMap.end(); ++it) {
        delete (it->second);
    }
}

list<string> SMTCarInfo::getDefaultVehicleTypeList() {
    list<string> result;
    for (map<string, SMTCarInfo*>::iterator it = defaultVTypeMap.begin();
            it != defaultVTypeMap.end(); it++) {
        result.push_back(it->first);
    }
    return result;
}
string SMTCarInfo::toString() {
    string str = "";
    str += "car id :" + id + ", ";
    str += "type :"
            + (string) (
                    type == SMTCARINFO_ROUTETYPE_OD ?
                            "SMTCARINFO_ROUTETYPE_OD" :
                            (type == SMTCARINFO_ROUTETYPE_LOOP ?
                                    "SMTCARINFO_ROUTETYPE_LOOP" : "null"))
            + ", ";
    str += "origin :" + origin + ", ";
    str += "destination :" + destination + ", ";
    str += "loop :" + loop + ", ";
    str += "time :" + Fanjing::StringHelper::dbl2str(time) + ", ";
    str += "vtype :" + vtype + ", ";
    str += "accel :" + Fanjing::StringHelper::dbl2str(accel) + ", ";
    str += "decel :" + Fanjing::StringHelper::dbl2str(decel) + ", ";
    str += "sigma :" + Fanjing::StringHelper::dbl2str(sigma) + ", ";
    str += "minGap :" + Fanjing::StringHelper::dbl2str(minGap) + ", ";
    str += "maxSpeed :" + Fanjing::StringHelper::dbl2str(maxSpeed) + ", ";
    str += "color :" + color + ", ";
    return str;
}

vector<SMTCarInfo*> SMTCarInfo::getDefaultVehicleTypeVector() {
    vector<SMTCarInfo*> result;
    for (map<string, SMTCarInfo*>::iterator it = defaultVTypeMap.begin();
            it != defaultVTypeMap.end(); it++) {
        result.push_back(it->second);
    }
    return result;
}
