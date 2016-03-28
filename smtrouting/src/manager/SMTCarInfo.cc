/*
 * SMTCarInfo.cc
 *
 *  Created on: Oct 7, 2015
 *      Author: Fanjing-LAB
 */

#include "SMTCarInfo.h"

namespace Fanjing {
string SMTCarInfo::path = "";
XMLDocument* SMTCarInfo::doc = NULL;
map<string, SMTCarInfo> SMTCarInfo::vTypeMap;
SMTCarInfo::SMTCarInfo() {
    id = "";
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
    // FIXME 静态变量doc需要clear().
}

void SMTCarInfo::loadVehicleTypeXML(string path) {
    if (doc != NULL) {
        vTypeMap.clear();
    } else {
        doc = new XMLDocument();
    }
    XMLError e = doc->LoadFile(path.c_str());
    if (e == XML_SUCCESS) {
        XMLElement* root = doc->FirstChildElement("routes");
        if (root != NULL) {
            XMLElement* e = root->FirstChildElement("vType");
            SMTCarInfo car;
            while (e) {
                if (NULL != e->Attribute("id")) {
                    car.vtype = e->Attribute("id");
                    car.accel = e->DoubleAttribute("accel");
                    car.decel = e->DoubleAttribute("decel");
                    car.sigma = e->DoubleAttribute("sigma");
                    car.length = e->DoubleAttribute("length");
                    car.minGap = e->DoubleAttribute("minGap");
                    car.maxSpeed = e->DoubleAttribute("maxSpeed");
                    car.color = e->Attribute("color");
                    vTypeMap[e->Attribute("id")] = car;
                }
                e = e->NextSiblingElement("vType");
            }
        }
    }
    doc->Clear();
}

SMTCarInfo SMTCarInfo::getDefaultVeicleTypeInfo(string vTypeId) {
    if (vTypeMap.find(vTypeId) != vTypeMap.end()) {
        return vTypeMap[vTypeId];
    }
    return SMTCarInfo();
}

bool SMTCarInfo::hasInitialized() {
    // FIXME this is a simple method
    return vTypeMap.size() > 0;
}

void SMTCarInfo::release() {
    if (doc != NULL) {
        doc->Clear();
        doc = NULL;
    }
}

list<string> SMTCarInfo::getDefaultVeicleTypeList() {
    list<string> result;
    for (map<string, SMTCarInfo>::iterator it = vTypeMap.begin();
            it != vTypeMap.end(); it++) {
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
    str += "time :" + StringHelper::dbl2str(time) + ", ";
    str += "vtype :" + vtype + ", ";
    str += "accel :" + StringHelper::dbl2str(accel) + ", ";
    str += "decel :" + StringHelper::dbl2str(decel) + ", ";
    str += "sigma :" + StringHelper::dbl2str(sigma) + ", ";
    str += "minGap :" + StringHelper::dbl2str(minGap) + ", ";
    str += "maxSpeed :" + StringHelper::dbl2str(maxSpeed) + ", ";
    str += "color :" + color + ", ";
    return str;
}

} /* namespace Fanjing */
