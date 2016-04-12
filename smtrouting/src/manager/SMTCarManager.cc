//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "SMTCarManager.h"
#include <cmath>

Define_Module(SMTCarManager);

SMTCarManager::~SMTCarManager() {
    releaseCarMap();
}

int SMTCarManager::numInitStages() const {
    return 2;
}

void SMTCarManager::initialize(int stage) {
    if (stage == 0) {
        // set configuration.
        debug = par("debug").boolValue();
        endAfterGenerateCarFlowFile =
                par("endAfterGenerateCarFlowFile").boolValue();
        carPrefix = par("carPrefix").stringValue();
        XMLPrefix = par("XMLPrefix").stringValue();
        rouXMLFileName = par("rouXMLFileName").stringValue();
        if (rouXMLFileName == "") {
            rouXMLFileName = XMLPrefix + ".rou.xml";
        }
        carFlowXMLFileName = par("carFlowXMLFileName").stringValue();
        if (carFlowXMLFileName == "") {
            carFlowXMLFileName = XMLPrefix + "_default" + ".cf.xml";
        }
        genPar.minGenNumPerHour = par("minGenNumPerHour").doubleValue();
        genPar.maxGenNumPerHour = par("maxGenNumPerHour").doubleValue();
        genPar.startTime = par("startTime").doubleValue();
        genPar.prePeriod = par("prePeriod").doubleValue();
        genPar.increasePeriod = par("increasePeriod").doubleValue();
        genPar.maxPeriod = par("maxPeriod").doubleValue();
        genPar.decreasePeriod = par("decreasePeriod").doubleValue();
        genPar.sufPeriod = par("sufPeriod").doubleValue();
        genPar.generateInterval = par("generateInterval").doubleValue();

        SMTCarInfo::loadVehicleTypeXML(rouXMLFileName);
        carInfoVec = SMTCarInfo::getDefaultVehicleTypeVector();
    }
    if (stage == 1) {
        // set map and car info
        mapPar.maxInnerIndex = getMap()->innerPrimaryEdges.size();
        mapPar.maxEnterIndex = getMap()->enterPrimaryEdges.size();
        mapPar.maxOutIndex = getMap()->outPrimaryEdges.size();
    }
}

void SMTCarManager::handleMessage(cMessage* msg) {
}

void SMTCarManager::finish() {
}

SMTMap* SMTCarManager::getMap() {
    if (_pMap == NULL) {
        _pMap = SMTMapAccess().get();
    }
    ASSERT2(_pMap!=NULL, "can not find SMTMap.");
    return _pMap;
}

SMTComInterface* SMTCarManager::getComIf() {
    if (_pComIf == NULL) {
        _pComIf = getMap()->getLaunchd()->getSMTComInterface();
    }
    ASSERT2(_pMap!=NULL, "can not find SMTComInterface.");
    return _pComIf;
}

void SMTCarManager::generateCarFlowFile(const string& path) {
    // TODO add generating process

    if (endAfterGenerateCarFlowFile) {
        endSimulation();
    }
}

void SMTCarManager::loadCarFlowFile(const string& path) {
    string xmlpath;
    releaseCarMap();
    if (path != "") {
        xmlpath = path;
    } else {
        xmlpath = carFlowXMLFileName;
    }
    if ((carFlowHelper.loadXML(xmlpath) & 1) != 0) {
        generateCarFlowFile(xmlpath);
        carFlowHelper.loadXML(xmlpath);
    }
    SMTCarInfo* car = carFlowHelper.getFirstCar();
    while (car != NULL) {
        if (carMapByID.find(car->id) == carMapByID.end()) {
            carMapByID[car->id] = car;
        } else {
            std::cout << "duplicate car with name " << car->id << std::endl;
        }
        car = carFlowHelper.getNextCar();
    }
}

void SMTCarManager::addOneVehicle(SMTCarInfo* car) {
    switch (car->type) {
    case SMTCarInfo::SMTCARINFO_ROUTETYPE_OD:
        if (!getComIf()->addVehicle(car->id, car->vtype, car->origin, car->time,
                0, car->maxSpeed, 0)) {
            if (debug) {
                cout << "add car failed: car id: " << car->id << ", road: "
                        << car->origin << ", @" << car->time << endl;
            }
        }
        break;
    case SMTCarInfo::SMTCARINFO_ROUTETYPE_LOOP:
        // can not handle this type now.
        break;
    default:
        break;
    }
}

int SMTCarManager::getGenCarNumAtTime(double time, double &remain) {
    double result = 0;
    double stageStartTime = 0;
    double douNum = 0;
    // before start time
    if (time < stageStartTime + genPar.startTime) {
        douNum = genPar.minGenNumPerHour / 3600 * genPar.generateInterval;
        remain += douNum;
        return (int) result;
    }
    stageStartTime += genPar.startTime;
    // at previous min stage
    if (time < stageStartTime + genPar.prePeriod) {
        douNum = genPar.minGenNumPerHour / 3600 * genPar.generateInterval;
        remain = std::modf(douNum, &result);
        return (int) result;
    }
    stageStartTime += genPar.prePeriod;
    // at increase stage
    if (time < stageStartTime + genPar.increasePeriod) {
        // TODO
        douNum = genPar.minGenNumPerHour / 3600 * genPar.generateInterval;
        remain = std::modf(douNum, &result);
        return (int) result;
    }
    stageStartTime += genPar.increasePeriod;
}

void SMTCarManager::releaseCarMap() {
    for (map<string, SMTCarInfo*>::iterator it = carMapByID.begin();
            it != carMapByID.end(); ++it) {
        delete (it->second);
    }
    carMapByID.clear();
    carMapByTime.clear();
}
