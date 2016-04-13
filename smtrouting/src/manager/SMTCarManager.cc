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
        loadCarFlowFile(carFlowXMLFileName);
        // start car generating process
        genSetpMsg = new cMessage("generate step msg");
        scheduleAt(genPar.startTime, genSetpMsg);
    }
}

void SMTCarManager::handleMessage(cMessage* msg) {
    if (msg == genSetpMsg) {
        handleGenMessage(msg);
    }
    std::cout << "unknown message" << std::endl;
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
    // add generating car flow file process
    double curTime = 0;
    double remain = 0;
    int n = getGenCarNumAtTime(curTime, remain);
    while (n >= 0) {
        addRandomInnerVehicleIntoXML(curTime, n);
        curTime += genPar.generateInterval;
        n = getGenCarNumAtTime(curTime, remain);
    }
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
            carMapByTime.insert(std::make_pair(car->time, car));
        } else {
            std::cout << "duplicate car with name " << car->id << std::endl;
        }
        car = carFlowHelper.getNextCar();
    }
}

void SMTCarManager::addOneVehicle(SMTCarInfo* car) {
    switch (car->type) {
    case SMTCarInfo::SMTCARINFO_ROUTETYPE_OD:
        if (!getComIf()->addVehicle(car->id, car->vtype, car->origin, 0, 0,
                car->maxSpeed, 0)) {
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
        double minL = time - stageStartTime;
        double maxL = genPar.increasePeriod - minL;
        douNum = (minL * genPar.minGenNumPerHour
                + maxL * genPar.maxGenNumPerHour) / genPar.increasePeriod / 3600
                * genPar.generateInterval;
        remain = std::modf(douNum, &result);
        return (int) result;
    }
    stageStartTime += genPar.increasePeriod;
    // at max stage
    if (time < stageStartTime + genPar.maxPeriod) {
        douNum = genPar.maxGenNumPerHour / 3600 * genPar.generateInterval;
        remain = std::modf(douNum, &result);
        return (int) result;
    }
    // at decrease stage
    stageStartTime += genPar.maxPeriod;
    if (time < stageStartTime + genPar.decreasePeriod) {
        double maxL = time - stageStartTime;
        double minL = genPar.decreasePeriod - maxL;
        douNum = (minL * genPar.minGenNumPerHour
                + maxL * genPar.maxGenNumPerHour) / genPar.decreasePeriod / 3600
                * genPar.generateInterval;
        remain = std::modf(douNum, &result);
        return (int) result;
    }
    stageStartTime += genPar.decreasePeriod;
    // at later min stage
    if (time < stageStartTime + genPar.sufPeriod) {
        douNum = genPar.minGenNumPerHour / 3600 * genPar.generateInterval;
        remain = std::modf(douNum, &result);
        return (int) result;
    }
    stageStartTime += genPar.sufPeriod;
    // return -1 if the time is out of range
    return -1;
}

void SMTCarManager::addRandomInnerVehicleIntoXML(double departTime,
        unsigned int num) {
    for (unsigned int i = 0; i < num; ++i) {
        carFlowHelper.addODCar(
                carPrefix + itoa(genPar.lastVechileIndex++, 0, 0),
                getRandomNotOutEdge()->id, getRandomNotEnterEdge()->id,
                departTime, getRandomCarType()->vtype);
    }
}

void SMTCarManager::addRandomThroughVehicleIntoXML(double departTime,
        unsigned int num) {
    for (unsigned int i = 0; i < num; ++i) {
        carFlowHelper.addODCar(
                carPrefix + itoa(genPar.lastVechileIndex++, 0, 0),
                getRandomEnterEdge()->id, getRandomOutEdge()->id, departTime,
                getRandomCarType()->vtype);
    }
}

SMTCarInfo* SMTCarManager::getRandomCarType() {
    if (carInfoVec.size() > 0) {
        return carInfoVec[intrand(carInfoVec.size())];
    } else {
        return NULL;
    }
}

SMTEdge* SMTCarManager::getRandomNotOutEdge() {
    int r = intrand(mapPar.maxInnerIndex + mapPar.maxEnterIndex);
    if (r < mapPar.maxInnerIndex) {
        return getMap()->innerPrimaryEdges[r];
    }
    r -= mapPar.maxInnerIndex;
    if (r < mapPar.maxEnterIndex) {
        return getMap()->enterPrimaryEdges[r];
    }
    return NULL;
}

SMTEdge* SMTCarManager::getRandomNotEnterEdge() {
    int r = intrand(mapPar.maxInnerIndex + mapPar.maxOutIndex);
    if (r < mapPar.maxInnerIndex) {
        return getMap()->innerPrimaryEdges[r];
    }
    r -= mapPar.maxInnerIndex;
    if (r < mapPar.maxOutIndex) {
        return getMap()->outPrimaryEdges[r];
    }
    return NULL;
}

SMTEdge* SMTCarManager::getRandomEnterEdge() {
    int r = intrand(mapPar.maxEnterIndex);
    if (r < mapPar.maxEnterIndex) {
        return getMap()->enterPrimaryEdges[r];
    }
    return NULL;
}

SMTEdge* SMTCarManager::getRandomOutEdge() {
    int r = intrand(mapPar.maxOutIndex);
    if (r < mapPar.maxOutIndex) {
        return getMap()->outPrimaryEdges[r];
    }
    return NULL;
}

SMTEdge* SMTCarManager::getRandomInnerEdge() {
    int r = intrand(mapPar.maxInnerIndex);
    if (r < mapPar.maxInnerIndex) {
        return getMap()->innerPrimaryEdges[r];
    }
    return NULL;
}

void SMTCarManager::handleGenMessage(cMessage* msg) {
    multimap<double, SMTCarInfo*>::iterator it = carMapByTime.begin();
    if (getMap()->getLaunchd()->isConnected()) {
        for (; it != carMapByTime.end() && it->first <= simTime().dbl(); ++it) {
            addOneVehicle(it->second);
            carMapByTime.erase(it);
            it = carMapByTime.begin();
        }
    }
    if (it != carMapByTime.end()) {
        scheduleAt(it->first, msg);
    } else {
        cancelAndDelete(msg);
    }
}

void SMTCarManager::releaseCarMap() {
    for (map<string, SMTCarInfo*>::iterator it = carMapByID.begin();
            it != carMapByID.end(); ++it) {
        delete (it->second);
    }
    carMapByID.clear();
    carMapByTime.clear();
}
