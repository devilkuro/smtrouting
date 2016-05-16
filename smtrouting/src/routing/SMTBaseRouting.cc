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

#include "SMTBaseRouting.h"
#include "StringHelper.h"
#include <cmath>
#include "SMTMobility.h"

Define_Module(SMTBaseRouting);

bool SMTBaseRouting::WeightLane::debug = false;
double SMTBaseRouting::WeightLane::outCarKeepDuration = 120;
double SMTBaseRouting::WeightLane::limitStart = 0.1;
double SMTBaseRouting::WeightLane::limitCap = 0.7;
double SMTBaseRouting::WeightLane::limitFix = 0.01;
double SMTBaseRouting::WeightLane::airK = 0.2287;
double SMTBaseRouting::WeightLane::airV = 0.6352;
bool SMTBaseRouting::WeightLane::minAllowedCostFix = false;
bool SMTBaseRouting::WeightLane::minRecentCostFix = false;
unsigned int SMTBaseRouting::WeightLane::operationNum = 0;
multimap<double, SMTBaseRouting::CoRPUpdateBlock*> *SMTBaseRouting::WeightLane::corpUpdateQueue =
NULL;
multimap<double, SMTBaseRouting::CoRPUpdateBlock*> *SMTBaseRouting::WeightLane::corpAddQueue =
NULL;
multimap<double, SMTBaseRouting::CoRPUpdateBlock*> *SMTBaseRouting::WeightLane::corpRemoveQueue =
NULL;

SMTBaseRouting::~SMTBaseRouting() {
    // 回收 dijkstra's algorithm 算法部分
    for (map<SMTEdge*, WeightEdge*>::iterator it = weightEdgeMap.begin();
            it != weightEdgeMap.end(); ++it) {
        delete (it->second);
    }
    for (map<SMTCarInfo*, WeightRoute*>::iterator it = hisRouteMapByCar.begin();
            it != hisRouteMapByCar.end(); ++it) {
        delete (it->second);
    }
}

int SMTBaseRouting::numInitStages() const {
    return 3;
}

void SMTBaseRouting::initialize(int stage) {
    if (stage == 0) {
        debug = par("debug").boolValue();
        WeightLane::debug = debug;
        suppressLength = par("suppressLength").doubleValue();
        WeightLane::limitStart = par("limitStart").doubleValue();
        WeightLane::limitFix = par("limitFix").doubleValue();
        WeightLane::limitCap = par("limitCap").doubleValue();
        WeightLane::airK = par("airK").doubleValue();
        WeightLane::airV = par("airV").doubleValue();
        WeightLane::corpUpdateQueue = &corpUpdateQueue;
        WeightLane::corpAddQueue = &corpAddQueue;
        WeightLane::corpRemoveQueue = &corpRemoveQueue;

        majorRoutingType =
                (enum SMT_ROUTING_TYPE) par("majorRoutingType").longValue();
        minorRoutingType =
                (enum SMT_ROUTING_TYPE) par("minorRoutingType").longValue();
        recordHisRecordRoutingType =
                par("recordHisRecordRoutingType").longValue();
        if (majorRoutingType == SMT_RT_AIR || minorRoutingType == SMT_RT_AIR) {
            enableAIR = true;
        } else {
            enableAIR = false;
        }
        corpUseHisRouteCEC = par("corpUseHisRouteCEC").longValue();
        corpReRouteCEC = par("corpReRouteCEC").longValue();
        // whether use fast with occupancy replace air routing
        replaceAIRWithITSWithOccupancy =
                par("replaceAIRWithITSWithOccupancy").boolValue();
        // if replace air, disable it
        if (replaceAIRWithITSWithOccupancy) {
            enableAIR = false;
        }
        if (majorRoutingType == SMT_RT_CORP_SELF
                || minorRoutingType == SMT_RT_CORP_SELF
                || majorRoutingType == SMT_RT_CORP_TTS
                || minorRoutingType == SMT_RT_CORP_TTS) {
            enableCoRP = true;
        } else {
            enableCoRP = false;
        }
        if (enableCoRP) {
            enableCoRPReroute = par("enableCoRPReroute").boolValue();
        }
        enableCoRPPreImport = par("enableCoRPPreImport").boolValue();
        srt = Fanjing::StatisticsRecordTools::request();
        rouStatus.recordActiveCarNum = par("recordActiveCarNum").boolValue();
        rouStatus.recordActiveCarInterval =
                par("recordActiveCarInterval").doubleValue();
        recordXMLPrefix = par("recordXMLPrefix").stringValue();
        recordHisRoutingData = par("recordHisRoutingData").boolValue();
        if (majorRoutingType == recordHisRecordRoutingType) {
            recordHisRoutingData = true;
            recordHisRoutingResult = true;
        }
        enableHisDataRecord = par("enableHisDataRecord").boolValue();
        hisRecordXMLPath = par("hisRecordXMLPath").stringValue();
        endAfterLoadHisXML = par("endAfterLoadHisXML").boolValue();

        SMTBaseRouting::WeightLane::minAllowedCostFix =
                par("minAllowedCostFix").boolValue();
        SMTBaseRouting::WeightLane::minRecentCostFix =
                par("minRecentCostFix").boolValue();
    }
    if (stage == 1) {
        // needs to init weightEdgeMap here
        // to make sure Map has initialized before
        // set numInitStages=2 and init weightEdgeMap at stage 1
        // set weightEdgeMap for dijkstra's algorithm
        for (set<SMTEdge*>::iterator it = getMap()->primaryEdgeSet.begin();
                it != getMap()->primaryEdgeSet.end(); ++it) {
            weightEdgeMap[*it] = new WeightEdge(*it);
        }
        for (map<SMTEdge*, WeightEdge*>::iterator it = weightEdgeMap.begin();
                it != weightEdgeMap.end(); ++it) {
            for (map<SMTEdge*, vector<SMTVia*> >::iterator vIt =
                    it->first->viaVecMap.begin();
                    vIt != it->first->viaVecMap.end(); ++vIt) {
                WeightLane* wLane = new WeightLane();
                // initialize via length
                // FIXME may need support multiple link
                wLane->via = vIt->second[0];
                wLane->con =
                        it->first->laneVector[wLane->via->fromLane]->conVector[0];
                wLane->viaLen = vIt->second[0]->getViaLength();
                // initialize occupation and occStep information
                wLane->occStep = 1 / it->first->length();
                if (wLane->con->tr == 0) {
                    wLane->corpEta = 2.0;
                    wLane->corpCalcEta = 2.0;
                } else {
                    wLane->corpEta = 1.3;
                    wLane->corpCalcEta = 1.3;
                }
                wLane->corpOta = wLane->viaLen / 30;
                wLane->from = it->second;
                wLane->to = weightEdgeMap[vIt->first];
                if (debug) {
                    std::cout << "viaLane from " << wLane->from->edge->id
                            << " to " << wLane->to->edge->id << "- Len:"
                            << wLane->viaLen << ", viaOta:" << wLane->corpOta
                            << std::endl;
                }
                wLane->initMinAllowedCost();
                if (it->second->w2NextMap.find(vIt->first)
                        == it->second->w2NextMap.end()) {
                    it->second->w2NextMap[vIt->first] = wLane;
                } else {
                    std::cout << "system do not support multiple link for now"
                            << std::endl;
                }
            }
        }
        if (debug) {
            debugMsg = new cMessage();
            scheduleAt(simTime() + 120, debugMsg);
        }
        if (enableAIR) {
            airUpdateMsg = new cMessage("air update");
            scheduleAt(simTime() + 1.0, airUpdateMsg);
        }
        if (enableCoRP) {
            corpUpdateMsg = new cMessage("corp update");
            scheduleAt(simTime() + 3.0, corpUpdateMsg);
        }
        statisticMsg = new cMessage("statisticMsg)");
        scheduleAt(simTime() + rouStatus.recordActiveCarInterval, statisticMsg);
    }
    if (stage == 2) {
        // load historical xml after car map has been initialized
        if (enableCoRP) {
            importHisXML();
            if (endAfterLoadHisXML) {
                endSimMsg = new cMessage("end simulation by routing)");
                scheduleAt(simTime(), endSimMsg);
            }
        }
    }
}

void SMTBaseRouting::handleMessage(cMessage* msg) {
    // only handle self message
    if (msg->isSelfMessage()) {
        if (msg == debugMsg) {
            if (debug) {
                scheduleAt(simTime() + 120, debugMsg);
                printStatisticInfo();
            }
        } else if (msg == corpUpdateMsg) {
            updateCoRPInfo();
            scheduleAt(simTime() + 3, corpUpdateMsg);
        } else if (msg == airUpdateMsg) {
            updateAIRInfo();
        } else if (msg == statisticMsg) {
            scheduleAt(simTime() + rouStatus.recordActiveCarInterval,
                    statisticMsg);
            updateStatisticInfo();
        } else if (msg == endSimMsg) {
            cancelAndDelete(msg);
            endSimMsg = NULL;
            endSimulation();
        }
    } else {
        cSimpleModule::handleMessage(msg);
    }
}

void SMTBaseRouting::finish() {
    if (debug) {
        printStatisticInfo();
    }
    srt->outputSeparate(recordXMLPrefix + ".txt");
    exportHisXML();
}

SMTMap* SMTBaseRouting::getMap() {
    if (_pMap == NULL) {
        _pMap = SMTMapAccess().get();
    }
    return _pMap;
}

SMTCarManager* SMTBaseRouting::getCarManager() {
    if (_pCarManager == NULL) {
        _pCarManager = SMTCarManagerAccess().get();
    }
    return _pCarManager;
}

void SMTBaseRouting::getShortestRoute(SMTEdge* origin, SMTEdge* destination,
        list<SMTEdge*> &rou, double time, SMTCarInfo* car) {
    // 最短路径使用迪杰斯特拉算法
    routeType = SMT_RT_SHOREST;
    startTime = time;
    carInfo = car;
    rou.clear();
    runDijkstraAlgorithm(origin, destination, rou);
}

void SMTBaseRouting::initDijkstra(SMTEdge* origin) {
    // 1. reset weightEdge
    map<SMTEdge*, WeightEdge*>::iterator ori_it = weightEdgeMap.find(origin);
    if (ori_it == weightEdgeMap.end()) {
        std::cout << "No edge " << origin->id << " in weightEdgeMap"
                << std::endl;
    }
    for (map<SMTEdge*, WeightEdge*>::iterator it = weightEdgeMap.begin();
            it != weightEdgeMap.end(); ++it) {
        it->second->previous = NULL;
        it->second->w = -1;
        it->second->t = -1;
    }
    // 2. init unSet, outSet, processMap
    // unSet = weightEdgeMap;
    // outSet.clear();
    processMap.clear();
    // insert origin edge into processMap
    ori_it->second->w = origin->length();
    ori_it->second->t = startTime;
    processMap.insert(std::make_pair(ori_it->second->w, ori_it->second));
}

void SMTBaseRouting::changeDijkstraWeight(WeightEdge* from, WeightEdge* to,
        double w, double t) {
    if (to->w != -1 && to->w <= w) {
        // do not change wEdge if the old weight is smaller
        return;
    }
    if (to->w > w) {
        // remove wEdge from processMap
        for (multimap<double, WeightEdge*>::iterator it = processMap.find(
                to->w); it->first == to->w; ++it) {
            if (it->second == to) {
                processMap.erase(it);
                break;
            }
        }
    }
    if (to->w == -1) {
        // if wEdge->w == -1, it is an untouched edge.
    }
    to->previous = from;
    to->w = w;
    to->t = t;
    processMap.insert(std::make_pair(w, to));
}

int SMTBaseRouting::processDijkstraLoop(SMTEdge* destination) {
    SMTEdge* curEdge;
    do {
        curEdge = processDijkstralNode(destination);
    } while (curEdge != destination && curEdge != NULL);
    if (curEdge == NULL) {
        std::cout << "dead end for car: " << carInfo->id << std::endl;
        return -1;
    }
    return 0;
}

void SMTBaseRouting::runDijkstraAlgorithm(SMTEdge* origin, SMTEdge* destination,
        list<SMTEdge*> &route) {
    // before operation
    if (enableCoRP && enableCoRPPreImport && route.size() == 0) {
        // remove and update CoRP info
        // route.size == 0 means it routing first time
        removeCoRPCar(hisRouteMapByCar[carInfo]);
    }
    if (enableCoRP && enableCoRPReroute && route.size() > 1) {
        // 构造需要移除的路径信息
        WeightRoute* rou = new WeightRoute();
        for (list<SMTEdge*>::iterator it = route.begin(); it != route.end();
                ++it) {
            rou->edges.push_back(weightEdgeMap[(*it)]);
        }
        list<WeightEdge*>::iterator itR = rou->edges.begin();
        WeightEdge* start = *itR;
        ++itR;
        WeightEdge* next = *itR;
        WeightLane* lane = start->w2NextMap[next->edge];
        rou->car = carInfo;
        rou->t = lane->corpCarMap[carInfo]->enterTime;
        removeCoRPCar(rou);
        delete rou;
    }
    initDijkstra(origin);
    processDijkstraLoop(destination);
    getDijkstralResult(destination, route);
}

SMTEdge* SMTBaseRouting::processDijkstralNode(SMTEdge* destination) {
    // 1. get WeightEdge with smallest w
    // A->2. if wEdge->edge is destination
    //    3. return wEdge->edge
    // B->2. if not, change its neighbors
    //    3. add neighbors to processMap
    //    4. move it to outSet
    //    5. return wEdge->edge
    // C->2. if processMap is empty, it is dead end.
    //    3. return NULL
    if (processMap.size() == 0) {
        std::cout << "dead end" << std::endl;
        return NULL;
    }
    WeightEdge* wEdge = processMap.begin()->second;
    if (wEdge->edge == destination) {
        return destination;
    } else {
        // remove wEdge out of processMap
        // since it will be out edge after process its neighbors
        // and it must be moved out of processMap
        // or it will be an infinite loop.
        processMap.erase(processMap.begin());
        processDijkstralNeighbors(wEdge);
        return wEdge->edge;
    }
}

void SMTBaseRouting::processDijkstralNeighbors(WeightEdge* wEdge) {
    // try to change w of neighbors to wEdge->w + wEdge->edge->length + viaLen
    for (map<SMTEdge*, vector<SMTVia*> >::iterator it =
            wEdge->edge->viaVecMap.begin(); it != wEdge->edge->viaVecMap.end();
            ++it) {
        WeightEdge* next = weightEdgeMap[it->first];
        if (next->w == getSmallerOne(next->w, wEdge->w)) {
            // if next->w is smaller than wEdge->w
            // the next edge is out edge, which means it has been processed already
            continue;
        }
        modifyWeightFromEdgeToEdge(wEdge, next);
    }

}

double SMTBaseRouting::getSmallerOne(double a, double b) {
    if (a < 0) {
        return b;
    } else if (b < 0) {
        return -1;
    } else if (a > b) {
        return b;
    } else {
        return a;
    }
}

void SMTBaseRouting::getFastestRoute(SMTEdge* origin, SMTEdge* destination,
        list<SMTEdge*>& rou, double time, SMTCarInfo* car) {
    // 最短路径使用迪杰斯特拉算法
    routeType = SMT_RT_FAST;
    startTime = time;
    carInfo = car;
    rou.clear();
    runDijkstraAlgorithm(origin, destination, rou);
}

void SMTBaseRouting::getAIRRoute(SMTEdge* origin, SMTEdge* destination,
        list<SMTEdge*>& rou, double time, SMTCarInfo* car) {
    routeType = SMT_RT_AIR;
    startTime = time;
    carInfo = car;
    rou.clear();
    runDijkstraAlgorithm(origin, destination, rou);
}

void SMTBaseRouting::getCORPSelfRoute(SMTEdge* origin, SMTEdge* destination,
        list<SMTEdge*>& rou, double time, SMTCarInfo* car) {
    routeType = SMT_RT_CORP_SELF;
    startTime = time;
    carInfo = car;
    rou.clear();
    runDijkstraAlgorithm(origin, destination, rou);
}

void SMTBaseRouting::getCORPTTSRoute(SMTEdge* origin, SMTEdge* destination,
        list<SMTEdge*>& rou, double time, SMTCarInfo* car) {
    routeType = SMT_RT_CORP_TTS;
    startTime = time;
    carInfo = car;
    rou.clear();
    runDijkstraAlgorithm(origin, destination, rou);
}

void SMTBaseRouting::getDYRPRoute(SMTEdge* origin, SMTEdge* destination,
        list<SMTEdge*>& rou, double time, SMTCarInfo* car) {
    routeType = SMT_RT_DYRP;
    startTime = time;
    carInfo = car;
    rou.clear();
    runDijkstraAlgorithm(origin, destination, rou);
}

bool SMTBaseRouting::getRouteByMajorMethod(SMTEdge* origin,
        SMTEdge* destination, list<SMTEdge*>& rou, double time,
        SMTCarInfo* car) {
    switch (majorRoutingType) {
    case SMT_RT_USEOLDROUTE:
        getOldRoute(origin, destination, rou, time, car);
        break;
    case SMT_RT_FAST:
        getFastestRoute(origin, destination, rou, time, car);
        break;
    case SMT_RT_AIR:
        getAIRRoute(origin, destination, rou, time, car);
        break;
    case SMT_RT_CORP_SELF:
        getCORPSelfRoute(origin, destination, rou, time, car);
        break;
    case SMT_RT_CORP_TTS:
        getCORPTTSRoute(origin, destination, rou, time, car);
        break;
    case SMT_RT_DYRP:
        getDYRPRoute(origin, destination, rou, time, car);
        break;
    default:
        getFastestRoute(origin, destination, rou, time, car);
        break;
    }
    if (majorRoutingType == SMT_RT_DYRP || enableCoRPReroute) {
        return true;
    }
    return false;
}

bool SMTBaseRouting::getRouteByMinorMethod(SMTEdge* origin,
        SMTEdge* destination, list<SMTEdge*>& rou, double time,
        SMTCarInfo* car) {
    switch (minorRoutingType) {
    case SMT_RT_USEOLDROUTE:
        getOldRoute(origin, destination, rou, time, car);
        break;
    case SMT_RT_FAST:
        getFastestRoute(origin, destination, rou, time, car);
        break;
    case SMT_RT_AIR:
        getAIRRoute(origin, destination, rou, time, car);
        break;
    case SMT_RT_CORP_SELF:
        getCORPSelfRoute(origin, destination, rou, time, car);
        break;
    case SMT_RT_CORP_TTS:
        getCORPTTSRoute(origin, destination, rou, time, car);
        break;
    case SMT_RT_DYRP:
        getDYRPRoute(origin, destination, rou, time, car);
        break;
    default:
        getFastestRoute(origin, destination, rou, time, car);
        break;
    }
    if (minorRoutingType == SMT_RT_DYRP || enableCoRPReroute) {
        return true;
    }
    return false;
}

void SMTBaseRouting::printStatisticInfo() {
    for (map<SMTEdge*, WeightEdge*>::iterator itWE = weightEdgeMap.begin();
            itWE != weightEdgeMap.end(); ++itWE) {
        for (map<SMTEdge*, WeightLane*>::iterator itWL =
                itWE->second->w2NextMap.begin();
                itWL != itWE->second->w2NextMap.end(); ++itWL) {
            if (itWL->second->status.passedCarNum > 0) {
                std::cout << "via from edge " << itWE->first->id << " to "
                        << itWL->first->id << std::endl;
                std::cout << "Avg speed: "
                        << itWL->second->viaLen
                                * itWL->second->status.passedCarNum
                                / itWL->second->status.totalViaPassTime
                        << ", Max Speed: "
                        << itWL->second->viaLen
                                / itWL->second->status.minViaPassTime
                        << ", Min Speed: "
                        << itWL->second->viaLen
                                / itWL->second->status.maxViaPassTime
                        << ", Max Lane Speed: "
                        << itWL->second->from->edge->length()
                                / itWL->second->status.minLanePassTime
                        << ", car number: " << itWL->second->status.passedCarNum
                        << std::endl;
            }
        }
    }
    std::cout << "arrivedCarCount:" << rouStatus.arrivedCarCount << std::endl;
    std::cout << "TTS" << getMap()->getLaunchd()->getTTS() << std::endl;
}

void SMTBaseRouting::updateAIRInfo() {
    for (map<SMTEdge*, WeightEdge*>::iterator itWE = weightEdgeMap.begin();
            itWE != weightEdgeMap.end(); ++itWE) {
        for (map<SMTEdge*, WeightLane*>::iterator itWL =
                itWE->second->w2NextMap.begin();
                itWL != itWE->second->w2NextMap.end(); ++itWL) {
            itWL->second->updateAIRsi();
        }
    }
}

void SMTBaseRouting::updateStatisticInfo() {
    double curTime = simTime().dbl();
    static string titleTime = "time";
    static string titleActiveCarCount = titleTime + "\t" + "carCount";
    static string titleArrivedCarCount = titleTime + "\t" + "arrivedCarCount";
    static string titleTTS = titleTime + "\t" + "TTS";
    static string titleMainCarTTS = titleTime + "\t" + "mainCarTTS";
    static string titleCO2Emission = titleTime + "\t" + "CO2Emission";
    static string titleMainCO2Emission = titleTime + "\t" + "mainCO2Emission";
    static string titleDistance = titleTime + "\t" + "Distance";
    static string titleMainDistance = titleTime + "\t" + "mainDistance";
    static string titleOperationNum = titleTime + "\t" + "operationNum";
    if (rouStatus.recordActiveCarNum) {
        double curCO2Emission = rouStatus.totalCO2EmissionForArrivedCars;
        double curMainCO2Emission =
                rouStatus.totalCO2EmissionForMajorArrivedCars;
        double curDis = rouStatus.totalDistanceForArrivedCars;
        double curMainDis =
                rouStatus.totalDistanceForMajorArrivedCars;
        double curTTS = rouStatus.TTS;
        double curMainTTS = rouStatus.mainCarTTS;
        for (map<string, SMTCarInfo*>::iterator it =
                getCarManager()->carMapByID.begin();
                it != getCarManager()->carMapByID.end(); ++it) {
            if (it->second->mobility != NULL || it->second->inTeleport) {
                curTTS += curTime - it->second->time;
                if (it->second->mobility != NULL) {
                    curCO2Emission +=
                            it->second->mobility->getStatistics()->totalCO2Emission;
                    curDis +=
                            it->second->mobility->getStatistics()->totalDistance;
                }
                if (it->second->isMajorType) {
                    curMainTTS += curTime - it->second->time;
                    if (it->second->mobility != NULL) {
                        curMainCO2Emission +=
                                it->second->mobility->getStatistics()->totalCO2Emission;
                        curMainDis +=
                                it->second->mobility->getStatistics()->totalDistance;
                    }
                }
            }
        }

        srt->changeName("activeCarCount", titleActiveCarCount) << curTime
                << getMap()->getLaunchd()->getActiveVehicleCount() << srt->endl;
        srt->changeName("TTS", titleTTS) << curTime << curTTS << srt->endl;
        srt->changeName("arrivedCarCount", titleArrivedCarCount) << curTime
                << rouStatus.arrivedCarCount << srt->endl;
        srt->changeName("mainCarTTS", titleMainCarTTS) << curTime << curMainTTS
                << srt->endl;
        srt->changeName("CO2Emission", titleCO2Emission) << curTime
                << curCO2Emission << srt->endl;
        srt->changeName("mainCO2Emission", titleMainCO2Emission) << curTime
                << curMainCO2Emission << srt->endl;
        srt->changeName("distance", titleDistance) << curTime
                << curDis << srt->endl;
        srt->changeName("mainDistance", titleMainDistance) << curTime
                << curMainDis << srt->endl;
        srt->changeName("operationNum", titleOperationNum) << curTime
                << WeightLane::operationNum << srt->endl;
    }

    std::cout << "updateCoRPCar:" << WeightLane::operationNum << std::endl;
    if (getCarManager()->carMapByTime.size() == 0) {
        // all cars are deployed
        static unsigned int activedCarSinceLastStatistics = 0;
        // 如果所有车辆都已部署且360秒内车辆数目不变,则认为存在环路阻塞,终止试验
        static int stackTimes = 0;
        if (getMap()->getLaunchd()->getActiveVehicleCount()
                == activedCarSinceLastStatistics) {
            stackTimes++;
        } else {
            activedCarSinceLastStatistics =
                    getMap()->getLaunchd()->getActiveVehicleCount();
            stackTimes = 0;
        }
        if (stackTimes >= 5
                || getMap()->getLaunchd()->getActiveVehicleCount() == 0) {
            if (debug) {
                std::cout << "getActiveVehicleCount: "
                        << getMap()->getLaunchd()->getActiveVehicleCount()
                        << "carMapByTime.size(): "
                        << getCarManager()->carMapByTime.size() << std::endl;
            }
            if (endSimMsg == NULL) {
                srt->outputSeparate(recordXMLPrefix + ".txt");
                endSimMsg = new cMessage("end simulation by routing)");
                scheduleAt(simTime() + 0.1, endSimMsg);
            }
        }
    }
}

void SMTBaseRouting::exportHisXML() {
    if (recordHisRoutingData) {
        XMLDocument* doc = new XMLDocument();
        XMLDeclaration* dec = doc->NewDeclaration();
        doc->LinkEndChild(dec);
        XMLComment* comment = doc->NewComment(
                "et=enterTime;lt=laneTime;vt=viaTime;"
                        "it=intervalToLast;opt=outPrimaryEdgeTime");
        doc->LinkEndChild(comment);
        XMLElement* fromEdgeElm;
        XMLElement* toEdgeElm;
        XMLElement* carElm;
        WeightEdge* fromWEdge = NULL;
        WeightLane* wLane = NULL;
        for (map<SMTEdge*, WeightEdge*>::iterator itWE = weightEdgeMap.begin();
                itWE != weightEdgeMap.end(); ++itWE) {
            fromEdgeElm = doc->NewElement("From");
            fromWEdge = itWE->second;
            fromEdgeElm->SetAttribute("id", fromWEdge->edge->id.c_str());
            for (map<SMTEdge*, WeightLane*>::iterator itWL =
                    fromWEdge->w2NextMap.begin();
                    itWL != fromWEdge->w2NextMap.end(); ++itWL) {
                toEdgeElm = doc->NewElement("To");
                wLane = itWL->second;
                toEdgeElm->SetAttribute("id", wLane->to->edge->id.c_str());
                SMTConnection* con = wLane->con;
                toEdgeElm->SetAttribute("t0", con->t0);
                toEdgeElm->SetAttribute("tg", con->tg);
                toEdgeElm->SetAttribute("ty", con->ty);
                toEdgeElm->SetAttribute("tr", con->tr);
                for (map<double, HisInfo*>::iterator itHis =
                        wLane->hisTimeMap.begin();
                        itHis != wLane->hisTimeMap.end(); ++itHis) {
                    carElm = doc->NewElement("CAR");
                    carElm->SetAttribute("id", itHis->second->car->id.c_str());
                    carElm->SetAttribute("et",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->enterTime, 1).c_str());
                    if (itHis->second->next != NULL) {
                        carElm->SetAttribute("next",
                                itHis->second->next->to->edge->id.c_str());
                    } else {
                        carElm->SetAttribute("next", "");
                    }
                    carElm->SetAttribute("lt",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->laneTime, 1).c_str());
                    carElm->SetAttribute("vt",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->viaTime, 1).c_str());
                    carElm->SetAttribute("it",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->intervalToLast, 1).c_str());
                    carElm->SetAttribute("opt",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->enterTime
                                            + itHis->second->laneTime, 1).c_str());
                    toEdgeElm->LinkEndChild(carElm);
                }
                fromEdgeElm->LinkEndChild(toEdgeElm);
            }
            doc->LinkEndChild(fromEdgeElm);
        }
        doc->SaveFile((hisRecordXMLPath + ".lane.xml").c_str());
        doc->Clear();
        delete doc;
    }
    if (recordHisRoutingResult) {
        hisRouteDoc = new XMLDocument();
        XMLDeclaration* dec = hisRouteDoc->NewDeclaration();
        hisRouteDoc->LinkEndChild(dec);
        XMLComment* comment = hisRouteDoc->NewComment("time=enterTime");
        hisRouteDoc->LinkEndChild(comment);
        hisRouteRoot = hisRouteDoc->NewElement("CARS");
        hisRouteDoc->LinkEndChild(hisRouteRoot);
        for (multimap<double, WeightRoute*>::iterator itWR =
                hisRouteMapByTime.begin(); itWR != hisRouteMapByTime.end();
                ++itWR) {
            XMLElement* carElm = hisRouteDoc->NewElement("car");
            carElm->SetAttribute("id", itWR->second->car->id.c_str());
            carElm->SetAttribute("time", itWR->second->t);
            string routeStr = "";
            for (list<WeightEdge*>::iterator it = itWR->second->edges.begin();
                    it != itWR->second->edges.end(); ++it) {
                if (it != itWR->second->edges.begin()) {
                    routeStr = routeStr + " ";
                }
                routeStr = routeStr + (*it)->edge->id;
            }
            carElm->SetAttribute("route", routeStr.c_str());
            hisRouteRoot->LinkEndChild(carElm);
        }
        hisRouteDoc->SaveFile((hisRecordXMLPath + ".rou.xml").c_str());
        hisRouteDoc->Clear();
        delete hisRouteDoc;
        hisRouteDoc = NULL;
    }
    if (enableHisDataRecord) {
        hisRouteDoc = new XMLDocument();
        XMLDeclaration* dec = hisRouteDoc->NewDeclaration();
        hisRouteDoc->LinkEndChild(dec);
        XMLComment* comment = hisRouteDoc->NewComment("time=enterTime");
        hisRouteDoc->LinkEndChild(comment);
        hisRouteRoot = hisRouteDoc->NewElement("CARS");
        hisRouteDoc->LinkEndChild(hisRouteRoot);
        for (multimap<double, WeightRoute*>::iterator itWR =
                hisRouteMapByTime.begin(); itWR != hisRouteMapByTime.end();
                ++itWR) {
            XMLElement* carElm = hisRouteDoc->NewElement("car");
            carElm->SetAttribute("id", itWR->second->car->id.c_str());
            carElm->SetAttribute("time", itWR->second->t);
            string routeStr = "";
            for (list<WeightEdge*>::iterator it = itWR->second->edges.begin();
                    it != itWR->second->edges.end(); ++it) {
                if (it != itWR->second->edges.begin()) {
                    routeStr = routeStr + " ";
                }
                routeStr = routeStr + (*it)->edge->id;
            }
            carElm->SetAttribute("route", routeStr.c_str());
            hisRouteRoot->LinkEndChild(carElm);
        }
        hisRouteDoc->SaveFile(
                (hisRecordXMLPath + ".rou."
                        + Fanjing::StringHelper::int2str(majorRoutingType)
                        + ".xml").c_str());
        hisRouteDoc->Clear();
        delete hisRouteDoc;
        hisRouteDoc = NULL;
    }
    if (debug) {
        XMLDocument* doc = new XMLDocument();
        XMLDeclaration* dec = doc->NewDeclaration();
        doc->LinkEndChild(dec);
        XMLComment* comment = doc->NewComment(
                "et=enterTime;lt=laneTime;vt=viaTime;"
                        "it=intervalToLast;opt=outPrimaryEdgeTime");
        doc->LinkEndChild(comment);
        XMLElement* fromEdgeElm;
        XMLElement* toEdgeElm;
        XMLElement* carElm;
        WeightEdge* fromWEdge = NULL;
        WeightLane* wLane = NULL;
        for (map<SMTEdge*, WeightEdge*>::iterator itWE = weightEdgeMap.begin();
                itWE != weightEdgeMap.end(); ++itWE) {
            fromEdgeElm = doc->NewElement("From");
            fromWEdge = itWE->second;
            fromEdgeElm->SetAttribute("id", fromWEdge->edge->id.c_str());
            for (map<SMTEdge*, WeightLane*>::iterator itWL =
                    fromWEdge->w2NextMap.begin();
                    itWL != fromWEdge->w2NextMap.end(); ++itWL) {
                toEdgeElm = doc->NewElement("To");
                wLane = itWL->second;
                toEdgeElm->SetAttribute("id", wLane->to->edge->id.c_str());
                SMTConnection* con = wLane->con;
                toEdgeElm->SetAttribute("t0", con->t0);
                toEdgeElm->SetAttribute("tg", con->tg);
                toEdgeElm->SetAttribute("ty", con->ty);
                toEdgeElm->SetAttribute("tr", con->tr);
                for (map<double, HisInfo*>::iterator itHis =
                        wLane->corpTimeMap.begin();
                        itHis != wLane->corpTimeMap.end(); ++itHis) {
                    carElm = doc->NewElement("CAR");
                    carElm->SetAttribute("id", itHis->second->car->id.c_str());
                    carElm->SetAttribute("et",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->enterTime, 1).c_str());
                    if (itHis->second->next != NULL) {
                        carElm->SetAttribute("next",
                                itHis->second->next->to->edge->id.c_str());
                    } else {
                        carElm->SetAttribute("next", "");
                    }
                    carElm->SetAttribute("lt",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->laneTime, 1).c_str());
                    carElm->SetAttribute("vt",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->viaTime, 1).c_str());
                    carElm->SetAttribute("it",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->intervalToLast, 1).c_str());
                    carElm->SetAttribute("opt",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->enterTime
                                            + itHis->second->laneTime, 1).c_str());
                    toEdgeElm->LinkEndChild(carElm);
                }
                fromEdgeElm->LinkEndChild(toEdgeElm);
            }
            doc->LinkEndChild(fromEdgeElm);
        }
        doc->SaveFile(
                (hisRecordXMLPath + ".cur"
                        + Fanjing::StringHelper::int2str(majorRoutingType)
                        + ".xml").c_str());
        doc->Clear();
        delete doc;
    }
    if (enableHisDataRecord) {
        XMLDocument* doc = new XMLDocument();
        XMLDeclaration* dec = doc->NewDeclaration();
        doc->LinkEndChild(dec);
        XMLComment* comment = doc->NewComment(
                "et=enterTime;lt=laneTime;vt=viaTime;"
                        "it=intervalToLast;opt=outPrimaryEdgeTime");
        doc->LinkEndChild(comment);
        XMLElement* fromEdgeElm;
        XMLElement* toEdgeElm;
        XMLElement* carElm;
        WeightEdge* fromWEdge = NULL;
        WeightLane* wLane = NULL;
        for (map<SMTEdge*, WeightEdge*>::iterator itWE = weightEdgeMap.begin();
                itWE != weightEdgeMap.end(); ++itWE) {
            fromEdgeElm = doc->NewElement("From");
            fromWEdge = itWE->second;
            fromEdgeElm->SetAttribute("id", fromWEdge->edge->id.c_str());
            for (map<SMTEdge*, WeightLane*>::iterator itWL =
                    fromWEdge->w2NextMap.begin();
                    itWL != fromWEdge->w2NextMap.end(); ++itWL) {
                toEdgeElm = doc->NewElement("To");
                wLane = itWL->second;
                toEdgeElm->SetAttribute("id", wLane->to->edge->id.c_str());
                SMTConnection* con = wLane->con;
                toEdgeElm->SetAttribute("t0", con->t0);
                toEdgeElm->SetAttribute("tg", con->tg);
                toEdgeElm->SetAttribute("ty", con->ty);
                toEdgeElm->SetAttribute("tr", con->tr);
                for (map<double, HisInfo*>::iterator itHis =
                        wLane->hisTimeMap.begin();
                        itHis != wLane->hisTimeMap.end(); ++itHis) {
                    carElm = doc->NewElement("CAR");
                    carElm->SetAttribute("id", itHis->second->car->id.c_str());
                    carElm->SetAttribute("et",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->enterTime, 1).c_str());
                    if (itHis->second->next != NULL) {
                        carElm->SetAttribute("next",
                                itHis->second->next->to->edge->id.c_str());
                    } else {
                        carElm->SetAttribute("next", "");
                    }
                    carElm->SetAttribute("lt",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->laneTime, 1).c_str());
                    carElm->SetAttribute("vt",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->viaTime, 1).c_str());
                    carElm->SetAttribute("it",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->intervalToLast, 1).c_str());
                    carElm->SetAttribute("opt",
                            Fanjing::StringHelper::dbl2str(
                                    itHis->second->enterTime
                                            + itHis->second->laneTime, 1).c_str());
                    toEdgeElm->LinkEndChild(carElm);
                }
                fromEdgeElm->LinkEndChild(toEdgeElm);
            }
            doc->LinkEndChild(fromEdgeElm);
        }
        doc->SaveFile(
                (hisRecordXMLPath + ".his"
                        + Fanjing::StringHelper::int2str(majorRoutingType)
                        + ".xml").c_str());
        doc->Clear();
        delete doc;
    }
}

void SMTBaseRouting::importHisXML() {
    XMLDocument* doc = new XMLDocument();
    if (XML_SUCCESS
            == doc->LoadFile((hisRecordXMLPath + ".lane.xml").c_str())) {
        XMLElement* fromEdgeElm;
        XMLElement* toEdgeElm;
        XMLElement* carElm;
        fromEdgeElm = doc->FirstChildElement("From");
        while (fromEdgeElm != NULL) {
            WeightEdge* fromWEdge = weightEdgeMap[getMap()->getSMTEdgeById(
                    fromEdgeElm->Attribute("id"))];
            toEdgeElm = fromEdgeElm->FirstChildElement("To");
            while (toEdgeElm != NULL) {
                WeightLane* toWLane =
                        fromWEdge->w2NextMap[getMap()->getSMTEdgeById(
                                toEdgeElm->Attribute("id"))];
                carElm = toEdgeElm->FirstChildElement("CAR");
                double vtA = 0;
                double itA = 0;
                double nvt = 0;
                double nit = 0;
                double lastEnter = 0;
                while (carElm != NULL) {
                    /*
                     carElm->SetAttribute("id", itHis->second->car->id);
                     carElm->SetAttribute("et",itHis->second->enterTime);
                     if (itHis->second->next != NULL) {
                     carElm->SetAttribute("next",
                     itHis->second->next->to->edge->id);
                     } else {
                     carElm->SetAttribute("next", "");
                     }
                     carElm->SetAttribute("lt",itHis->second->laneTime);
                     carElm->SetAttribute("vt",itHis->second->viaTime);
                     carElm->SetAttribute("it",itHis->second->intervalToLast);
                     carElm->SetAttribute("opt",
                     itHis->second->enterTime + itHis->second->laneTime);
                     */
                    HisInfo* hisInfo = new HisInfo();
                    hisInfo->car =
                            getCarManager()->carMapByID[carElm->Attribute("id")];
                    hisInfo->enterTime = carElm->DoubleAttribute("et");
                    string nextEdge = carElm->Attribute("next");
                    if (nextEdge != "") {
                        hisInfo->next =
                                toWLane->to->w2NextMap[getMap()->getSMTEdgeById(
                                        nextEdge)];
                    } else {
                        hisInfo->next = NULL;
                    }
                    hisInfo->laneTime = carElm->DoubleAttribute("lt");
                    hisInfo->viaTime = carElm->DoubleAttribute("vt");
                    hisInfo->intervalToLast = carElm->DoubleAttribute("it");
                    hisInfo->tau = hisInfo->laneTime + hisInfo->enterTime;
                    hisInfo->outTime = hisInfo->tau + hisInfo->viaTime;
                    if (debug) {
                        std::cout << "add car " << hisInfo->car->id
                                << " into lane from " << fromWEdge->edge->id
                                << " to " << toWLane->from->edge->id
                                << ", the next edge is "
                                << (hisInfo->next == NULL ?
                                        "NULL" : hisInfo->next->from->edge->id)
                                << "." << std::endl;
                    }
                    if (hisInfo->viaTime < toWLane->viaLen / 25) {
                        vtA += hisInfo->viaTime;
                        ++nvt;
                    }
                    if (hisInfo->intervalToLast > hisInfo->enterTime - lastEnter
                            && hisInfo->intervalToLast < 3) {
                        itA += hisInfo->intervalToLast;
                        ++nit;
                    }
                    lastEnter = hisInfo->enterTime;
                    // toWLane->addHistoricalCar(hisInfo->car, hisInfo->enterTime);
                    delete hisInfo;
                    carElm = carElm->NextSiblingElement("CAR");
                }
                if (nvt > 0) {
                    toWLane->corpOta = vtA / nvt;
                }
                if (nit > 0) {
                    toWLane->corpEta = itA / nit;
                    toWLane->corpCalcEta = toWLane->corpOta;
                }
                if (debug) {
                    std::cout << "corpOta:" << toWLane->corpOta << ",vtA:"
                            << vtA << ",vtN" << nvt << std::endl;
                    std::cout << "corpEta:" << toWLane->corpEta << ",itA:"
                            << itA << ",itN" << nit << std::endl;
                }
                toEdgeElm = toEdgeElm->FirstChildElement("To");
            }
            fromEdgeElm = fromEdgeElm->NextSiblingElement("From");
        }
    }
    doc->Clear();

    hisRouteDoc = new XMLDocument();
    if (XML_SUCCESS
            == hisRouteDoc->LoadFile((hisRecordXMLPath + ".rou.xml").c_str())) {
        hisRouteRoot = hisRouteDoc->FirstChildElement("CARS");
        XMLElement* carElm = hisRouteRoot->FirstChildElement("car");
        while (carElm != NULL) {
            WeightRoute* rou = new WeightRoute();
            rou->car = getCarManager()->carMapByID[carElm->Attribute("id")];
            rou->t = carElm->DoubleAttribute("time");
            list<string> strRou = Fanjing::StringHelper::splitStringToWordsList(
                    carElm->Attribute("route"));
            for (list<string>::iterator it = strRou.begin(); it != strRou.end();
                    ++it) {
                rou->edges.push_back(
                        weightEdgeMap[getMap()->getSMTEdgeById(*it)]);
            }
            // FIX the burst flow
//            map<SMTCarInfo*, HisInfo*>::iterator it = block->lane->hisCarMap.find(
//                    block->car);
//            if (it != block->lane->hisCarMap.end()) {
//                block->toTime = it->second->enterTime;
//                block->timeStamp = it->second->enterTime;
//            }
            if (enableCoRPPreImport) {
//                if(rou->car->id=="car20"){
//                    rou->cost = 0;
//                }
                addCoRPCar(rou);
            }
            hisRouteMapByCar[rou->car] = rou;
            hisRouteMapByTime.insert(std::make_pair(rou->t, rou));
            carElm = carElm->NextSiblingElement("car");
        }
    }
    hisRouteDoc->Clear();
    std::cout << "import historical routes:" << hisRouteMapByCar.size()
            << std::endl;
    if (enableHisDataRecord) {
        hisRouteMapByTime.clear();
    }
}

void SMTBaseRouting::updateCoRPQueue() {
    WeightLane::updateCoRPCar();
}

void SMTBaseRouting::getOldRoute(SMTEdge* origin, SMTEdge* destination,
        list<SMTEdge*>& rou, double time, SMTCarInfo* car) {
    ASSERT2(hisRouteMapByCar.find(car) != hisRouteMapByCar.end(),
            "missing historical route data");
    WeightRoute* oldrou = hisRouteMapByCar[car];
    ASSERT2(
            origin == oldrou->edges.front()->edge
                    && destination == oldrou->edges.back()->edge,
            "unmatched historical route data");

    for (list<WeightEdge*>::iterator it = oldrou->edges.begin();
            it != oldrou->edges.end(); ++it) {
        rou.push_back((*it)->edge);
    }
}

void SMTBaseRouting::addCoRPCar(WeightRoute* rou) {
    // 为了避免交叉更新,增加路径需要每次独立添加至corpAddQueue
    // 并在完成corpUpdateQueue更新后依次进行
    if (rou->edges.size() > 1) {
        // 只有一条道路的属于短暂车辆,不影响道路状态
        CoRPUpdateBlock* block = new CoRPUpdateBlock();
        block->fromTime = -1;
        block->toTime = rou->t;
        block->timeStamp = rou->t;
        block->car = rou->car;
        block->rouIt = rou->edges.begin();
        ++(block->rouIt);
        block->lane = rou->edges.front()->w2NextMap[(*(block->rouIt))->edge];
        block->rou = new WeightRoute();
        block->rou->edges = rou->edges;
        block->rouIt = block->rou->edges.begin();
        ++(block->rouIt);
        corpAddQueue.insert(std::make_pair(block->timeStamp, block));
        updateCoRPQueue();
    }
}

void SMTBaseRouting::removeCoRPCar(WeightRoute* rou) {
    // remove队列具有最高优先级
    // 依次删除所有队列,然后依次更新被影响车辆
    if (rou->edges.size() > 1) {
        // 只有一条道路的属于短暂车辆,不影响道路状态
        CoRPUpdateBlock* block = new CoRPUpdateBlock();
        block->fromTime = rou->t;
        block->toTime = -1;
        block->timeStamp = rou->t;
        block->car = rou->car;
        block->rouIt = rou->edges.begin();
        ++(block->rouIt);
        block->lane = rou->edges.front()->w2NextMap[(*(block->rouIt))->edge];
        // FIX the burst flow
//        map<SMTCarInfo*, HisInfo*>::iterator it = block->lane->hisCarMap.find(
//                block->car);
//        if (it != block->lane->hisCarMap.end()) {
//            block->fromTime = it->second->enterTime;
//            block->timeStamp = it->second->enterTime;
//        }
        // FIXME [delay]似乎没用
        if (debug) {
            block->rou = new WeightRoute();
            block->rou->edges = rou->edges;
            block->rouIt = block->rou->edges.begin();
            ++(block->rouIt);
        }
        corpRemoveQueue.insert(std::make_pair(block->timeStamp, block));
        updateCoRPQueue();
    }
}

void SMTBaseRouting::updateCoRPInfo() {
    ++corpUpdateCount;
    if (corpUpdateCount == 5) {
        updateCoRPAllLaneInfo(simTime().dbl());
        corpUpdateCount = 0;
    } else {
        updateCoRPSuppressedLaneInfo(simTime().dbl());
    }
}

void SMTBaseRouting::updateCoRPAllLaneInfo(double curTime) {
    WeightEdge* fromWEdge = NULL;
    for (map<SMTEdge*, WeightEdge*>::iterator itWE = weightEdgeMap.begin();
            itWE != weightEdgeMap.end(); ++itWE) {
        fromWEdge = itWE->second;
        for (map<SMTEdge*, WeightLane*>::iterator itWL =
                fromWEdge->w2NextMap.begin();
                itWL != fromWEdge->w2NextMap.end(); ++itWL) {
            itWL->second->updateCoRPEdgeTime(curTime);
        }
    }
}

void SMTBaseRouting::updateCoRPSuppressedLaneInfo(double curTime) {
    if (suppressedEdges.size() > 0) {
        WeightEdge* fromWEdge = NULL;
        for (map<SMTEdge*, double>::iterator itSE = suppressedEdges.begin();
                itSE != suppressedEdges.end(); ++itSE) {
            fromWEdge = weightEdgeMap[itSE->first];
            for (map<SMTEdge*, WeightLane*>::iterator itWL =
                    fromWEdge->w2NextMap.begin();
                    itWL != fromWEdge->w2NextMap.end(); ++itWL) {
                itWL->second->updateCoRPEdgeTime(curTime);
            }
        }
    }
}

void SMTBaseRouting::getDijkstralResult(SMTEdge* destination,
        list<SMTEdge*>& route) {
    WeightEdge* wEdge = weightEdgeMap[destination];
    if (recordHisRoutingResult || enableCoRP) {
        // if use dynamic routing
        WeightRoute* rou = new WeightRoute();
        rou->car = carInfo;
        rou->t = startTime;
        while (wEdge != NULL) {
            route.push_front(wEdge->edge);
            rou->edges.push_front(wEdge);
            wEdge = wEdge->previous;
        }
        if (enableCoRP) {
            addCoRPCar(rou);
        }
        delete rou;
    } else {
        while (wEdge != NULL) {
            route.push_front(wEdge->edge);
            wEdge = wEdge->previous;
        }
    }
}

void SMTBaseRouting::changeRoad(SMTEdge* from, SMTEdge* to, int toLaneIndex,
        double time, SMTCarInfo* car, double viaTime, double laneTime,
        list<SMTEdge*> &passdRoute, double startTime) {
    // update pass time and remove car from weightEdge 'from'
    WeightLane* fromLane = NULL;
    WeightLane* toLane = NULL;
    // if from == NULL, the car enter map first time
    if (from != NULL) {
        map<SMTEdge*, WeightEdge*>::iterator itFromEdge = weightEdgeMap.find(
                from);
        map<SMTEdge*, WeightLane*>::iterator itFromLane =
                itFromEdge->second->w2NextMap.find(to);
        fromLane = itFromLane->second;
        fromLane->removeCar(car, time);
        if (viaTime > 0) {
            fromLane->carPassVia(viaTime);
        } else {
            std::cout << "Untouchable code in SMTBaseRouting::changeRoad"
                    << std::endl;
        }
        if (laneTime > 0) {
            fromLane->carPassLane(laneTime);
        } else {
            std::cout << "Untouchable code in SMTBaseRouting::changeRoad"
                    << std::endl;
        }
        // 执行更新队列
        if (enableCoRP) {
            fromLane->setCoRPOutInfo(car, laneTime, viaTime, time);
            updateCoRPQueue();
        }
        // 如果使用动态寻路, 更新道路情况
    }
    // add car into weightEdge 'to'
    if (toLaneIndex != -1) {
        map<SMTEdge*, WeightEdge*>::iterator itToEdge = weightEdgeMap.find(to);
        if (itToEdge->first->laneVector[toLaneIndex]->nextVector.size() > 1) {
            std::cout << "system does not support multiple link for now@"
                    << itToEdge->first->laneVector[toLaneIndex]->id
                    << std::endl;
        }
        SMTEdge* next =
                itToEdge->first->laneVector[toLaneIndex]->nextVector[0]->edge;
        map<SMTEdge*, WeightLane*>::iterator itToLane =
                itToEdge->second->w2NextMap.find(next);
        toLane = itToLane->second;
        toLane->insertCar(car, time);
    } else {
        // car reaches destination
        // 构造历史路径
        WeightRoute* rou = new WeightRoute();
        rou->car = car;
        rou->t = startTime;
        for (list<SMTEdge*>::iterator it = passdRoute.begin();
                it != passdRoute.end(); ++it) {
            rou->edges.push_back(weightEdgeMap[(*it)]);
        }
        if (enableCoRP) {
            delete (hisRouteMapByCar[carInfo]);
        }
        hisRouteMapByCar[carInfo] = rou;
        if (recordHisRoutingResult || enableHisDataRecord) {
            hisRouteMapByTime.insert(std::make_pair(rou->t, rou));
        }
    }
    if (recordHisRoutingData || enableHisDataRecord) {
        if (fromLane != NULL) {
            // if toLane is NULL, the car reached destination
            fromLane->getOutHistoricalCar(car, laneTime, viaTime, time, toLane);
        }
        if (toLane != NULL) {
            toLane->addHistoricalCar(car, time);
        }
    }
}

double SMTBaseRouting::modifyWeightFromEdgeToEdge(WeightEdge* from,
        WeightEdge* to) {
    // w means the delta weight from wEdge to next
    double deltaW = -1;
    double costTime = 0;
    if (to == NULL) {
        std::cout << "processDijkstralNeighbors:" << " No edge " << to->edge->id
                << " in weightEdgeMap" << std::endl;
    }
    map<SMTEdge*, WeightLane*>::iterator itWL;
    switch (routeType) {
    case SMT_RT_SHOREST:
        for (unsigned int i = 0; i < from->edge->viaVecMap[to->edge].size();
                ++i) {
            double viaLen = from->edge->viaVecMap[to->edge][i]->getViaLength();
            deltaW = getSmallerOne(deltaW, viaLen);
        }
        if (deltaW < 0) {
            std::cout << "processDijkstralNeighbors:"
                    << "cannot handle negative via cost from" << from->edge->id
                    << " to " << to->edge->id << std::endl;
        }
        if (startTime != -1 && carInfo != NULL) {
            to->t = from->t + deltaW / carInfo->maxSpeed;
        }
        deltaW += from->edge->length();

        changeDijkstraWeight(from, to, deltaW + from->w);
        break;
    case SMT_RT_FAST:
        itWL = from->w2NextMap.find(to->edge);
        // since w2NextMap is initialized in initialize()
        // itWL will never equal to from->w2NextMap.end()
        ASSERT2(itWL != from->w2NextMap.end(),
                "w2NextMap initialized abnormally");
        if (itWL->second->getCost(simTime().dbl()) > 0) {
            deltaW = itWL->second->getCost(simTime().dbl());
        } else {
            deltaW = (from->edge->length()
                    + from->edge->viaVecMap[to->edge][0]->getViaLength())
                    / carInfo->maxSpeed;
        }
        if (WeightLane::limitStart > 0) {
            // fix deltaW by occupation if occupation is bigger than half
            // fix deltaW only when cars in this lane cannot pass in one green time
            // and the occupation reach the limit
            if (itWL->second->occupation > WeightLane::limitStart
                    && itWL->second->occupation
                            * itWL->second->from->edge->length()
                            > (carInfo->length + carInfo->minGap) * 24) {
                if (itWL->second->occupation
                        < WeightLane::limitCap - WeightLane::limitFix) {
                    deltaW = deltaW
                            / (WeightLane::limitCap - itWL->second->occupation);
                } else {
                    if (debug) {
                        if (itWL->second->occupaChangeFlagForDebug) {
                            itWL->second->occupaChangeFlagForDebug = false;
                            std::cout << "occupation from " << from->edge->id
                                    << " to " << to->edge->id << " is "
                                    << itWL->second->occupation << std::endl;
                        }
                    }
                    deltaW = deltaW * 1000;
                }
            }
        }
        if (deltaW < 0) {
            std::cout << "processDijkstralNeighbors:"
                    << "cannot handle negative via cost from" << from->edge->id
                    << " to " << to->edge->id << std::endl;
        }
        changeDijkstraWeight(from, to, deltaW + from->w);
        break;
    case SMT_RT_AIR:
        itWL = from->w2NextMap.find(to->edge);
        // since w2NextMap is initialized in initialize()
        // itWL will never equal to from->w2NextMap.end()
        ASSERT2(itWL != from->w2NextMap.end(),
                "w2NextMap initialized abnormally");

        // whether use fast with occupancy replace air routing
        if (replaceAIRWithITSWithOccupancy) {
            // if replace air, use airK as start, airV as cap
            if (itWL->second->getCost(simTime().dbl()) > 0) {
                deltaW = itWL->second->getCost(simTime().dbl());
            } else {
                deltaW = (from->edge->length()
                        + from->edge->viaVecMap[to->edge][0]->getViaLength())
                        / carInfo->maxSpeed;
            }
            if (WeightLane::airK > 0) {
                // fix deltaW by occupation if occupation is bigger than half
                // fix deltaW only when cars in this lane cannot pass in one green time
                // and the occupation reach the limit
                if (itWL->second->occupation > WeightLane::airK
                        && itWL->second->occupation
                                * itWL->second->from->edge->length()
                                > (carInfo->length + carInfo->minGap) * 24) {
                    if (itWL->second->occupation < WeightLane::airV - 0.01) {
                        deltaW = deltaW
                                / (WeightLane::airV - itWL->second->occupation);
                    } else {
                        if (debug) {
                            if (itWL->second->occupaChangeFlagForDebug) {
                                itWL->second->occupaChangeFlagForDebug = false;
                                std::cout << "occupation from "
                                        << from->edge->id << " to "
                                        << to->edge->id << " is "
                                        << itWL->second->occupation
                                        << std::endl;
                            }
                        }
                        deltaW = deltaW * 1000;
                    }
                }
            }
        } else {
            if (itWL->second->getAIRCost(simTime().dbl()) > 0) {
                deltaW = itWL->second->getAIRCost(simTime().dbl());
            } else {
                deltaW = (from->edge->length()
                        + from->edge->viaVecMap[to->edge][0]->getViaLength())
                        / carInfo->maxSpeed;
            }
        }
        if (deltaW < 0) {
            std::cout << "processDijkstralNeighbors:"
                    << "cannot handle negative via cost from" << from->edge->id
                    << " to " << to->edge->id << std::endl;
        }
        changeDijkstraWeight(from, to, deltaW + from->w);
        break;
    case SMT_RT_CORP_SELF:
        itWL = from->w2NextMap.find(to->edge);
        // since w2NextMap is initialized in initialize()
        // itWL will never equal to from->w2NextMap.end()
        ASSERT2(itWL != from->w2NextMap.end(),
                "w2NextMap initialized abnormally");
        deltaW = itWL->second->getCoRPSelfCost(from->t, carInfo, costTime);
        if (deltaW < 0) {
            std::cout << "processDijkstralNeighbors:"
                    << "cannot handle negative via cost from" << from->edge->id
                    << " to " << to->edge->id << std::endl;
        }
        changeDijkstraWeight(from, to, deltaW + from->w, from->t + costTime);
        break;
    case SMT_RT_CORP_TTS:
        itWL = from->w2NextMap.find(to->edge);
        // since w2NextMap is initialized in initialize()
        // itWL will never equal to from->w2NextMap.end()
        ASSERT2(itWL != from->w2NextMap.end(),
                "w2NextMap initialized abnormally");
        deltaW = itWL->second->getCoRPTTSCost(from->t, carInfo, costTime);
        if (false) {
            if (suppressedEdges.size() > 0) {
                map<SMTEdge*, double>::iterator itSE = suppressedEdges.find(
                        from->edge);
                if (itSE != suppressedEdges.end()) {
                    deltaW = deltaW * itSE->second / 17;
                }
            }
        }
        if (deltaW < 0) {
            std::cout << "processDijkstralNeighbors:"
                    << "cannot handle negative via cost from" << from->edge->id
                    << " to " << to->edge->id << std::endl;
        }
        changeDijkstraWeight(from, to, deltaW + from->w, from->t + costTime);
        break;
    case SMT_RT_DYRP:
        // the cost fix function needs change here
        itWL = from->w2NextMap.find(to->edge);
        // since w2NextMap is initialized in initialize()
        // itWL will never equal to from->w2NextMap.end()
        ASSERT2(itWL != from->w2NextMap.end(),
                "w2NextMap initialized abnormally");
        if (itWL->second->getCost(simTime().dbl()) > 0) {
            deltaW = itWL->second->getCost(simTime().dbl());
        } else {
            deltaW = (from->edge->length()
                    + from->edge->viaVecMap[to->edge][0]->getViaLength())
                    / carInfo->maxSpeed;
        }
        // fix deltaW by occupation if occupation is bigger than half
        // fix deltaW only when cars in this lane cannot pass in one green time
        // and the occupation reach the limit
        if (itWL->second->occupation > WeightLane::airK
                && itWL->second->occupation * itWL->second->from->edge->length()
                        > (carInfo->length + carInfo->minGap) * 24) {
            if (itWL->second->occupation < WeightLane::airV - 0.01) {
                deltaW = deltaW / (WeightLane::airV - itWL->second->occupation);
            } else {
                if (debug) {
                    if (itWL->second->occupaChangeFlagForDebug) {
                        itWL->second->occupaChangeFlagForDebug = false;
                        std::cout << "occupation from " << from->edge->id
                                << " to " << to->edge->id << " is "
                                << itWL->second->occupation << std::endl;
                    }
                }
                deltaW = deltaW * 1000;
            }
        }

        if (deltaW < 0) {
            std::cout << "processDijkstralNeighbors:"
                    << "cannot handle negative via cost from" << from->edge->id
                    << " to " << to->edge->id << std::endl;
        }
        changeDijkstraWeight(from, to, deltaW + from->w);
        break;
    default:
        break;
    }
    return deltaW;
}

bool SMTBaseRouting::suppressEdge(SMTEdge* edge, double pos) {
    map<SMTEdge*, double>::iterator it = suppressedEdges.find(edge);
    if (it == suppressedEdges.end()) {
        if (pos < 0) {
            suppressedEdges[edge] = suppressLength;
        } else {
            suppressedEdges[edge] = pos;
        }
        return true;
    }
    return false;
}

void SMTBaseRouting::releaseEdge(SMTEdge* edge) {
    suppressedEdges.erase(edge);
}

SMTBaseRouting::WeightLane::~WeightLane() {
    // release HisInfo in hisCarMap
    for (map<SMTCarInfo*, HisInfo*>::iterator it = hisCarMap.begin();
            it != hisCarMap.end(); ++it) {
        delete (it->second);
    }
    // release HisInfo in corpCarMap
    for (map<SMTCarInfo*, HisInfo*>::iterator it = corpCarMap.begin();
            it != corpCarMap.end(); ++it) {
        delete (it->second);
    }
}

void SMTBaseRouting::WeightLane::carGetOut(SMTCarInfo* car, const double& t,
        const double& cost) {
    recentOutCars.insert(make_pair(t, CarTime(car, cost)));
    totalRecentCost += cost;
    recentCostRefreshFlag = true;
}

void SMTBaseRouting::WeightLane::initMinAllowedCost() {
    SMTConnection* con = via->start->laneVector[via->fromLane]->conVector[0];
    if (minAllowedCostFix) {
        minAllowedCost = ((con->tr + con->ty / 2) * (con->tr + con->ty / 2) / 2)
                / (con->tr + con->tg + con->ty)
                + con->fromSMTEdge->length()
                        / via->start->laneVector[via->fromLane]->speed;
    } else {
        minAllowedCost = con->fromSMTEdge->length()
                / via->start->laneVector[via->fromLane]->speed;
    }
}

void SMTBaseRouting::WeightLane::carPassVia(double time) {
    if (status.maxViaPassTime < time) {
        status.maxViaPassTime = time;
    }
    if (status.minViaPassTime > time || status.minViaPassTime < 0) {
        status.minViaPassTime = time;
    }
    ++status.passedCarNum;
    status.totalViaPassTime += time;
}

void SMTBaseRouting::WeightLane::carPassLane(double time) {
    if (status.maxLanePassTime < time) {
        status.maxLanePassTime = time;
    }
    if (status.minLanePassTime > time || status.minLanePassTime < 0) {
        status.minLanePassTime = time;
    }
    status.totalLanePassTime += time;
}

void SMTBaseRouting::WeightLane::insertCar(SMTCarInfo* car, double t) {
    ASSERT2(carMap.find(car) == carMap.end(),
            "car has been already in this lane");
    carMap[car] = t;
    enterTimeMap.insert(std::make_pair(t, car));
    // update occupation information
    occupation += occStep * (car->length + car->minGap);
    occupaChangeFlagForDebug = true;
    airCostUpdateFlag = true;
}

void SMTBaseRouting::WeightLane::removeCar(SMTCarInfo* car, double t) {
    map<SMTCarInfo*, double>::iterator itCar = carMap.find(car);
    multimap<double, SMTCarInfo*>::iterator itT = enterTimeMap.find(
            itCar->second);
    while (itT->second != car) {
        ++itT;
        if (itT->first != itCar->second) {
            std::cout << "try to remove inexistent car " << itCar->first->id
                    << ", but find car " << itT->second->id << std::endl;
        }
    }
    carGetOut(car, t, t - itCar->second);
    enterTimeMap.erase(itT);
    carMap.erase(itCar);
    // update occupation information
    occupation -= occStep * (car->length + car->minGap);
    occupaChangeFlagForDebug = true;
    airCostUpdateFlag = true;
}

void SMTBaseRouting::WeightLane::updateCost(double time) {
    // update outed car map when new car out or time pass
    if (time > recentCostLastupdateTime || recentCostRefreshFlag) {
        // remove invalid outed car
        for (multimap<double, CarTime>::iterator it = recentOutCars.begin();
                it != recentOutCars.end(); it = recentOutCars.begin()) {
            // keep at least three cars
            if (recentOutCars.size() > 3
                    && it->first < time - outCarKeepDuration) {
                totalRecentCost -= it->second.cost;
                recentOutCars.erase(it);
                recentCostRefreshFlag = true;
            } else {
                break;
            }
        }
    }
    // update cost value
    if (recentCostRefreshFlag) {
        // recentOutCars.size() will never be zero
        // when recentCostRefreshFlag is true
        recentCost = totalRecentCost / recentOutCars.size();
        if (recentOutCars.size() <= 3) {
            if (enterTimeMap.size() == 0) {
                recentCost = minAllowedCost;
            } else if (minRecentCostFix) {
                if (recentCost < time - enterTimeMap.begin()->first) {
                    recentCost = time - enterTimeMap.begin()->first;
                }
            }
        }
        recentCost = recentCost > minAllowedCost ? recentCost : minAllowedCost;
        recentCostLastupdateTime = time;
        recentCostRefreshFlag = false;
    }
}

double SMTBaseRouting::WeightLane::getCost(double time) {
    updateCost(time);
    return recentCost;
}

void SMTBaseRouting::WeightLane::updateAIRsi() {
    airSI = airSI - airK + occupation * airV;
    if (airSI < 0) {
        airSI = 0;
    } else if (airSI > 1) {
        airSI = 1;
    }
    airCostUpdateFlag = true;
}

double SMTBaseRouting::WeightLane::getAIRCost(double time) {
    // update air cost
    if (airCostUpdateFlag && time > airDLastUpdateTime) {
        if (airSI > 0.9999) {
            airD = 10000 * getCost(time);
        } else {
            airD = getCost(time) / (1 - airSI);
        }
        airDLastUpdateTime = time;
    }
    return airD;
}

void SMTBaseRouting::WeightLane::addHistoricalCar(SMTCarInfo* car, double t) {
    ASSERT2(hisCarMap.find(car) == hisCarMap.end(),
            "car has been already in this lane");
    HisInfo* hisInfo = new HisInfo();
    hisInfo->enterTime = t;
    hisInfo->car = car;
    hisCarMap[car] = hisInfo;
    hisTimeMap.insert(std::make_pair(t, hisInfo));
}

void SMTBaseRouting::WeightLane::getOutHistoricalCar(SMTCarInfo* car,
        double laneTime, double viaTime, double time, WeightLane* next) {
    map<SMTCarInfo*, HisInfo*>::iterator it = hisCarMap.find(car);
    if (it == hisCarMap.end()) {
        std::cout << "try to get out unknown car " << car->id << std::endl;
        return;
    }
    it->second->laneTime = laneTime;
    it->second->viaTime = viaTime;
    it->second->tau = it->second->enterTime + laneTime;
    it->second->intervalToLast = time - lastCarOutTime;
    it->second->outTime = it->second->tau + it->second->viaTime;
    lastCarOutTime = time;
    it->second->next = next;
}

void SMTBaseRouting::WeightLane::removeHistoricalCar(SMTCarInfo* car,
        double t) {
    map<SMTCarInfo*, HisInfo*>::iterator itCar = hisCarMap.find(car);
    multimap<double, HisInfo*>::iterator itT = hisTimeMap.find(
            itCar->second->enterTime);
    while (itT->second->car != car) {
        ++itT;
        if (itT->first != itCar->second->enterTime) {
            std::cout << "try to remove inexistent car in hisTimeMap"
                    << itCar->first->id << ", but find car "
                    << itT->second->car->id << std::endl;
        }
    }
    hisTimeMap.erase(itT);
    delete (itCar->second);
    hisCarMap.erase(itCar);
}

multimap<double, SMTBaseRouting::HisInfo*>::iterator SMTBaseRouting::WeightLane::getCoRPOutTime(
        HisInfo* hisInfo) {
    // get the corresponding out info in corpCarMap
    // of the historical info of car
    // find previous car in the corpTimeMap or begin car if no previous car
    ASSERT2(corpCarMap.find(hisInfo->car) == corpCarMap.end(),
            "remove the info in corpMap, before routing method.");
    multimap<double, HisInfo*>::iterator it = corpTimeMap.upper_bound(
            hisInfo->enterTime);
    double tau = from->edge->length() / (hisInfo->car->maxSpeed - 0.05)
            + hisInfo->enterTime;
#ifdef _FANJING_BASE_ROUTING_DEBUG
    double reach = tau;
#endif
    // fix by previous car
    if (it != corpTimeMap.begin()) {
        --it;
        if (tau <= it->second->tau + corpEta) {
            tau = it->second->tau + corpEta;
        }
    }
    // fix by traffic signal
#ifdef _FANJING_BASE_ROUTING_DEBUG
    double beforeSignal = tau;
#endif
    double _fmod = std::fmod(tau + con->_t0, con->ta);
#ifdef _FANJING_BASE_ROUTING_DEBUG
    double signalTime = tau - _fmod + con->ta;
#endif
    if (_fmod > con->tg) {
        tau = tau - _fmod + con->ta + corpEta;
    }
    hisInfo->laneTime = tau - hisInfo->enterTime;
    hisInfo->tau = tau;
    hisInfo->viaTime = corpOta;
    hisInfo->outTime = hisInfo->tau + hisInfo->viaTime;
#ifdef _FANJING_BASE_ROUTING_DEBUG
    if (debug) {
        std::cout << "t0:" << con->t0 << ", tg:" << con->tg << ", ta:"
        << con->ta << std::endl;
        std::cout << "enter:" << hisInfo->enterTime << ", reach:" << reach
        << ", pre:" << it->second->tau << ", beSi:" << beforeSignal
        << ", sig:" << signalTime << ", tau:" << hisInfo->tau
        << ", out:" << hisInfo->outTime << std::endl;
    }
#endif
    // it is the previous car last car whose enter time is not later than hisInfo
    return it;
}

double SMTBaseRouting::WeightLane::getCoRPSelfCost(double enterTime,
        SMTCarInfo* car, double &costTime) {
    tempHisInfo.enterTime = enterTime;
    tempHisInfo.car = car;
    // it is the previous car last car whose enter time is not later than hisInfo
    // it是enter time之前的最后一辆进入车辆
    multimap<double, SMTBaseRouting::HisInfo*>::iterator it = getCoRPOutTime(
            &tempHisInfo);
    // the iterator of the starter of queue
    multimap<double, SMTBaseRouting::HisInfo*>::iterator itQS = it;
    costTime = tempHisInfo.outTime - enterTime;
    double queueLen = getCoRPQueueLength(enterTime, itQS);
    double m = 1;
    double p = 0;
    getCoRPQueueFixPar(queueLen, m, p);
#ifdef _FANJING_BASE_ROUTING_DEBUG
    if (debug) {
        if (m != 1 || p != 0) {
            std::cout << "at time " << enterTime << ", edge:" << from->edge->id
            << ", m = " << m << ", p = " << p << std::endl;
        }
    }
#endif
    return m * (costTime + p);
}

double SMTBaseRouting::WeightLane::getCoRPTTSCost(double enterTime,
        SMTCarInfo* car, double& costTime) {
    tempHisInfo.enterTime = enterTime;
    tempHisInfo.car = car;
    // it is the previous car last car whose enter time is not later than hisInfo
    // it是enter time之前的最后一辆进入车辆
    // 或者为队列第一辆车如果enter time之前没有车
    multimap<double, SMTBaseRouting::HisInfo*>::iterator it = getCoRPOutTime(
            &tempHisInfo);
    // the iterator of the starter of queue
    multimap<double, SMTBaseRouting::HisInfo*>::iterator itQS = it;
    multimap<double, SMTBaseRouting::HisInfo*>::iterator itQE = it;
    costTime = tempHisInfo.outTime - enterTime;
    double queueLen = getCoRPQueueLength(enterTime, itQS) + car->length
            + car->minGap;
    double maxQueueLen = queueLen;
    // calculate the affection to other cars
    // 遍历enter time后方队列,计算可能的最大队列长度.
    // 计算对所有车辆的形式时间影响总和
    // 一般而言等于退出饱和状态车辆的nextDummyTime-enterTime
    // 对出饱和状态判定基于队列中离开时间tempHisInfo.outTime之后
    // 车辆nFDT(next free decision time)与后一辆离开时间tau之间的大小
    // nFDT<=tau则认为过程终止

    // 判定是否饱和
    // 若后方空间能够容纳额外一辆车辆通过,则认为非饱和,反之饱和
    // 计算如果存在后方紧跟进入车辆,其对应的离开时间
    // (即当前车辆后移后的虚拟通过时间)
    tempHisInfo.nFDT = tempHisInfo.tau + corpEta;
    double _fmod = std::fmod(tempHisInfo.nFDT + con->_t0, con->ta);
    if (_fmod > con->tg) {
        tempHisInfo.nFDT = tempHisInfo.nFDT - _fmod + con->ta + corpEta;
    }
    tempHisInfo.nextDummyTime = tempHisInfo.tau;
    // 判定后方进入车辆是否退出饱和状态
    // 同时进行最大可能队列长度判定

#ifdef _FANJING_WL_DEBUG
    static WeightLane* debugLane = NULL;
    if (debugLane == NULL) {
        if(from->edge->id=="18/12to18/14") {
            if(con->fromLane==1) {
                debugLane = this;
            }
        }
    }
    if (this == debugLane) {
        std::cout << car->id << " -> ";
    }
#endif
    int followingCar = 0;
    if (it != corpTimeMap.end()) {
        // 修正it为enter time 后方车辆
        // 此处不需要修改queuelen,因为it可能为end
        // 之前版本为总是++it
        // 会导致前方没有车辆时跳过插入之后的地一辆车,导致后续判定错误
        if (it->second->enterTime <= enterTime) {
            ++it;
        }
        while (it != corpTimeMap.end()) {
#ifdef _FANJING_WL_DEBUG
            if (this == debugLane) {
                std::cout << it->second->car->id << "-"<<it->second->tau<<" ";
            }
#endif
            if (it->second->tau >= tempHisInfo.nFDT + 0.05) {
                break;
            } else {
                // it所在车辆受前一辆(或者当前插入车辆影响)
                // update queue length
                // it对应车辆进入队列
                queueLen += it->second->car->minGap + it->second->car->length;
                // 移除在it进入队列时已经离开的车辆
                // 仅移除it之前的车辆
                while (itQS != it) {
                    // it进入时itQS已经离开
                    if (itQS->second->tau < it->second->enterTime) {
                        queueLen -= itQS->second->car->minGap
                                + itQS->second->car->length;
                        ++itQS;
                    } else {
                        break;
                    }
                }
                // 更新最大队列长度
                maxQueueLen = maxQueueLen > queueLen ? maxQueueLen : queueLen;
                tempHisInfo.nFDT = it->second->nFDT;
                tempHisInfo.nextDummyTime = it->second->nextDummyTime;
                ++it;
                ++followingCar;
            }
        }
#ifdef _FANJING_WL_DEBUG
        if (this == debugLane) {
            std::cout << std::endl;
        }
#endif
    }
    double m = 1;
    double p = 0;
    getCoRPQueueFixPar(maxQueueLen, m, p);
#ifdef _FANJING_BASE_ROUTING_DEBUG
    if (debug) {
        std::cout << "followed cars at time " << enterTime << ":"
        << followingCar << std::endl;
    }
#endif
    double deltaW = tempHisInfo.nextDummyTime + corpOta - enterTime;

#ifdef _FANJING_BASE_ROUTING_DEBUG
    if (debug) {
        if (m != 1 || p != 0) {
            std::cout << "at time " << enterTime << ", edge:" << from->edge->id
            << "_" << con->fromLane << ", len = " << from->edge->length()
            << ", queue = " << maxQueueLen << ", m = " << m << ", p = " << p
            << ", deltaW = " << deltaW << std::endl;
        }
    }
#endif

#ifdef _FANJING_WL_DEBUG
    if (this == debugLane && (m != 1 || p != 0)) {
        std::cout << "at time " << enterTime << ", edge:" << from->edge->id
        << "_" << con->fromLane << ", len = " << from->edge->length()
        << ", queue = " << maxQueueLen << ", m = " << m << ", p = " << p
        << ", deltaW = " << deltaW << std::endl;
    }
#endif
    return m * (deltaW + p);
}

double SMTBaseRouting::WeightLane::getCoRPQueueLength(double enterTime,
        multimap<double, HisInfo*>::iterator &it) {
    // it指针会被指向队列包含的最早节点
    // 即tau>=enter time且enterTime>enter time的最后一个节点
    double result = 0;
    if (it != corpTimeMap.end()) {
        if (it->second->enterTime > enterTime) {
            // 车辆进入时间必须不晚于enterTime
            return result;
        }
        // 进入时还未离开车辆即队列中车辆
        while (it != corpTimeMap.begin()) {
            if (it->second->tau >= enterTime) {
                result += it->second->car->length + it->second->car->minGap;
            } else {
                // 还原指针
                break;
            }
            --it;
        }
    }
    return result;
}

void SMTBaseRouting::WeightLane::getCoRPQueueFixPar(double queueLen, double& m,
        double& p) {
    double laneLen = from->edge->length();
    // fix only queueLen >= 120 (beyond 28 cars)
    // fix only occupancy >=50%
    double fixedQueueLen = queueLen * 1.3;
    if (queueLen < 140 || fixedQueueLen * 2 < laneLen) {
        m = 1;
        p = 0;
        return;
    }
    double freeLen = laneLen - fixedQueueLen;
//    double ocRatio = fixedQueueLen / laneLen;
    m = 1;
    p = 0;
    if (freeLen > 250) {
        p = 1;
    } else {
        if (freeLen > 100) {
            p = (251 - freeLen) * 10000;
        } else {
            p = 2000000;
        }
    }
//    if (m > 1) {
//        if (laneLen < 800) {
//            p = 800 - laneLen;
//        }
//    }
}

void SMTBaseRouting::WeightLane::updateCoRPEdgeTime(double time) {
    // 更改离开时间,并添加后续影响至队列
    // 检索车辆
    multimap<double, HisInfo*>::iterator itT = corpTimeMap.find(0);
    // 如果没车,啥都不做
    if (itT == corpTimeMap.end()) {
        return;
    }
    // 如果第一辆车离开时间不早于当前时间什么都不做
    if (itT->second->outTime >= time) {
        return;
    }
    // 保留当前车辆指针,用于后续路径操作
    HisInfo* hisInfo = itT->second;
    SMTCarInfo* car = hisInfo->car;
    // itT后移,用于指示下一个车辆
    ++itT;
    // 保存原始下一条道路进入时间
    double oldOutTime = hisInfo->outTime;
    // 更新离开信息
    hisInfo->laneTime = time - corpOta - hisInfo->enterTime;
    hisInfo->viaTime = corpOta;
    hisInfo->tau = time - corpOta;
    hisInfo->outTime = time;
    // 跟新后续影响判定条件
    hisInfo->nextDummyTime = hisInfo->tau + corpEta;
    double _fmod = std::fmod(hisInfo->nextDummyTime + con->_t0, con->ta);
    if (_fmod > con->tg) {
        hisInfo->nextDummyTime = hisInfo->nextDummyTime - _fmod + con->ta
                + corpEta;
        hisInfo->nFDT = hisInfo->nextDummyTime + corpEta;
    } else {
        hisInfo->nFDT = hisInfo->nextDummyTime + corpEta;
        _fmod = std::fmod(hisInfo->nFDT + con->_t0, con->ta);
        if (_fmod > con->tg) {
            hisInfo->nFDT = hisInfo->nFDT - _fmod + con->ta + corpEta;
        }
    }
    // 2nd. 添加后续道路更新和受影响车辆更新
    // 添加后续道路更新
    CoRPUpdateBlock* block = NULL;
    if (hisInfo->next != NULL) {
        // 如果离开状态改变,则进行下一条道路的更新
        if (oldOutTime != hisInfo->outTime) {
            block = new CoRPUpdateBlock();
            block->fromTime = oldOutTime;
            block->toTime = hisInfo->outTime;
            block->timeStamp =
                    block->fromTime < block->toTime ?
                            block->fromTime : block->toTime;
            block->lane = hisInfo->next;
            block->car = car;
            // 修改车辆信息不需要设置route信息,因为对应hisInfo有next参数
            block->rou = NULL;
            block->srcHisInfo = hisInfo;
            corpUpdateQueue->insert(std::make_pair(block->timeStamp, block));
        }
    }
    // 受影响车辆更新
    // 构造out info更新消息
    if (itT != corpTimeMap.end()) {
        hisInfo = itT->second;
        // 设置受影响车辆outInfo更新消息
        block = new CoRPUpdateBlock();
        // 重设block为修改被影响车辆进入时间
        block->fromTime = hisInfo->enterTime;
        block->toTime = hisInfo->enterTime;
        block->timeStamp = hisInfo->enterTime;
        block->lane = this;
        block->car = hisInfo->car;
        // 修改车辆信息不需要设置route信息,因为对应hisInfo有next参数
        block->rou = NULL;
        block->srcHisInfo = hisInfo;
        corpUpdateQueue->insert(std::make_pair(block->timeStamp, block));
    }
}

void SMTBaseRouting::WeightLane::updateCoRPOutInfo(HisInfo* hisInfo,
        multimap<double, HisInfo*>::iterator itHI) {
    // get the corresponding out info in corpCarMap
    // of the historical info of car
    // find previous car in the corpTimeMap
    ASSERT2(itHI->second == hisInfo, "the iterator unmatched.");
    double tau = from->edge->length() / (hisInfo->car->maxSpeed - 0.05)
            + hisInfo->enterTime;
    // fix by previous car
    if (itHI != corpTimeMap.begin()) {
        --itHI;
        if (tau <= itHI->second->tau + corpEta) {
            tau = itHI->second->tau + corpEta;
        }
        ++itHI;
    }
    // fix by traffic signal
    double _fmod = std::fmod(tau + con->_t0, con->ta);
    if (_fmod > con->tg) {
        tau = tau - _fmod + con->ta + corpEta;
    }
    hisInfo->laneTime = tau - hisInfo->enterTime;
    hisInfo->tau = tau;
    hisInfo->viaTime = corpOta;
    hisInfo->outTime = tau + corpOta;
    // 判定是否饱和
    // 若后方空间能够容纳额外一辆车辆通过,则认为非饱和,反之饱和
    // 计算如果存在后方紧跟进入车辆,其对应的离开时间
    // (即当前车辆后移后的虚拟通过时间)
    hisInfo->nextDummyTime = tau + corpEta;
    _fmod = std::fmod(hisInfo->nextDummyTime + con->_t0, con->ta);
    if (_fmod > con->tg) {
        hisInfo->nextDummyTime = hisInfo->nextDummyTime - _fmod + con->ta
                + corpEta;
        hisInfo->nFDT = hisInfo->nextDummyTime + corpEta;
    } else {
        hisInfo->nFDT = hisInfo->nextDummyTime + corpEta;
        _fmod = std::fmod(hisInfo->nFDT + con->_t0, con->ta);
        if (_fmod > con->tg) {
            hisInfo->nFDT = hisInfo->nFDT - _fmod + con->ta + corpEta;
        }
    }
}

void SMTBaseRouting::WeightLane::updateCoRPCar() {
    // 车辆更新策略
    // 车辆更新主要分为2部分
    // 1. 进入时间更新
    // 更新进入时间时间戳为更新前后进入时间较早者
    // 对应一个hisInfo,进入时间更新拥有更高权限
    // 改变一个hisInfo的进入时间需要更新受影响车辆的离开时间
    // 改变一个hisInfo的进入时间不会影响其他车辆的进入时间
    // 更新hisInfo的进入时间需要更新hisInfo后方车辆的离开时间
    // (前移后移后时间中较早者后方第一辆车,时间戳为后方车辆进入时间)
    // 更新hisInfo的进入时间需要更新hisInfo车辆离开时间
    // 时间戳为hisInfo更新后进入时间,即前移立即更新,后移等待至移动后的进入时间
    // 2. 离开时间更新
    // 更新离开时间时间戳为当前车辆进入时间
    // 改变一个hisInfo的离开时间会影响该车下一跳道路的进入时间
    // 改变一个hisInfo的离开时间需要更新受影响车辆的离开时间
    // 改变一个hisInfo的离开时间不会影响其他车辆的进入时间
    // 更新hisInfo的离开时间需要更新hisInfo下一跳的进入时间
    // 更新hisInfo的离开时间需要更新后方进入车辆的离开时间
    // (若后方车辆已经处于需要更新状态则不做其他处理,需要使用调试信息测试)

    // 如果from!=-1,to==-1则表示删除当前车辆
    // 如果from==-1,to!=-1则表示添加车辆
    // 如果from!=-1,to!=-1则表示修改车辆
    // 都不为-1时,如果from!=to则修改进入时间
    // 反之from==to,则修改离开时间
    // 3. 增加车辆路径
    // 见SMTBaseRouting::addCoRPCar
    // 4. 删除车辆路径
    // 见SMTBaseRouting::removeCoRPCar
    // 继续更新queue

    CoRPUpdateBlock* block = NULL;
    while (true) {
        ++operationNum;
        if (corpRemoveQueue->size() > 0) {
            block = corpRemoveQueue->begin()->second;
            block->lane->removeCoRPQueueInfo(block);
        } else if (corpUpdateQueue->size() > 0) {
            block = corpUpdateQueue->begin()->second;
            if (block->fromTime == block->toTime) {
                block->lane->updateCoRPQueueOutInfo(block);
            } else {
                block->lane->updateCoRPQueueEnterInfo(block);
            }
        } else if (corpAddQueue->size() > 0) {
            block = corpAddQueue->begin()->second;
            block->lane->addCoRPQueueInfo(block);
        } else {
            break;
        }
    }
}

void SMTBaseRouting::WeightLane::addCoRPQueueInfo(CoRPUpdateBlock* block) {
    if (block->lane != this) {
        std::cout << "unmatched lane" << std::endl;
        block->lane->addCoRPQueueInfo(block);
        return;
    }
    // 同一时间,之前进入的不做处理
    // 插入车辆信息更新离开信息
    // 将受影响车辆更新需求插入queue
    // 1st. 插入车辆信息
    map<SMTCarInfo*, HisInfo*>::iterator itCar = corpCarMap.find(block->car);
    // 该功能用于导入历史路径信息阶段，此时对应车辆不应存在于对应corpCarMap队列
    if (itCar != corpCarMap.end()) {
        // TODO remove过程改变需要更新
        // 在remove当前车辆信息前先从queue临时移除当前block
        corpAddQueue->erase(corpAddQueue->begin());
        std::cout << "this function is used in import historical xml file."
                << std::endl;
        std::cout << "the car should not be here before insert it by new route."
                << std::endl;
        std::cout << "if you want update its info, use update function instead."
                << std::endl;
        // try to remove this car
        CoRPUpdateBlock* rBlock = new CoRPUpdateBlock();
        rBlock->timeStamp = itCar->second->enterTime;
        rBlock->fromTime = rBlock->timeStamp;
        rBlock->toTime = -1;
        rBlock->lane = block->lane;
        rBlock->car = block->car;
#ifdef _FANJING_BASE_ROUTING_DEBUG
        if (debug) {
            // construct the route of rBlock
            // FIXME [delay] remove operation does not need route info
            // since the hisInfo has it already
            rBlock->rou = new WeightRoute();
            rBlock->rou->t = itCar->second->enterTime;
            rBlock->rou->car = itCar->second->car;
            WeightLane* lane = block->lane;
            rBlock->rou->edges.push_back(lane->from);
            while (itCar->second->next != NULL) {
                rBlock->rou->edges.push_back(lane->to);
                lane = itCar->second->next;
                itCar = lane->corpCarMap.find(block->car);
                if (itCar == lane->corpCarMap.end()) {
                    std::cout << "error@addCoRPCarInfo: "
                    << "the next lane in his info does not contain this car"
                    << std::endl;
                    break;
                }
            }
            rBlock->rouIt = rBlock->rou->edges.begin();
            ++(rBlock->rouIt);
        }
#endif
        rBlock->srcHisInfo = itCar->second;
        corpRemoveQueue->insert(corpRemoveQueue->begin(),
                std::make_pair(rBlock->timeStamp, rBlock));
        updateCoRPCar();
        // 将block加回queue
        corpAddQueue->insert(std::make_pair(block->timeStamp, block));
    }
    // 插入对应车辆信息至corpTimeMap与corpCarMap,并为后续影响修改queue信息
    HisInfo* hisInfo = new HisInfo();
    hisInfo->car = block->car;
    hisInfo->enterTime = block->toTime;
    multimap<double, HisInfo*>::iterator itT = corpTimeMap.upper_bound(
            block->toTime);
    corpCarMap[hisInfo->car] = hisInfo;
    itT = corpTimeMap.insert(itT, std::make_pair(hisInfo->enterTime, hisInfo));
    // 同步更新离开信息
    updateCoRPOutInfo(hisInfo, itT);
    // 对应block处理完毕,从queue移除block
    corpAddQueue->erase(corpAddQueue->begin());
    // 2nd. 后续影响,将受影响车辆更新需求插入queue
    // 如果还有后续道路,则将其加入队列.
    // 若下一跳道路是终点,并且不影响其他车辆则释放当前block内存
    bool releaseFlag = true;
    ++(block->rouIt);
    if (block->rouIt != block->rou->edges.end()) {
        // 若lane->to道路不是终点再利用则当前block不需要被释放
        releaseFlag = false;
        hisInfo->next = block->lane->getNextLane((*(block->rouIt))->edge);
        block->lane = hisInfo->next;
        block->toTime = hisInfo->outTime;
        block->timeStamp = block->toTime;
        block->srcHisInfo = hisInfo;
        corpAddQueue->insert(std::make_pair(block->timeStamp, block));
    } else {
        hisInfo->next = NULL;
        // 在最后的添加block需要释放rou占用的空间
        delete (block->rou);
        block->rou = NULL;
    }
    // 添加后续可能被影响车辆
    ++itT;
    // 判定后方车辆离开信息是否受到该车影响
    if (itT != corpTimeMap.end()) {
        if (itT->second->tau < hisInfo->tau + corpEta) {
            // 判定是否对block进行重利用
            if (!releaseFlag) {
                block = new CoRPUpdateBlock();
            } else {
                releaseFlag = false;
            }
            block->car = itT->second->car;
            block->fromTime = itT->second->enterTime;
            block->timeStamp = block->fromTime;
            block->toTime = block->fromTime;
            block->lane = this;
            // 更新信息不需要设置route信息,因为对应hisInfo有next参数
            block->rou = NULL;
            block->srcHisInfo = itT->second;
            corpUpdateQueue->insert(std::make_pair(block->timeStamp, block));
        }
    }
    if (releaseFlag) {
        delete block;
    }
}

void SMTBaseRouting::WeightLane::removeCoRPQueueInfo(CoRPUpdateBlock* block) {
    // 移除车辆,并添加后续影响至queue
    // 1st. 移除车辆
    if (block->lane != this) {
        std::cout << "unmatched lane" << std::endl;
        block->lane->removeCoRPQueueInfo(block);
        return;
    }
    // 检索车辆
    multimap<SMTCarInfo*, HisInfo*>::iterator itCar = corpCarMap.find(
            block->car);
    ASSERT2(itCar != corpCarMap.end(),
            "try to remove corp car from the lane does not contain it");
    // 保留当前车辆指针,用于后续路径操作
    HisInfo* hisInfo = itCar->second;
    if (block->fromTime != hisInfo->enterTime) {
        std::cout << "unmatched from Time when process remove block"
                << std::endl;
    }
    multimap<double, HisInfo*>::iterator itT = corpTimeMap.find(
            hisInfo->enterTime);
    while (itT != corpTimeMap.end()) {
        if (itT->second->car == block->car) {
            break;
        }
        ASSERT2(itT->first == block->fromTime,
                "no such car in this lane when remove it.");
        ++itT;
    }
    // 保留当前itT用于删除元素
    multimap<double, HisInfo*>::iterator itTold = itT;
    // itT后移,用于指示下一个车辆
    ++itT;
    corpTimeMap.erase(itTold);
    corpCarMap.erase(itCar);
    // 对应block处理完毕,从queue移除block
    corpRemoveQueue->erase(corpRemoveQueue->begin());
    // 2nd. 后续影响
    bool releaseFlag = true;
    // 继续移除当前车辆的后续路径
    if (hisInfo->next != NULL) {
        // 若再利用则当前block不需要被释放
        releaseFlag = false;
        block->fromTime = hisInfo->outTime;
        block->timeStamp = block->fromTime;
        block->lane = hisInfo->next;
#ifdef _FANJING_BASE_ROUTING_DEBUG
        if (debug) {
            if (block->rou != NULL) {
                // FIXME [delay] 似乎没用
                ++(block->rouIt);
            }
        }
#endif
        corpRemoveQueue->insert(std::make_pair(block->timeStamp, block));
    } else {
#ifdef _FANJING_BASE_ROUTING_DEBUG
        if (debug) {
            // FIXME [delay] 似乎没用
            if (block->rou != NULL) {
                delete (block->rou);
                block->rou = NULL;
            }
        }
#endif
    }
    // hisInfo失效,释放hisInfo
    delete hisInfo;
    if (itT != corpTimeMap.end()) {
        // 使用hisInfo记录受影响车辆
        hisInfo = itT->second;
        // 设置outInfo更新消息
        // 如果block内存用于他用,则申请新的空间
        // 反之如果block不在有效,重用block内存空间
        if (!releaseFlag) {
            block = new CoRPUpdateBlock();
        }
        releaseFlag = false;
        // 重设block为修改被影响车辆进入时间
        block->fromTime = hisInfo->enterTime;
        block->toTime = hisInfo->enterTime;
        block->timeStamp = hisInfo->enterTime;
        block->lane = this;
        block->car = hisInfo->car;
        // 修改车辆信息不需要设置route信息,因为对应hisInfo有next参数
        block->rou = NULL;
        block->srcHisInfo = hisInfo;
        corpUpdateQueue->insert(std::make_pair(block->timeStamp, block));
    }
    if (releaseFlag) {
        delete block;
    }
}

void SMTBaseRouting::WeightLane::updateCoRPQueueEnterInfo(
        CoRPUpdateBlock* block) {
    // 移动车辆,并添加后续影响至队列
    // 1st. 移动车辆
    if (block->lane != this) {
        std::cout << "unmatched lane" << std::endl;
        block->lane->updateCoRPQueueEnterInfo(block);
        return;
    }
    // 通过判定toTime是否与srcHisInfo内容一致验证block是否过期
    if (block->srcHisInfo->outTime != block->toTime) {

#ifdef _FANJING_BASE_ROUTING_DEBUG
        if (debug) {
            std::cout << "expired update enter info block" << std::endl;
        }
#endif
        corpUpdateQueue->erase(corpUpdateQueue->begin());
        delete block;
        return;
    }
    // 检索车辆
    multimap<SMTCarInfo*, HisInfo*>::iterator itCar = corpCarMap.find(
            block->car);
    ASSERT2(itCar != corpCarMap.end(),
            "try to update corp car from the lane does not contain it");
    // 保留当前车辆指针,用于后续路径操作
    HisInfo* hisInfo = itCar->second;
    if (block->fromTime != hisInfo->enterTime) {
        // from time 可能由于多次更新前一条道路的离开时间而不匹配
#ifdef _FANJING_BASE_ROUTING_DEBUG
        if (debug) {
            std::cout << "unmatched from Time when process update block"
            << std::endl;
        }
#endif
        // 可能存在上一跳outTime被多次改变使得fromTime无法对齐的问题
        // 因此此处需要进行更新
        block->fromTime = hisInfo->enterTime;
        if (hisInfo->enterTime == block->toTime) {
            // 由于enter time 不变时,仅前一辆车辆离开状态改变会改变后续状态
            // 而对应信息更新由out info update完成
            // 因此在enter info update block中
            // 若多次改变后enter time 不变,则后续状态亦无改变
            corpUpdateQueue->erase(corpUpdateQueue->begin());
            delete block;
            return;
        }
    }
    multimap<double, HisInfo*>::iterator itT = corpTimeMap.find(
            hisInfo->enterTime);
    while (itT != corpTimeMap.end()) {
        if (itT->second->car == block->car) {
            break;
        }
        ASSERT2(itT->first == block->fromTime,
                "no such car in this lane when update it.");
        ++itT;
    }
    // 保留当前itT用于删除元素
    multimap<double, HisInfo*>::iterator itTold = itT;
    // itT后移,用于指示下一个车辆
    ++itT;
    corpTimeMap.erase(itTold);
    // 仅仅做移动,不做删除
    // corpCarMap.erase(itCar);
    // 保存原始下一条道路进入时间
    double oldOutTime = hisInfo->outTime;
    // 更新hisInfo信息
    hisInfo->enterTime = block->toTime;
    // 将hisInfo插回队列
    itTold = corpTimeMap.insert(itT,
            std::make_pair(hisInfo->enterTime, hisInfo));
    updateCoRPOutInfo(hisInfo, itTold);
    // 对应block处理完毕,从queue移除block
    corpUpdateQueue->erase(corpUpdateQueue->begin());
    // 2nd. 添加后续道路更新和受影响车辆更新
    bool releaseFlag = true;
    // 获取时间点最早的受影响车辆,并将其迭代器赋给itTold
    if (block->fromTime > block->toTime) {
        // 前移则最早受影响车辆为更新后后方车辆
        ++itTold;
    } else if (block->fromTime < block->toTime) {
        // 后移则最早受影响车辆为更新前后方车辆
        itTold = itT;
    } else {
        std::cout << "to time equals to from time when process update block"
                << std::endl;
    }
    // 添加后续道路更新
    if (hisInfo->next != NULL) {
        // 如果离开状态改变,则重用block用于更新下一条道路
        if (oldOutTime != hisInfo->outTime) {
            // 若再利用则当前block不需要被释放
            releaseFlag = false;
            block->fromTime = oldOutTime;
            block->toTime = hisInfo->outTime;
            block->timeStamp =
                    block->fromTime < block->toTime ?
                            block->fromTime : block->toTime;
            block->lane = hisInfo->next;
            block->srcHisInfo = hisInfo;
            corpUpdateQueue->insert(std::make_pair(block->timeStamp, block));
        }
    }
    // 受影响车辆更新
    // 构造out info更新消息
    if (itTold != corpTimeMap.end()) {
        hisInfo = itTold->second;
        // 设置outInfo更新消息
        // 如果block内存用于他用,则申请新的空间
        // 反之如果block不在有效,重用block内存空间
        if (!releaseFlag) {
            block = new CoRPUpdateBlock();
        }
        releaseFlag = false;
        // 重设block为修改被影响车辆进入时间
        block->fromTime = hisInfo->enterTime;
        block->toTime = hisInfo->enterTime;
        block->timeStamp = hisInfo->enterTime;
        block->lane = this;
        block->car = hisInfo->car;
        // 修改车辆信息不需要设置route信息,因为对应hisInfo有next参数
        block->rou = NULL;
        block->srcHisInfo = hisInfo;
        corpUpdateQueue->insert(std::make_pair(block->timeStamp, block));
    }
    if (releaseFlag) {
        delete block;
    }
}

void SMTBaseRouting::WeightLane::updateCoRPQueueOutInfo(
        CoRPUpdateBlock* block) {
    if (block->lane != this) {
        std::cout << "unmatched lane" << std::endl;
        block->lane->updateCoRPQueueOutInfo(block);
        return;
    }
    // 通过判定toTime是否与srcHisInfo内容一致验证block是否过期
    if (block->srcHisInfo->enterTime != block->toTime) {
        corpUpdateQueue->erase(corpUpdateQueue->begin());
        delete block;
        return;
    }
    // 检索车辆
    multimap<SMTCarInfo*, HisInfo*>::iterator itCar = corpCarMap.find(
            block->car);
    ASSERT2(itCar != corpCarMap.end(),
            "try to update out info from the lane does not contain it");
    // 保留当前车辆指针,用于后续路径操作
    HisInfo* hisInfo = itCar->second;
    // 通过判定toTime是否与enterTime内容一致验证block是否过期
    if (hisInfo->enterTime != block->toTime) {
        std::cout << "expired update out info block" << std::endl;
        corpUpdateQueue->erase(corpUpdateQueue->begin());
        delete block;
        return;
    }
    multimap<double, HisInfo*>::iterator itT = corpTimeMap.find(
            hisInfo->enterTime);
    while (itT != corpTimeMap.end()) {
        if (itT->second->car == block->car) {
            break;
        }
        ASSERT2(itT->first == block->fromTime,
                "no such car in this lane when update it.");
        ++itT;
    }
    // 更新离开信息
    double oldTau = hisInfo->tau;
    double oldOutTime = hisInfo->outTime;
    updateCoRPOutInfo(hisInfo, itT);
    // 信息更新完成,移除block
    corpUpdateQueue->erase(corpUpdateQueue->begin());
    bool releaseFlag = true;
    // 若离开路口状态改变, 则更新后续车辆
    if (hisInfo->tau != oldTau) {
        // 添加后续道路更新
        if (hisInfo->next != NULL) {
            // 如果离开状态(即进入下一条道路时间)改变,则重用block用于更新下一条道路
            if (oldOutTime != hisInfo->outTime) {
                // 若再利用则当前block不需要被释放
                releaseFlag = false;
                block->fromTime = oldOutTime;
                block->toTime = hisInfo->outTime;
                block->timeStamp =
                        block->fromTime < block->toTime ?
                                block->fromTime : block->toTime;
                block->lane = hisInfo->next;
                block->srcHisInfo = hisInfo;
                corpUpdateQueue->insert(
                        std::make_pair(block->timeStamp, block));
            }
        }
        // 添加后方受影响车辆更新信息
        // 获取后方车辆
        ++itT;
        // 构造out info更新消息
        if (itT != corpTimeMap.end()) {
            hisInfo = itT->second;
            // 设置outInfo更新消息
            // 如果block内存用于他用,则申请新的空间
            // 反之如果block不在有效,重用block内存空间
            if (!releaseFlag) {
                block = new CoRPUpdateBlock();
            }
            releaseFlag = false;
            // 重设block为修改被影响车辆进入时间
            block->fromTime = hisInfo->enterTime;
            block->toTime = hisInfo->enterTime;
            block->timeStamp = hisInfo->enterTime;
            block->lane = this;
            block->car = hisInfo->car;
            // 修改车辆信息不需要设置route信息,因为对应hisInfo有next参数
            block->rou = NULL;
            block->srcHisInfo = hisInfo;
            corpUpdateQueue->insert(std::make_pair(block->timeStamp, block));
        }
    }
    if (releaseFlag) {
        delete block;
    }
}

SMTBaseRouting::WeightLane* SMTBaseRouting::WeightLane::getNextLane(
        SMTEdge* toEdge) {
    map<SMTEdge*, WeightLane*>::iterator it = to->w2NextMap.find(toEdge);
    if (it == to->w2NextMap.end()) {
        std::cout << "no connection from lane " << con->fromLane << " of edge "
                << from->edge->id << " to " << toEdge->id << "." << std::endl;
    }
    return it->second;
}

SMTBaseRouting::WeightEdge::~WeightEdge() {
    for (map<SMTEdge*, WeightLane*>::iterator it = w2NextMap.begin();
            it != w2NextMap.end(); ++it) {
        delete it->second;
    }
}

SMTBaseRouting::CoRPUpdateBlock::~CoRPUpdateBlock() {
    if (rou != NULL) {
        delete rou;
        rou = NULL;
    }
}

void SMTBaseRouting::WeightLane::setCoRPOutInfo(SMTCarInfo* car,
        double laneTime, double viaTime, double outTime) {
    // 移动车辆进入时间至队列顶部(队列中车辆都是未离开车辆)
    // 更改离开时间,并添加后续影响至队列
    // 检索车辆
    multimap<SMTCarInfo*, HisInfo*>::iterator itCar = corpCarMap.find(car);
    ASSERT2(itCar != corpCarMap.end(),
            "try to set corp car from the lane does not contain it");
    // 保留当前车辆指针,用于后续路径操作
    HisInfo* hisInfo = itCar->second;
    // 验证hisInfo信息
    double _eTime = outTime - viaTime - laneTime;
    ASSERT2(
            hisInfo->enterTime < _eTime + 0.01
                    && hisInfo->enterTime > _eTime - 0.01,
            "unmatched enter time when set corp enter info");
    multimap<double, HisInfo*>::iterator itT = corpTimeMap.find(
            hisInfo->enterTime);
    while (itT != corpTimeMap.end()) {
        if (itT->second->car == car) {
            break;
        }
        ASSERT2(itT->first == hisInfo->enterTime,
                "no such car in this lane when update it.");
        ++itT;
    }
    // 保留当前itT用于删除元素
    multimap<double, HisInfo*>::iterator itTold = itT;
    // itT后移,用于指示下一个车辆
    ++itT;
    corpTimeMap.erase(itTold);
    // 仅仅做移动,不做删除
    // corpCarMap.erase(itCar);
    // 保存原始下一条道路进入时间
    double oldOutTime = hisInfo->outTime;
    double oldTau = hisInfo->tau;
    // 更新离开信息
    hisInfo->laneTime = laneTime;
    hisInfo->viaTime = viaTime;
    hisInfo->tau = hisInfo->enterTime + laneTime;
    hisInfo->outTime = outTime;
    // 将hisInfo插入队列头部
    if (firstCoRPInfo != NULL) {
        // 若修正tau之前,预测前方车辆与当前车辆连续通过,则可能更新corpEta
        if (firstCoRPInfo->tau + corpEta + 0.001 > oldTau) {
            double newEta = hisInfo->tau - firstCoRPInfo->tau;
#ifdef _FANJING_BASE_ROUTING_DEBUG
            double oldEta = corpCalcEta;
#endif
            if (con->tr == 0) {
                if (con->ty == 0) {
                    // 只有绿灯时仅对小于阈值的eta进行更新
                    if (newEta < 2.5) {
                        if (newEta > corpCalcEta) {
                            corpCalcEta = corpCalcEta
                                    + (newEta - corpCalcEta) * 0.125;
                        } else {
                            corpCalcEta = corpCalcEta
                                    + (newEta - corpCalcEta) * 0.5;
                        }
                    }
                } else {
                    // 没有红灯且有黄灯时的情况下总是对通过间隔进行更新
                    if (newEta > corpCalcEta) {
                        corpCalcEta = corpCalcEta
                                + (newEta - corpCalcEta) * 0.125;
                    } else {
                        corpCalcEta = corpCalcEta
                                + (newEta - corpCalcEta) * 0.5;
                    }
                }
            } else {
                if (newEta < 1.5) {
                    if (newEta > corpCalcEta) {
                        corpCalcEta = corpCalcEta
                                + (newEta - corpCalcEta) * 0.125;
                    } else {
                        corpCalcEta = corpCalcEta
                                + (newEta - corpCalcEta) * 0.25;
                    }
                }
                // 如果大于红等间隔则认为受红灯影响,不做更新
            }
            // 仅在变化大于0.1时更新eta
            if (corpCalcEta > corpEta + 0.1 || corpCalcEta < corpEta - 0.1) {
                corpEta = corpCalcEta;
#ifdef _FANJING_BASE_ROUTING_DEBUG
                if (debug) {
                    std::cout << "at time " << outTime << ", lane:" << from->edge->id
                    << "_" << con->fromLane << "corpEta:" << oldEta << "->"
                    << newEta << ":" << corpEta << std::endl;
                }
#endif
            }

#ifdef _FANJING_BASE_ROUTING_DEBUG
            if (debug) {
                std::cout << "corpEta:" << corpEta << std::endl;
            }
#endif
        }
        // 若存在已经离开道路的头结点,则移除并释放头结点
        corpTimeMap.erase(corpTimeMap.begin());
        corpCarMap.erase(firstCoRPInfo->car);
        delete firstCoRPInfo;
    }
    // 将hisInfo设为新的头结点
    firstCoRPInfo = hisInfo;
    itTold = corpTimeMap.insert(corpTimeMap.begin(),
            std::make_pair(-1, hisInfo));
    // 跟新后续影响判定条件
    hisInfo->nextDummyTime = hisInfo->tau + corpEta;
    double _fmod = std::fmod(hisInfo->nextDummyTime + con->_t0, con->ta);
    if (_fmod > con->tg) {
        hisInfo->nextDummyTime = hisInfo->nextDummyTime - _fmod + con->ta
                + corpEta;
        hisInfo->nFDT = hisInfo->nextDummyTime + corpEta;
    } else {
        hisInfo->nFDT = hisInfo->nextDummyTime + corpEta;
        _fmod = std::fmod(hisInfo->nFDT + con->_t0, con->ta);
        if (_fmod > con->tg) {
            hisInfo->nFDT = hisInfo->nFDT - _fmod + con->ta + corpEta;
        }
    }
    // 2nd. 添加后续道路更新和受影响车辆更新
    // 获取时间点最早的受影响车辆,并将其迭代器赋给itTold
    // 由于当前车辆为队列最早车辆,因此后续车辆肯定为itTold后方车辆
    ++itTold;
    // 添加后续道路更新
    CoRPUpdateBlock* block = NULL;
    if (hisInfo->next != NULL) {
        // 如果离开状态改变,则进行下一条道路的更新
        if (oldOutTime != hisInfo->outTime) {
            block = new CoRPUpdateBlock();
            block->fromTime = oldOutTime;
            block->toTime = hisInfo->outTime;
            block->timeStamp =
                    block->fromTime < block->toTime ?
                            block->fromTime : block->toTime;
            block->lane = hisInfo->next;
            block->car = car;
            // 修改车辆信息不需要设置route信息,因为对应hisInfo有next参数
            block->rou = NULL;
            block->srcHisInfo = hisInfo;
            corpUpdateQueue->insert(std::make_pair(block->timeStamp, block));
        }
    }
    // 受影响车辆更新
    // 构造out info更新消息
    if (itTold != corpTimeMap.end()) {
        hisInfo = itTold->second;
        // 设置受影响车辆outInfo更新消息
        block = new CoRPUpdateBlock();
        // 重设block为修改被影响车辆进入时间
        block->fromTime = hisInfo->enterTime;
        block->toTime = hisInfo->enterTime;
        block->timeStamp = hisInfo->enterTime;
        block->lane = this;
        block->car = hisInfo->car;
        // 修改车辆信息不需要设置route信息,因为对应hisInfo有next参数
        block->rou = NULL;
        block->srcHisInfo = hisInfo;
        corpUpdateQueue->insert(std::make_pair(block->timeStamp, block));
    }
}
