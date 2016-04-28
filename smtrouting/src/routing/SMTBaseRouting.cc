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

Define_Module(SMTBaseRouting);
double SMTBaseRouting::WeightLane::outCarKeepDuration = 120;
double SMTBaseRouting::WeightLane::limitStart = 0.1;
double SMTBaseRouting::WeightLane::limitCap = 0.7;
double SMTBaseRouting::WeightLane::limitFix = 0.01;
double SMTBaseRouting::WeightLane::airK = 0.2287;
double SMTBaseRouting::WeightLane::airV = 0.6352;

SMTBaseRouting::~SMTBaseRouting() {
    // 回收 dijkstra's algorithm 算法部分
    for (map<SMTEdge*, WeightEdge*>::iterator it = weightEdgeMap.begin();
            it != weightEdgeMap.end(); ++it) {
        delete (it->second);
    }
}

int SMTBaseRouting::numInitStages() const {
    return 3;
}

void SMTBaseRouting::initialize(int stage) {
    if (stage == 0) {
        debug = par("debug").boolValue();
        suppressLength = par("suppressLength").doubleValue();
        WeightLane::limitStart = par("limitStart").doubleValue();
        WeightLane::limitFix = par("limitFix").doubleValue();
        WeightLane::limitCap = par("limitCap").doubleValue();
        WeightLane::airK = par("airK").doubleValue();
        WeightLane::airV = par("airV").doubleValue();
        majorRoutingType =
                (enum SMT_ROUTING_TYPE) par("majorRoutingType").longValue();
        minorRoutingType =
                (enum SMT_ROUTING_TYPE) par("majorRoutingType").longValue();
        recordHisRecordRoutingType =
                par("recordHisRecordRoutingType").longValue();
        if (majorRoutingType == SMT_RT_AIR || minorRoutingType == SMT_RT_AIR) {
            enableAIR = true;
        } else {
            enableAIR = false;
        }
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
        srt = Fanjing::StatisticsRecordTools::request();
        rouState.recordActiveCarNum = par("recordActiveCarNum").boolValue();
        rouState.recordActiveCarInterval =
                par("recordActiveCarInterval").doubleValue();
        recordXMLPrefix = par("recordXMLPrefix").stringValue();
        recordHisRoutingData = par("recordHisRoutingData").boolValue();
        if (majorRoutingType == recordHisRecordRoutingType) {
            recordHisRoutingData = true;
        }
        hisRecordXMLPath = par("hisRecordXMLPath").stringValue();
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
                wLane->viaLen = vIt->second[0]->getViaLength();
                // initialize occupation and occStep information
                wLane->occStep = 1 / it->first->length();
                wLane->from = it->second;
                wLane->to = weightEdgeMap[vIt->first];
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
        statisticMsg = new cMessage("statisticMsg)");
        scheduleAt(simTime() + rouState.recordActiveCarInterval, statisticMsg);
    }
    if (stage == 2) {
        // load historical xml after car map has been initialized
        if (enableCoRP) {
            importHisXML();
            // TODO end simulation
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
        } else if (msg == airUpdateMsg) {
            updateAIRInfo();
        } else if (msg == statisticMsg) {
            scheduleAt(simTime() + rouState.recordActiveCarInterval,
                    statisticMsg);
            updateStatisticInfo();
        }
    }
}

void SMTBaseRouting::finish() {
    printStatisticInfo();
    srt->outputSeparate(recordXMLPrefix + ".txt");
    if (recordHisRoutingData) {
        exportHisXML();
    }
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
    // TODO needs test
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
        double w) {
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

void SMTBaseRouting::getRouteByMajorMethod(SMTEdge* origin,
        SMTEdge* destination, list<SMTEdge*>& rou, double time,
        SMTCarInfo* car) {
    switch (majorRoutingType) {
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
    default:
        getFastestRoute(origin, destination, rou, time, car);
        break;
    }
}

void SMTBaseRouting::getRouteByMinorMethod(SMTEdge* origin,
        SMTEdge* destination, list<SMTEdge*>& rou, double time,
        SMTCarInfo* car) {
    switch (minorRoutingType) {
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
    default:
        getFastestRoute(origin, destination, rou, time, car);
        break;
    }
}

void SMTBaseRouting::printStatisticInfo() {
    for (map<SMTEdge*, WeightEdge*>::iterator itWE = weightEdgeMap.begin();
            itWE != weightEdgeMap.end(); ++itWE) {
        for (map<SMTEdge*, WeightLane*>::iterator itWL =
                itWE->second->w2NextMap.begin();
                itWL != itWE->second->w2NextMap.end(); ++itWL) {
            if (itWL->second->statistic.passedCarNum > 0) {
                std::cout << "via from edge " << itWE->first->id << " to "
                        << itWL->first->id << std::endl;
                std::cout << "Avg speed: "
                        << itWL->second->viaLen
                                * itWL->second->statistic.passedCarNum
                                / itWL->second->statistic.totalViaPassTime
                        << ", Max Speed: "
                        << itWL->second->viaLen
                                / itWL->second->statistic.minViaPassTime
                        << ", Min Speed: "
                        << itWL->second->viaLen
                                / itWL->second->statistic.maxViaPassTime
                        << ", Max Lane Speed: "
                        << itWL->second->from->edge->length()
                                / itWL->second->statistic.minLanePassTime
                        << ", car number: "
                        << itWL->second->statistic.passedCarNum << std::endl;
            }
        }
    }
    std::cout << "arrivedCarCount:" << rouState.arrivedCarCount << std::endl;
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
    static string titleTime = "time";
    static string titleActiveCarCount = titleTime + "\t" + "carCount";
    if (rouState.recordActiveCarNum) {
        srt->changeName("activeCarCount", titleActiveCarCount)
                << simTime().dbl()
                << getMap()->getLaunchd()->getActiveVehicleCount() << srt->endl;
    }
    static string titleTTS = titleTime + "\t" + "TTS";
    srt->changeName("TTS", titleTTS) << simTime().dbl()
            << getMap()->getLaunchd()->getTTS() << srt->endl;
    static string titleArrivedCarCount = titleTime + "\t" + "arrivedCarCount";
    srt->changeName("arrivedCarCount", titleArrivedCarCount) << simTime().dbl()
            << rouState.arrivedCarCount << srt->endl;
}

void SMTBaseRouting::exportHisXML() {
    XMLDocument* doc = new XMLDocument();
    XMLDeclaration* dec = doc->NewDeclaration();
    doc->LinkEndChild(dec);
    XMLComment* comment = doc->NewComment("et=enterTime;lt=laneTime;vt=viaTime;it=intervalToLast");
    doc->LinkEndChild(comment);
    XMLElement* fromEdgeElm;
    XMLElement* toEdgeElm;
    XMLElement* carElm;
    for (map<SMTEdge*, WeightEdge*>::iterator itWE = weightEdgeMap.begin();
            itWE != weightEdgeMap.end(); ++itWE) {
        fromEdgeElm = doc->NewElement("From");
        fromEdgeElm->SetAttribute("id", itWE->second->edge->id.c_str());
        for (map<SMTEdge*, WeightLane*>::iterator itWL =
                itWE->second->w2NextMap.begin();
                itWL != itWE->second->w2NextMap.end(); ++itWL) {
            toEdgeElm = doc->NewElement("To");
            toEdgeElm->SetAttribute("id",
                    itWL->second->to->edge->id.c_str());
            for (map<double, WeightLane::HisInfo*>::iterator itHis =
                    itWL->second->hisTimeMap.begin();
                    itHis != itWL->second->hisTimeMap.end(); ++itHis) {
                carElm = doc->NewElement("CAR");
                carElm->SetAttribute("id", itHis->second->car->id.c_str());
                carElm->SetAttribute("et",
                        Fanjing::StringHelper::dbl2str(itHis->second->time, 1).c_str());
                if (itHis->second->next != NULL) {
                    carElm->SetAttribute("next",
                            itHis->second->next->from->edge->id.c_str());
                } else {
                    carElm->SetAttribute("next", "");
                }
                carElm->SetAttribute("lt",
                        Fanjing::StringHelper::dbl2str(itHis->second->laneTime,
                                1).c_str());
                carElm->SetAttribute("vt",
                        Fanjing::StringHelper::dbl2str(itHis->second->viaTime,
                                1).c_str());
                carElm->SetAttribute("it",
                        Fanjing::StringHelper::dbl2str(
                                itHis->second->intervalToLast, 1).c_str());
                toEdgeElm->LinkEndChild(carElm);
            }
            fromEdgeElm->LinkEndChild(toEdgeElm);
        }
        doc->LinkEndChild(fromEdgeElm);
    }
    doc->SaveFile(hisRecordXMLPath.c_str());
    doc->Clear();
}

void SMTBaseRouting::importHisXML() {
    XMLDocument* doc = new XMLDocument();
    doc->LoadFile(hisRecordXMLPath.c_str());

    XMLElement* fromEdgeElm;
    XMLElement* toEdgeElm;
    XMLElement* carElm;
    fromEdgeElm = doc->FirstChildElement("From");
    WeightEdge* fromWEdge = weightEdgeMap[getMap()->getSMTEdgeById(
            fromEdgeElm->Attribute("id"))];
    while (fromEdgeElm != NULL) {
        toEdgeElm = fromEdgeElm->FirstChildElement("To");
        WeightLane* toWLane = fromWEdge->w2NextMap[getMap()->getSMTEdgeById(
                toEdgeElm->Attribute("id"))];
        while (toEdgeElm != NULL) {
            carElm = toEdgeElm->FirstChildElement("CAR");
            while (carElm != NULL) {
                /*
                 carElm->SetAttribute("id", itHis->second->car->id.c_str());
                 carElm->SetAttribute("enterTime", itHis->second->time);
                 carElm->SetAttribute("next",
                 itHis->second->next->from->edge->id.c_str());
                 carElm->SetAttribute("laneTime", itHis->second->laneTime);
                 carElm->SetAttribute("viaTime", itHis->second->viaTime);
                 carElm->SetAttribute("intervalToLast",
                 itHis->second->intervalToLast);
                 */
                WeightLane::HisInfo* hisInfo = new WeightLane::HisInfo();
                hisInfo->car = getCarManager()->carMapByID[carElm->Attribute(
                        "id")];
                hisInfo->time = carElm->DoubleAttribute("et");
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
                hisInfo->intervalToLast = carElm->DoubleAttribute(
                        "it");
                toWLane->addHistoricalCar(hisInfo->car, hisInfo->time);
                carElm = carElm->NextSiblingElement("CAR");
            }
            toEdgeElm = toEdgeElm->FirstChildElement("To");
        }
        fromEdgeElm = fromEdgeElm->NextSiblingElement("From");
    }
}

void SMTBaseRouting::getDijkstralResult(SMTEdge* destination,
        list<SMTEdge*>& route) {
    WeightEdge* wEdge = weightEdgeMap[destination];
    // before operation
    if (routeType == SMT_RT_CORP_SELF) {

    }
    while (wEdge != NULL) {
        route.push_front(wEdge->edge);
        wEdge = wEdge->previous;
    }
}

void SMTBaseRouting::changeRoad(SMTEdge* from, SMTEdge* to, int toLaneIndex,
        double time, SMTCarInfo* car, double viaTime, double laneTime) {
    // update pass time and remove car from weightEdge 'from'
    WeightLane* fromLane = NULL;
    WeightLane* toLane = NULL;
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
        rouState.arrivedCarCount++;
    }
    if (recordHisRoutingData) {
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
        // FIXME since w2NextMap is initialized in initialize()
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
                    if (itWL->second->occupaChangeFlag) {
                        itWL->second->occupaChangeFlag = false;
                        std::cout << "occupation from " << from->edge->id
                                << " to " << to->edge->id << " is "
                                << itWL->second->occupation << std::endl;
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
                        if (itWL->second->occupaChangeFlag) {
                            itWL->second->occupaChangeFlag = false;
                            std::cout << "occupation from " << from->edge->id
                                    << " to " << to->edge->id << " is "
                                    << itWL->second->occupation << std::endl;
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
        // TODO add cooperative route plan method
        if (deltaW < 0) {
            std::cout << "processDijkstralNeighbors:"
                    << "cannot handle negative via cost from" << from->edge->id
                    << " to " << to->edge->id << std::endl;
        }
        changeDijkstraWeight(from, to, deltaW + from->w);
        break;
    case SMT_RT_CORP_TTS:
        itWL = from->w2NextMap.find(to->edge);
        // since w2NextMap is initialized in initialize()
        // itWL will never equal to from->w2NextMap.end()
        ASSERT2(itWL != from->w2NextMap.end(),
                "w2NextMap initialized abnormally");
        // TODO add cooperative route plan method
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
}

void SMTBaseRouting::WeightLane::carGetOut(SMTCarInfo* car, const double& t,
        const double& cost) {
    recentOutCars.insert(make_pair(t, CarTime(car, cost)));
    totalRecentCost += cost;
    recentCostRefreshFlag = true;
}

void SMTBaseRouting::WeightLane::initMinAllowedCost() {
    SMTConnection* con = via->start->laneVector[via->fromLane]->conVector[0];
    minAllowedCost = ((con->tr + con->ty / 2) * (con->tr + con->ty / 2) / 2)
            / (con->tr + con->tg + con->ty)
            + con->fromSMTEdge->length()
                    / via->start->laneVector[via->fromLane]->speed;
}

void SMTBaseRouting::WeightLane::carPassVia(double time) {
    if (statistic.maxViaPassTime < time) {
        statistic.maxViaPassTime = time;
    }
    if (statistic.minViaPassTime > time || statistic.minViaPassTime < 0) {
        statistic.minViaPassTime = time;
    }
    ++statistic.passedCarNum;
    statistic.totalViaPassTime += time;
}

void SMTBaseRouting::WeightLane::carPassLane(double time) {
    if (statistic.maxLanePassTime < time) {
        statistic.maxLanePassTime = time;
    }
    if (statistic.minLanePassTime > time || statistic.minLanePassTime < 0) {
        statistic.minLanePassTime = time;
    }
    statistic.totalLanePassTime += time;
}

void SMTBaseRouting::WeightLane::insertCar(SMTCarInfo* car, double t) {
    ASSERT2(carMap.find(car) == carMap.end(),
            "car has been already in this lane");
    carMap[car] = t;
    enterTimeMap.insert(std::make_pair(t, car));
    // update occupation information
    occupation += occStep * (car->length + car->minGap);
    occupaChangeFlag = true;
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
    occupaChangeFlag = true;
    airCostUpdateFlag = true;
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
    hisInfo->time = t;
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
    it->second->intervalToLast = time - lastCarOutTime;
    lastCarOutTime = time;
    it->second->next = next;
}

void SMTBaseRouting::WeightLane::removeHistoricalCar(SMTCarInfo* car,
        double t) {
    map<SMTCarInfo*, HisInfo*>::iterator itCar = hisCarMap.find(car);
    multimap<double, HisInfo*>::iterator itT = hisTimeMap.find(
            itCar->second->time);
    while (itT->second->car != car) {
        ++itT;
        if (itT->first != itCar->second->time) {
            std::cout << "try to remove inexistent car in hisTimeMap"
                    << itCar->first->id << ", but find car "
                    << itT->second->car->id << std::endl;
        }
    }
    hisTimeMap.erase(itT);
    delete (itCar->second);
    hisCarMap.erase(itCar);
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
            } else if (recentCost < time - enterTimeMap.begin()->first) {
                recentCost = time - enterTimeMap.begin()->first;
            }
        }
        recentCost = recentCost > minAllowedCost ? recentCost : minAllowedCost;
        recentCostLastupdateTime = time;
        recentCostRefreshFlag = false;
    }
}

SMTBaseRouting::WeightEdge::~WeightEdge() {
    for (map<SMTEdge*, WeightLane*>::iterator it = w2NextMap.begin();
            it != w2NextMap.end(); ++it) {
        delete it->second;
    }
}

