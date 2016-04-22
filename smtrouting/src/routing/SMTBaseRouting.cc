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
double SMTBaseRouting::WeightLane::limitFix = 0.69;

SMTBaseRouting::~SMTBaseRouting() {
    // 回收 dijkstra's algorithm 算法部分
    for (map<SMTEdge*, WeightEdge*>::iterator it = weightEdgeMap.begin();
            it != weightEdgeMap.end(); ++it) {
        delete (it->second);
    }
}

int SMTBaseRouting::numInitStages() const {
    return 2;
}

void SMTBaseRouting::initialize(int stage) {
    if (stage == 0) {
        suppressLength = par("suppressLength").doubleValue();
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
                // initialize occupation and occStep information
                wLane->occStep = 1 / it->first->length();
                wLane->to = weightEdgeMap[vIt->first];
                if (it->second->w2NextMap.find(vIt->first)
                        == it->second->w2NextMap.end()) {
                    it->second->w2NextMap[vIt->first] = wLane;
                } else {
                    std::cout << "system do not support multiple link for now"
                            << std::endl;
                }
            }
        }
    }
}

void SMTBaseRouting::handleMessage(cMessage* msg) {
}

void SMTBaseRouting::finish() {
}

SMTMap* SMTBaseRouting::getMap() {
    if (_pMap == NULL) {
        _pMap = SMTMapAccess().get();
    }
    return _pMap;
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
        std::cout << "dead end" << std::endl;
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

//@Removed function
void SMTBaseRouting::updatePassTime(SMTEdge* from, SMTEdge* to, double w,
        double curTime, SMTCarInfo* car) {
    // FIXME removed function
//    WeightEdge* wEdge = weightEdgeMap[from];
//    wEdge->w2NextMap[to]->recentCost = w;
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

void SMTBaseRouting::getDijkstralResult(SMTEdge* destination,
        list<SMTEdge*>& route) {
    WeightEdge* wEdge = weightEdgeMap[destination];
    while (wEdge != NULL) {
        route.push_front(wEdge->edge);
        wEdge = wEdge->previous;
    }
}

void SMTBaseRouting::changeRoad(SMTEdge* from, SMTEdge* to, int toLane,
        double time, SMTCarInfo* car) {
    // update pass time and remove car from weightEdge 'from'
    if (from != NULL) {
        map<SMTEdge*, WeightEdge*>::iterator itFromEdge = weightEdgeMap.find(
                from);
        map<SMTEdge*, WeightLane*>::iterator itFromLane =
                itFromEdge->second->w2NextMap.find(to);
        itFromLane->second->removeCar(car, time);
    }
    // add car into weightEdge 'to'
    if (toLane != -1) {
        map<SMTEdge*, WeightEdge*>::iterator itToEdge = weightEdgeMap.find(to);
        if (itToEdge->first->laneVector[toLane]->nextVector.size() > 1) {
            std::cout << "system does not support multiple link for now@"
                    << itToEdge->first->laneVector[toLane]->id << std::endl;
        }
        SMTEdge* next = itToEdge->first->laneVector[toLane]->nextVector[0]->edge;
        map<SMTEdge*, WeightLane*>::iterator itToLane =
                itToEdge->second->w2NextMap.find(next);
        itToLane->second->insertCar(car, time);
    }
}

double SMTBaseRouting::modifyWeightFromEdgeToEdge(WeightEdge* from,
        WeightEdge* to) {
    // w means the delta weight from wEdge to next
    double deltaW = -1;
    if (to == NULL) {
        std::cout << "processDijkstralNeighbors:" << "No edge " << to->edge->id
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
        {
            // fix deltaW by occupation if occupation is bigger than half
            // fix deltaW only when cars in this lane cannot pass in one green time
            // and the occupation reach the limit
            if (itWL->second->occupation > WeightLane::limitStart
                    && itWL->second->occupation / itWL->second->occStep
                            > 5 * 20) {
                if (itWL->second->occupaChangeFlag) {
                    itWL->second->occupaChangeFlag = false;
                    std::cout << "occupation from " << from->edge->id << " to "
                            << to->edge->id << " is "
                            << itWL->second->occupation << std::endl;
                }
                if (itWL->second->occupation < WeightLane::limitFix) {
                    deltaW = deltaW
                            / (WeightLane::limitCap - itWL->second->occupation);
                } else {
                    deltaW = deltaW * 10000;
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
    case SMT_RT_CORP:
        // TODO add cooperative route plan method
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

void SMTBaseRouting::WeightLane::insertCar(SMTCarInfo* car, double t) {
    ASSERT2(carMap.find(car) == carMap.end(),
            "car has been already in this lane");
    carMap[car] = t;
    enterTimeMap.insert(std::make_pair(t, car));
    // update occupation information
    occupation += occStep * (car->length + car->minGap);
    occupaChangeFlag = true;
}

double SMTBaseRouting::WeightLane::getCost(double time) {
    updateCost(time);
    return recentCost;
}

void SMTBaseRouting::WeightLane::updateCost(double time) {
    // update outed car map when new car out or time pass
    if (time > recentCostLastupdateTime || recentCostRefreshFlag) {
        // remove invalid outed car
        for (multimap<double, CarTime>::iterator it = recentOutCars.begin();
                it != recentOutCars.end(); ++it) {
            // keep at least three cars
            if (it->first < time - outCarKeepDuration
                    && recentOutCars.size() > 3) {
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
        recentCost = totalRecentCost / recentOutCars.size();
        recentCostLastupdateTime = time;
        recentCostRefreshFlag = false;
    }
}

void SMTBaseRouting::WeightLane::carGetOut(SMTCarInfo* car, const double& t,
        const double& cost) {
    recentOutCars.insert(make_pair(t, CarTime(car, cost)));
    totalRecentCost += cost;
    recentCostRefreshFlag = true;
}

SMTBaseRouting::WeightLane::~WeightLane() {
    // do nothing
}

void SMTBaseRouting::WeightLane::removeCar(SMTCarInfo* car, double t) {
    map<SMTCarInfo*, double>::iterator itCar = carMap.find(car);
    multimap<double, SMTCarInfo*>::iterator itT = enterTimeMap.find(
            itCar->second);
    while (itT->second != car) {
        ++itT;
        ASSERT2(itT->first != itCar->second, "try to remove inexistent car");
    }
    carGetOut(car, t, t - itCar->second);
    enterTimeMap.erase(itT);
    carMap.erase(itCar);
    // update occupation information
    occupation -= occStep * (car->length + car->minGap);
    occupaChangeFlag = true;
}

SMTBaseRouting::WeightEdge::~WeightEdge() {
    for (map<SMTEdge*, WeightLane*>::iterator it = w2NextMap.begin();
            it != w2NextMap.end(); ++it) {
        delete it->second;
    }
}
