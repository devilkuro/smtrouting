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
    if (stage == 1) {
        // needs to init weightEdgeMap here
        // to make sure Map has initialized before
        // set numInitStages=2 and init weightEdgeMap at stage 1
        // set weightEdgeMap for dijkstra's algorithm
        for (set<SMTEdge*>::iterator it = getMap()->primaryEdgeSet.begin();
                it != getMap()->primaryEdgeSet.end(); ++it) {
            weightEdgeMap[*it] = new WeightEdge(*it);
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

list<SMTEdge*> SMTBaseRouting::getShortestRoute(SMTEdge* origin,
        SMTEdge* destination) {
    // 最短路径使用迪杰斯特拉算法
    multimap<double, Route> costMap;
    list<SMTEdge*> rou;
    runDijkstraAlgorithm(origin, destination, rou);
    // TODO 添加选路过程
    return rou;
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
    }
    // 2. init unSet, outSet, processMap
    // unSet = weightEdgeMap;
    // outSet.clear();
    processMap.clear();
    // insert origin edge into processMap
    processMap.insert(std::make_pair(origin->length(), ori_it->second));
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
        // move it out of unSet before modify processMap
        // FIXME unSet may be useless
        // unSet.erase(to->edge);
    }
    to->previous = from;
    to->w = w;
    processMap.insert(std::make_pair(w, wEdge));
}

int SMTBaseRouting::processDijkstraLoop(SMTEdge* destination) {
    SMTEdge* curEdge;
    do {
        curEdge = processDijkstralNode(destination);
    } while (curEdge != destination && curEdge != NULL);
    if (curEdge == NULL) {
        return -1;
    }
    return 0;
}

void SMTBaseRouting::runDijkstraAlgorithm(SMTEdge* origin, SMTEdge* destination,
        list<SMTEdge*> &route) {
    initDijkstra(origin);
    processDijkstraLoop(destination);
    getDijkstralResult(origin, route);
    // TODO needs test
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
        return NULL;
    }
    WeightEdge* wEdge = processMap.begin()->second;
    if (wEdge->edge == destination) {
        return destination;
    } else {
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
        // w means the delta weight from wEdge to next
        double w = -1;
        if (next == NULL) {
            std::cout << "processDijkstralNeighbors:" << "No edge "
                    << next->edge->id << " in weightEdgeMap" << std::endl;
        }
        for (unsigned int i = 0; i < it->second.size(); ++i) {
            double viaLen = it->second[i]->getViaLength();
            if (w == -1) {
                w = viaLen;
            } else {
                w = w > viaLen ? viaLen : w;
            }
        }
        w += wEdge->edge->length();
        if (w < 0) {
            std::cout << "processDijkstralNeighbors:"
                    << "cannot handle negative cost from" << wEdge->edge->id
                    << " to " << next->edge->id << std::endl;
        }
        changeDijkstraWeight(wEdge, next, w + wEdge->w);
    }
}

void SMTBaseRouting::getDijkstralResult(SMTEdge* destination,
        list<SMTEdge*>& route) {
    WeightEdge* wEdge = weightEdgeMap[destination];
    while (wEdge != NULL) {
        route.push_front(wEdge->edge);
        wEdge = wEdge->previous;
    }
}
