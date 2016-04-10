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

    // TODO 添加选路过程
    return rou;
}

void SMTBaseRouting::initDijkstra() {
    // 1. reset weightEdge
    for (map<SMTEdge*, WeightEdge*>::iterator it = weightEdgeMap.begin();
            it != weightEdgeMap.end(); ++it) {
        it->second->previous = NULL;
        it->second->w = -1;
    }
    // 2. init unSet, outSet, processMap
    unSet = weightEdgeMap;
    outSet.clear();
    processMap.clear();
}

void SMTBaseRouting::changeDijkstraWeight(SMTEdge* from, SMTEdge* to,
        double w) {
    WeightEdge * wEdge;
    map<SMTEdge*, WeightEdge*>::iterator it_to = weightEdgeMap.find(to);
    if (it_to != weightEdgeMap.end()) {
        wEdge = it_to->second;
    }
    if (wEdge->w != -1 && wEdge->w <= w) {
        // do not change wEdge if the old weight is smaller
        return;
    }
    if (wEdge->w > w) {
        // remove wEdge from processMap
        for (multimap<double, WeightEdge*>::iterator it = processMap.find(
                wEdge->w); it->first == wEdge->w; ++it) {
            if (it->second == wEdge) {
                processMap.erase(it);
                break;
            }
        }
    }
    if (wEdge->w == -1) {
        // if wEdge->w == -1, it is an untouched edge.
        // move it out of unSet before modify processMap
        unSet.erase(wEdge->edge);
    }
    wEdge->previous = from;
    wEdge->w = w;
    processMap.insert(std::make_pair(w, wEdge));
}

void SMTBaseRouting::processDijkstraLoop() {
    // get WeightEdge with smallest w, and change its neighbors.
    // TODO
}
