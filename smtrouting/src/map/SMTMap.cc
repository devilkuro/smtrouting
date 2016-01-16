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

#include "SMTMap.h"
#include "StatisticsRecordTools.h"

Define_Module(SMTMap);

SMTEdge::~SMTEdge() {
    // TODO 释放资源
    for (unsigned int i = 0; i < conVector.size(); i++) {
        delete (conVector[i]);
    }
}

SMTLane::~SMTLane() {
    // TODO 释放资源
}

SMTConnection::~SMTConnection() {
    // TODO 释放资源
}

SMTRoute::~SMTRoute() {
    // TODO 释放资源
}

double SMTRoute::getViaLength() {
    // TODO 获取via路径长度
    return 0;
}
SMTMap::~SMTMap() {
    // release edgeMap
    // 释放SMTEdge的同时会释放相关的SMTConnection内存
    for (map<string, SMTEdge*>::iterator it = edgeMap.begin();
            it != edgeMap.end(); it++) {
        delete (it->second);
    }
    // release laneMap
    for (map<string, SMTLane*>::iterator it = laneMap.begin();
            it != laneMap.end(); it++) {
        delete (it->second);
    }
}

SMTLaunchd* SMTMap::getLaunchd() {
    if (NULL == launchd) {
        launchd = SMTLaunchdAccess().get();
    }
    return launchd;
}

SMTEdge* SMTMap::getSMTEdge(string id) {
    ASSERT2(edgeMap.find(id)!=edgeMap.end(),"try to get SMTEdge from an unknown id.");
    return edgeMap[id];
}

void SMTMap::initialize() {
    debug = hasPar("debug") ? par("debug") : false;
    rouXML = par("rouXML").xmlValue();
    netXML = par("netXML").xmlValue();

    initNetFromXML(netXML);
    // TODO optimizeNet
    optimizeNet();
    hasInitialized = true;
    stepMsg = new cMessage("step message of SMTMap");
    scheduleAt(simTime() + 0.1, stepMsg);
}

void SMTMap::handleMessage(cMessage *msg) {
    if (msg == stepMsg) {
        if (getLaunchd()->isConnected()) {
            if (debug) {
                verifyNetConfig();
            }
        } else {
            scheduleAt(simTime() + 0.1, stepMsg);
        }
    }
}

void SMTMap::initNetFromXML(cXMLElement* xml) {
    // TODO 导入道路与连接
    // import edges and lanes
    cXMLElement* edgeXML = xml->getFirstChildWithTag("edge");
    while (edgeXML) {
        addEdgeFromEdgeXML(edgeXML);
        edgeXML = edgeXML->getNextSiblingWithTag("edge");
    }
    // import connections
    cXMLElement* conXML = xml->getFirstChildWithTag("connection");
    while (conXML) {
        addConFromConXML(conXML);
        conXML = conXML->getNextSiblingWithTag("connection");
    }
}

void SMTMap::optimizeNet() {
    // TODO 优化网络,生成易于寻路的参数
}

void SMTMap::initVechileTypeFromXML(cXMLElement* xml) {
    // TODO 导入车辆类型
}

void SMTMap::addEdgeFromEdgeXML(cXMLElement* xml) {
    // 读取并添加edge和lane
    SMTEdge* edge = new SMTEdge();
    cXMLAttributeMap attrMap = xml->getAttributes();
    edge->id = attrMap["id"];
    edge->function = attrMap["function"];
    if (edge->function == "internal") {
        edge->isInternal = true;
    } else {
        edge->from = attrMap["from"];
        edge->to = attrMap["to"];
        edge->priority = atoi(attrMap["priority"].c_str());
    }
    // get the lanes in this edge
    cXMLElement* laneXML = xml->getFirstChildWithTag("lane");
    map<int, SMTLane*> tempLaneMap;
    while (laneXML) {
        SMTLane* lane = new SMTLane();
        cXMLAttributeMap laneAttrMap = laneXML->getAttributes();
        lane->id = laneAttrMap["id"];
        lane->index = strtol(laneAttrMap["index"].c_str(), 0, 0);
        lane->speed = strtod(laneAttrMap["speed"].c_str(), 0);
        lane->length = strtod(laneAttrMap["length"].c_str(), 0);
        lane->edge = edge;
        if (tempLaneMap.find(lane->index) != tempLaneMap.end()) {
            std::cout
                    << "Warning@SMTMap::insertEdgeFromEdgeXML-duplicated lane index"
                    << std::endl;
            delete lane;
        } else {
            ASSERT2(laneMap.find(lane->id) == laneMap.end(),
                    "duplicated lane id");
            laneMap[lane->id] = lane;
            tempLaneMap[lane->index] = lane;
            if (debug) {
                std::cout << "add lane '" << lane->id << "' to map."
                        << std::endl;
            }

        }
        laneXML = laneXML->getNextSiblingWithTag("lane");
    }
    // set the vector
    for (int i = 0; i < (int) tempLaneMap.size(); i++) {
        ASSERT2(tempLaneMap.find(i) != tempLaneMap.end(), "missing lane index");
        edge->laneVector.push_back(tempLaneMap[i]);
    }
    if (edgeMap.find(edge->id) == edgeMap.end()) {
        edgeMap[edge->id] = edge;
        if (debug) {
            std::cout << "add edge '" << edge->id << "' to map." << std::endl;
        }
    } else {
        std::cout << "Warning@SMTMap::insertEdgeFromEdgeXML-duplicated edge"
                << std::endl;
        delete edge;
    }
}

void SMTMap::addConFromConXML(cXMLElement* xml) {
    // 读取并添加connection
    // 读取属性
    SMTConnection* con = new SMTConnection();
    cXMLAttributeMap attrMap = xml->getAttributes();
    con->from = attrMap["from"];
    con->to = attrMap["to"];
    con->fromLane = strtol(attrMap["fromLane"].c_str(), 0, 0);
    con->toLane = strtol(attrMap["toLane"].c_str(), 0, 0);
    con->via = attrMap["via"];
    con->linkIndex = strtol(attrMap["linkIndex"].c_str(), 0, 0);
    con->dir = attrMap["dir"];
    // SMT 属性
    ASSERT2(edgeMap.find(con->from) != edgeMap.end(), "unknown from edge.");
    con->fromSMTEdge = edgeMap[con->from];
    ASSERT2(edgeMap.find(con->to) != edgeMap.end(), "unknown to edge.");
    con->toSMTEdge = edgeMap[con->to];
    con->fromSMTLane = edgeMap[con->from]->laneVector[con->fromLane];
    con->toSMTLane = edgeMap[con->to]->laneVector[con->toLane];
    if (laneMap.find(con->via) != laneMap.end()) {
        con->viaSMTLane = laneMap[con->via];
    }

    // 处理关联
    if (debug) {
        std::cout << "add connection between '" << con->from << "' and '"
                << con->to << "' via '" << con->via << "'." << std::endl;
    }
    bool hasConnected = false;
    for (int i = 0; i < (int) con->fromSMTLane->nextVector.size(); i++) {
        if (con->fromSMTLane->nextVector[i] == con->toSMTLane) {
            hasConnected = true;
        }
    }
    if (hasConnected) {
        std::cout
                << "Warning@SMTMap::insertEdgeFromEdgeXML-duplicated connection"
                << std::endl;
        delete (con);
    } else {
        con->fromSMTEdge->conVector.push_back(con);
        con->fromSMTLane->conVector.push_back(con);
        con->fromSMTLane->nextVector.push_back(con->toSMTLane);
    }
}

void SMTMap::verifyNetConfig() {
    SMTComInterface* comIfc = getLaunchd()->getSMTComInterface();

    // verify the edges and lanes.
    list<string> laneList = comIfc->getLaneIds();
    if (laneList.size() != laneMap.size()) {
        std::cout << "lane number is mismatch." << std::endl;
    }
    for (list<string>::iterator it = laneList.begin(); it != laneList.end();
            it++) {
        if (laneMap.find(*it) == laneMap.end()) {
            std::cout << "lane '" << (*it) << "' is missing." << std::endl;
        }
        string edge = comIfc->getLaneEdgeId(*it);
        if (edgeMap.find(edge) == edgeMap.end()) {
            std::cout << "edge '" << (*it) << "' is missing." << std::endl;
        }
    }
    set<string> laneSet = set<string>(laneList.begin(), laneList.end());
    for (map<string, SMTLane*>::iterator it = laneMap.begin();
            it != laneMap.end(); it++) {
        if (laneSet.find(it->first) == laneSet.end()) {
            std::cout << "lane '" << (it->first) << "' is redundant."
                    << std::endl;
        }
        string edge = comIfc->getLaneEdgeId(it->first);
        if (edgeMap.find(edge) == edgeMap.end()) {
            std::cout << "edge '" << (it->first) << "' is redundant."
                    << std::endl;
        }
    }
    std::cout << "verifying lanes and edges finished." << std::endl;
    // vrtify the connections
    for (list<string>::iterator it = laneList.begin(); it != laneList.end();
            it++) {
        list<string> laneLinkedLanes = comIfc->getLaneLinkedLaneIds(*it);
        set<string> laneLinkedSet;
        for (int i = 0; i < (int) laneMap[*it]->nextVector.size(); i++) {
            laneLinkedSet.insert(laneMap[*it]->nextVector[i]->id);
        }
        if (laneLinkedLanes.size() != laneMap[*it]->nextVector.size()) {
            std::cout << "lane linked lane number is mismatch." << std::endl;
        }
        for (list<string>::iterator linkedIt = laneLinkedLanes.begin();
                linkedIt != laneLinkedLanes.end(); linkedIt++) {
            if (laneLinkedSet.find(*linkedIt) == laneLinkedSet.end()) {
                std::cout << "lane '" << (*it) << "' linked lane '"
                        << (*linkedIt) << "' is missing." << std::endl;
            } else {
                laneLinkedSet.erase(*linkedIt);
            }
        }
        for (set<string>::iterator linkedIt = laneLinkedSet.begin();
                linkedIt != laneLinkedSet.end(); linkedIt++) {
            std::cout << "lane '" << (*it) << "' linked lane '" << (*linkedIt)
                    << "' is redundant." << std::endl;
        }
    }
    std::cout << "verifying connections finished." << std::endl;
}

void SMTMap::finish() {
    Fanjing::StatisticsRecordTools *srt = Fanjing::StatisticsRecordTools::request();
    srt->outputSeparate("trajectory.txt","./results");
    srt->clean();
    std::cout<<"Map::finish"<<std::endl;
}
