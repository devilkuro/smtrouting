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

Define_Module(SMTMap);

SMTEdge::~SMTEdge() {
    // TODO 释放资源
}

SMTLane::~SMTLane() {
    // TODO 释放资源
}

SMTConnection::~SMTConnection() {
    // TODO 释放资源
}

SMTMap::~SMTMap() {
    // TODO 释放资源
}

SMTLaunchd* SMTMap::getLaunchd() {
    if (NULL == launchd) {
        launchd = SMTLaunchdAccess().get();
    }
    return launchd;
}
void SMTMap::initialize() {
    debug = hasPar("debug") ? par("debug") : false;
    rouXML = par("rouXML").xmlValue();
    netXML = par("netXML").xmlValue();

    initNetFromXML(netXML);
    // TODO optimizeNet
    stepMsg = new cMessage("step message of SMTMap");
    scheduleAt(simTime() + 0.1, stepMsg);
}

void SMTMap::handleMessage(cMessage *msg) {
    if (msg == stepMsg) {
        if (getLaunchd()->isConnected()) {
            if(debug){
                verfyNetConfig();
            }
        }else{
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
                    << "Error@SMTMap::insertEdgeFromEdgeXML-duplicated lane index"
                    << std::endl;
        } else {
            tempLaneMap[lane->index] = lane;
            if (laneMap.find(lane->id) != laneMap.end()) {
                std::cout
                        << "Error@SMTMap::insertEdgeFromEdgeXML-duplicated lane"
                        << std::endl;
            } else {
                laneMap[lane->id] = lane;
            }
        }
        laneXML = laneXML->getNextSiblingWithTag("lane");
    }
    // set the vector
    for (int i = 0; i < (int) tempLaneMap.size(); i++) {
        if (tempLaneMap.find(i) == tempLaneMap.end()) {
            std::cout
                    << "Error@SMTMap::insertEdgeFromEdgeXML-missing lane index"
                    << std::endl;
        } else {
            edge->laneVector.push_back(tempLaneMap[i]);
        }
    }
    if (edgeMap.find(edge->id) == edgeMap.end()) {
        edgeMap[edge->id] = edge;
        if (debug) {
            std::cout << "add edge " << edge->id << " to map." << std::endl;
        }
    } else {
        std::cout << "Error@SMTMap::insertEdgeFromEdgeXML-duplicated edge"
                << std::endl;
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
    con->fromSMTEdge = edgeMap[con->from];
    con->toSMTEdge = edgeMap[con->to];
    con->fromSMTLane = edgeMap[con->from]->laneVector[con->fromLane];
    con->toSMTLane = edgeMap[con->to]->laneVector[con->toLane];
    con->viaSMTLane = laneMap[con->via];

    // 处理关联
    con->fromSMTEdge->conVector.push_back(con);
    if (debug) {
        std::cout << "add connection between " << con->from << " and "
                << con->to << " via " << con->via << "." << std::endl;
    }
    bool hasConnected = false;
    for (int i = 0; i < (int) con->fromSMTLane->nextVector.size(); i++) {
        if (con->fromSMTLane->nextVector[i] == con->toSMTLane) {
            hasConnected = true;
        }
    }
    if (hasConnected) {
        std::cout << "Error@SMTMap::insertEdgeFromEdgeXML-duplicated connection"
                << std::endl;
    } else {
        con->fromSMTLane->nextVector.push_back(con->toSMTLane);
    }

}

void SMTMap::verfyNetConfig() {
//    list<string> edgeList = getLaunchd()->getCommandInterface()->get
    //TODO
}
