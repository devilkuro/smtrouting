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

SMTVia::~SMTVia() {
    // TODO 释放资源
}

double SMTEdge::length() {
    if (_len < 0) {
        _len = 0;
        for (unsigned int i = 0; i < laneVector.size(); ++i) {
            _len += laneVector[i]->length;
            if (laneVector[i]->length != laneVector[0]->length) {
                std::cout << "unmatched lane length." << std::endl;
            }
        }
        _len = _len / laneVector.size();
    }
    return _len;
}

double SMTVia::getViaLength() {
    // TODO 获取via路径长度
    if (length == -1) {
        length = 0;
        for (list<SMTEdge*>::iterator it = vias.begin(); it != vias.end();
                ++it) {
            length += (*it)->length();
        }
    }
    return length;
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
    ASSERT2(edgeMap.find(id) != edgeMap.end(),
            "try to get SMTEdge from an unknown id.");
    return edgeMap[id];
}

void SMTMap::initialize() {
    debug = hasPar("debug") ? par("debug") : false;
    rouXML = par("rouXML").xmlValue();
    netXML = par("netXML").xmlValue();

    initNetFromXML(netXML);
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
            cancelAndDelete(stepMsg);
            stepMsg = NULL;
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
    // import traffic lights
    cXMLElement* tlXML = xml->getFirstChildWithTag("tlLogic");
    while (tlXML) {
        addTLFromTLXML(tlXML);
        tlXML = tlXML->getNextSiblingWithTag("tlLogic");
    }
    // import connections
    cXMLElement* conXML = xml->getFirstChildWithTag("connection");
    while (conXML) {
        addConFromConXML(conXML);
        conXML = conXML->getNextSiblingWithTag("connection");
    }
    optimizeNet();
}

void SMTMap::optimizeNet() {
    // TODO 优化网络,生成易于寻路的参数

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

void SMTMap::addTLFromTLXML(cXMLElement* xml) {
    SMTTLLogic* tl = new SMTTLLogic();
    cXMLAttributeMap attrMap = xml->getAttributes();
    tl->id = attrMap["id"];
    tl->type = attrMap["type"];
    tl->programID = attrMap["programID"];
    tl->offset = atoi(attrMap["offset"].c_str());
    if (tl->type == "static") {
        // set phases in this tl
        cXMLElement* phaseXML = xml->getFirstChildWithTag("phase");
        unsigned int n = 0;
        while (phaseXML) {
            cXMLAttributeMap phaseAttrMap = phaseXML->getAttributes();
            if (n == 0) {
                n = phaseAttrMap["state"].size();
            } else {
                if (n != phaseAttrMap["state"].size()) {
                    std::cout
                            << "Error@SMTMap::addTLFromTLXML-unstable state size"
                            << std::endl;
                }
            }
            // 添加phases
            tl->phases.push_back(SMTPhase());
            tl->phases.back().duration = atoi(phaseAttrMap["duration"].c_str());
            tl->phases.back().state = phaseAttrMap["state"];
            phaseXML = phaseXML->getNextSiblingWithTag("phase");
        }
        tl->segments.assign(n, SMTSegment());
        // 设置phase状态队列
        vector<list<double> > durList(n);
        vector<list<string> > valList(n);
        for (unsigned int i = 0; i < tl->phases.size(); ++i) {
            for (unsigned int j = 0; j < n; ++j) {
                durList[j].push_back(tl->phases[i].duration);
                valList[j].push_back(tl->phases[i].state.substr(j, 1));
            }
        }
        // 设置segments信息
        for (unsigned int i = 0; i < n; ++i) {
            tl->segments[i].setSegment(durList[i], valList[i], tl->offset);
        }
        // 记录tl信息至tlMap
        tlMap[tl->id] = tl;
    } else {
        std::cout << "Warning@SMTMap::addTLFromTLXML-non-static traffic light"
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
    con->tl = attrMap["tl"];
    con->linkIndex = strtol(attrMap["linkIndex"].c_str(), 0, 0);
    con->dir = attrMap["dir"];
    // SMT 属性
    ASSERT2(edgeMap.find(con->from) != edgeMap.end(), "unknown from edge.");
    con->fromSMTEdge = edgeMap[con->from];
    ASSERT2(edgeMap.find(con->to) != edgeMap.end(), "unknown to edge.");
    con->toSMTEdge = edgeMap[con->to];
    // via,tl 为可选属性
    if (con->via != "") {
        ASSERT2(laneMap.find(con->via) != laneMap.end(), "unknown tl.");
        con->viaSMTLane = laneMap[con->via];
    }
    if (con->tl != "") {
        ASSERT2(tlMap.find(con->tl) != tlMap.end(), "unknown tl.");
        con->tlSMTTL = tlMap[con->tl];
        // 转存控制信号时间信息
        SMTSegment seg = con->tlSMTTL->segments[con->linkIndex];
        con->t0 = seg.t0;
        con->tg = seg.tg;
        con->ty = seg.ty;
        con->tr = seg.tr;
    }
    con->fromSMTLane = edgeMap[con->from]->laneVector[con->fromLane];
    con->toSMTLane = edgeMap[con->to]->laneVector[con->toLane];
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
    std::cout << "verifying lanes and edges ..." << std::endl;
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
    // verify the connections
    std::cout << "verifying connections ..." << std::endl;
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
    // verify the tl (by print them)
    std::cout << "printing tls ..." << std::endl;
    for (map<string, SMTLane*>::iterator it = laneMap.begin();
            it != laneMap.end(); ++it) {
        for (unsigned int i = 0; i < it->second->conVector.size(); ++i) {
            SMTConnection* con = it->second->conVector[i];
            if (con->tl != "") {
                SMTSegment seg = con->tlSMTTL->segments[con->linkIndex];
                std::cout << "{from:" << con->fromSMTLane->id << ",to:"
                        << con->toSMTLane->id << ",via:" << con->viaSMTLane->id
                        << "}" << "-{T0,Tg,Ty,Tr}:{" << seg.t0 << "," << seg.tg
                        << "," << seg.ty << "," << seg.tr << "}" << std::endl;
            }
        }
    }
    std::cout << "printing tls finished." << std::endl;
    // verify the connections from edge to edge (by print them)
    std::cout << "printing connections from edge to edge ..." << std::endl;
    for (map<string, SMTEdge*>::iterator it = edgeMap.begin();
            it != edgeMap.end(); ++it) {
        if (!it->second->isInternal) {
            // ViaPath打印
            it->second->printViaPath();
        }
        if (it->second->isInternal) {
            // ViaPath打印
            it->second->printViaPath();
        }
    }
    if (par("endAfterVerified").boolValue()) {
        endSimulation();
    }
}

void SMTMap::finish() {
    Fanjing::StatisticsRecordTools *srt =
            Fanjing::StatisticsRecordTools::request();
    srt->outputSeparate("trajectory.txt", "./results");
    srt->clean();
    std::cout << "Map::finish" << std::endl;
}

SMTTLLogic::~SMTTLLogic() {
    // TODO
}

SMTPhase::~SMTPhase() {
}

void SMTEdge::printViaPath(const int ttl, const SMTEdge* toEdge,
        const string &prefix, const string &suffix) {
    // 打印Via路径
    // 使用ttl防止循环内联道路
    if (ttl > 5) {
        std::cout << "may have loop." << std::endl;
        return;
    }
    // 若无连接则为死路
    if (conVector.size() == 0) {
        std::cout << prefix + id + " dead end " + suffix << std::endl;
        return;
    }
    for (unsigned int i = 0; i < conVector.size(); ++i) {
        // 若有中间edge,则打印中间edge
        if (conVector[i]->via == "") {
            // 如果连接向内部节点则遍历所有可能出口,并继续寻找直至找到主要道路
            // 反之则打印结果
            if (conVector[i]->toSMTEdge->isInternal) {
                conVector[i]->toSMTEdge->printViaPath(ttl + 1, toEdge,
                        prefix + id + "->", suffix);
            } else {
                if (toEdge == NULL || toEdge == conVector[i]->toSMTEdge) {
                    std::cout << prefix << id << "->"
                            << conVector[i]->toSMTEdge->id << suffix
                            << std::endl;
                } else {
                    std::cout << prefix + "unmatched end." + suffix
                            << std::endl;
                }
            }
        } else {
            if (toEdge == NULL && !conVector[i]->toSMTEdge->isInternal) {
                if (!isInternal) {
                    conVector[i]->viaSMTLane->edge->printViaPath(ttl + 1,
                            conVector[i]->toSMTEdge,
                            prefix + "'" + id + "'->'" + conVector[i]->to
                                    + "' - " + id + "->[", "]" + suffix);
                } else {
                    conVector[i]->viaSMTLane->edge->printViaPath(ttl + 1,
                            conVector[i]->toSMTEdge, prefix + id + "->[",
                            "]" + suffix);
                }
            } else {
                conVector[i]->viaSMTLane->edge->printViaPath(ttl + 1, toEdge,
                        prefix + id + "->[", "]" + suffix);
            }
        }
    }
}
