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
#include "StringHelper.h"

Define_Module(SMTMap);

SMTEdge::~SMTEdge() {
    // release connections
    for (unsigned int i = 0; i < conVector.size(); i++) {
        delete (conVector[i]);
    }
    // release vias
    for (map<SMTEdge*, vector<SMTVia*> >::iterator it = viaVecMap.begin();
            it != viaVecMap.end(); ++it) {
        for (unsigned int i = 0; i < it->second.size(); ++i) {
            delete (it->second[i]);
        }
    }
}

SMTLane::~SMTLane() {
    // do nothing
}

SMTVia::~SMTVia() {
    // do nothing
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

SMTVia::SMTVia(SMTEdge* edge, unsigned int conIndex) :
        start(NULL), fromLane(-1), target(NULL), toLane(-1), length(-1) {
    initVia(edge, conIndex);
}

double SMTVia::getViaLength() {
    if (length == -1) {
        calcViaLength();
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

SMTEdge* SMTMap::getSMTEdgeById(string id) {
    ASSERT2(edgeMap.find(id) != edgeMap.end(),
            "try to get SMTEdge from an unknown id.");
    return edgeMap[id];
}

void SMTMap::initialize(int stage) {
    if (stage == 0) {
        debug = hasPar("debug") ? par("debug") : false;
        rouXML = par("rouXML").xmlValue();
        netXML = par("netXML").xmlValue();

        initNetFromXML(netXML);
        hasInitialized = true;
        stepMsg = new cMessage("step message of SMTMap");
        scheduleAt(simTime() + 0.1, stepMsg);
    }
}

void SMTMap::handleMessage(cMessage *msg) {
    if (msg == stepMsg) {
        if (getLaunchd()->isConnected()) {
            addBaseRoutes();
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
    // 导入道路与连接并优化地图网络用于寻路
    // import edges and lanes
    int edgeNum = 0;
    int primaryEdgeNum = 0;
    cXMLElement* edgeXML = xml->getFirstChildWithTag("edge");
    while (edgeXML) {
        if (addEdgeFromEdgeXML(edgeXML)) {
            ++primaryEdgeNum;
        }
        ++edgeNum;
        edgeXML = edgeXML->getNextSiblingWithTag("edge");
    }
    std::cout << "edge number:" << edgeNum << std::endl;
    std::cout << "primary edge number:" << primaryEdgeNum << std::endl;
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
    // 优化网络,生成易于寻路的参数
    // 获取primary edge 集合
    for (map<string, SMTEdge*>::iterator it = edgeMap.begin();
            it != edgeMap.end(); ++it) {
        // 为edge填充viaMap(实际只有primary edge会使用)
        it->second->fillViaMap();
        if (!it->second->isInternal) {
            if (primaryEdgeSet.find(it->second) == primaryEdgeSet.end()) {
                primaryEdgeSet.insert(it->second);
            } else {
                std::cout << "redundant edge" << std::endl;
            }
        }
    }
    // 分析地图拓扑,找出需要标注的地图变量
    // 1. 若道路出口仅有掉头,则其为outEdge,对应掉头道路为enterEdge
    // 2. 从主要道路中移除enterEdge与outEdge后其他道路为内部道路
    set<SMTEdge*> innerSet; // 内部互联道路
    set<SMTEdge*> outSet;   // 地图出口道路
    set<SMTEdge*> enterSet;    // 地图入口道路
    innerSet = primaryEdgeSet;
    for (set<SMTEdge*>::iterator it = innerSet.begin(); it != innerSet.end();) {
        SMTEdge* edge = *it;
        if (edge->conVector.size() == 1) {
            SMTEdge* reEdge = getReverseEdge(edge);
            if (edge->conVector[0]->toSMTEdge == reEdge) {
                enterSet.insert(reEdge);
                outSet.insert(edge);
                // erase reverse edge(in edge) first
                // move it forward and then erase edge(out edge)
                innerSet.erase(reEdge);
                ++it;
                innerSet.erase(edge);
                // since it goes forward in erase operation
                // use continue to escape ++it of this loop
                continue;
            }
        }
        ++it;
    }
    if (debug) {
        std::cout << "enter primary edge:" << std::endl;
        for (set<SMTEdge*>::iterator it = enterSet.begin();
                it != enterSet.end(); ++it) {
            std::cout << (*it)->id << std::endl;
        }
        std::cout << "out primary edge:" << std::endl;
        for (set<SMTEdge*>::iterator it = outSet.begin(); it != outSet.end();
                ++it) {
            std::cout << (*it)->id << " to " << getReverseEdge(*it)->id
                    << std::endl;
        }
    }
    // set vector for each category
    innerPrimaryEdges = vector<SMTEdge*>(innerSet.begin(), innerSet.end());
    outPrimaryEdges = vector<SMTEdge*>(outSet.begin(), outSet.end());
    enterPrimaryEdges = vector<SMTEdge*>(enterSet.begin(), enterSet.end());
}

bool SMTMap::addEdgeFromEdgeXML(cXMLElement* xml) {
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
    return !edge->isInternal;
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
    con->fromLane = Fanjing::StringHelper::str2int(attrMap["fromLane"]);
    con->toLane = Fanjing::StringHelper::str2int(attrMap["toLane"]);
    con->via = attrMap["via"];
    con->tl = attrMap["tl"];
    con->linkIndex = Fanjing::StringHelper::str2int(attrMap["linkIndex"]);
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
                << "Warning@SMTMap::insertEdgeFromEdgeXML-duplicated connection:"
                << con->fromSMTLane->id << "->" << con->toSMTLane->id
                << std::endl;
        delete (con);
    } else {
        con->fromSMTEdge->conVector.push_back(con);
        con->fromSMTLane->conVector.push_back(con);
        con->fromSMTLane->nextVector.push_back(con->toSMTLane);
    }
}

SMTEdge* SMTMap::getReverseEdge(SMTEdge* edge) {
    string reverseEdgeName = getReverseEdgeName(edge->id);
    if (edgeMap.find(reverseEdgeName) == edgeMap.end()) {
        std::cout << "Warning@SMTMap::getReverseEdge-no reverse edge of '"
                << edge->id << "'" << std::endl;
    }
    return edgeMap[reverseEdgeName];
}

string SMTMap::getReverseEdgeName(const string& id) {
    return getEndEdgeName(id) + "to" + getStartEdgeName(id);
}

string SMTMap::getStartEdgeName(const string& id) {
    string result = "";
    string to = "to";
    string::size_type i = id.find(to);
    if (i < id.length() && i != string::npos) {
        result = id.substr(0, id.find(to));
    }
    return result;
}

string SMTMap::getEndEdgeName(const string& id) {
    string result = "";
    string to = "to";
    string::size_type i = id.find(to);
    if (i < id.length() && i != string::npos) {
        // '_'用于判定Lane,实际edge不会包含该字符无意义
        string::size_type j = id.find("_");
        if (j < id.length() && j != string::npos) {
            std::cout << "Warning@SMTMap::getReverseEdgeName-abnormal edge name"
                    << std::endl;
            result = id.substr(i + to.length(), j - i - 1);
        } else {
            result = id.substr(i + to.length());
        }
    }
    return result;
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
    // do nothing
}

SMTPhase::~SMTPhase() {
    // do nothing
}

void SMTEdge::fillViaMap() {
    for (unsigned int i = 0; i < conVector.size(); ++i) {
        if (conVector[i]->toSMTEdge != NULL) {
            viaVecMap[conVector[i]->toSMTEdge].push_back(new SMTVia(this, i));
        }
    }
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
        SMTConnection* con = conVector[i];
        if (con->via == "") {
            // 如果连接向内部节点则遍历所有可能出口,并继续寻找直至找到主要道路
            // 反之则打印结果
            if (con->toSMTEdge->isInternal) {
                con->toSMTEdge->printViaPath(ttl + 1, toEdge,
                        prefix + id + "->", suffix);
            } else {
                if (toEdge == NULL || toEdge == con->toSMTEdge) {
                    std::cout << prefix << id << "->" << con->toSMTEdge->id
                            << suffix << std::endl;
                } else {
                    std::cout << prefix + "unmatched end." + suffix
                            << std::endl;
                }
            }
        } else {
            if (toEdge == NULL && !con->toSMTEdge->isInternal) {
                if (!isInternal) {
                    //  搜索via列表
                    if (viaVecMap.find(con->toSMTEdge) == viaVecMap.end()) {
                        std::cout << "cannot find via to " << con->toSMTEdge->id
                                << " in edge " << id << std::endl;
                        return;
                    }
                    double viaLen;
                    for (unsigned int k = 0; true; ++k) {
                        if (k == viaVecMap[con->toSMTEdge].size()) {
                            std::cout << "cannot find via to lane "
                                    << con->toSMTEdge->laneVector[con->toLane]->id
                                    << " in edge " << id << std::endl;
                            break;
                        }
                        if (viaVecMap[con->toSMTEdge][k]->toLane
                                == con->toLane) {
                            viaLen =
                                    viaVecMap[con->toSMTEdge][k]->getViaLength();
                            break;
                        }
                    }
                    string strViaLen = ", via length="
                            + Fanjing::StringHelper::dbl2str(viaLen);
                    con->viaSMTLane->edge->printViaPath(ttl + 1, con->toSMTEdge,
                            prefix + "'" + id + "'->'" + con->to + "' - " + id
                                    + "->[", "]" + suffix + strViaLen);
                } else {
                    con->viaSMTLane->edge->printViaPath(ttl + 1, con->toSMTEdge,
                            prefix + id + "->[", "]" + suffix);
                }
            } else {
                con->viaSMTLane->edge->printViaPath(ttl + 1, toEdge,
                        prefix + id + "->[", "]" + suffix);
            }
        }
    }
}

void SMTVia::calcViaLength() {
    length = 0;
    for (list<SMTLane*>::iterator it = vias.begin(); it != vias.end(); ++it) {
        length += (*it)->length;
    }
}

void SMTVia::initVia(SMTEdge* edge, unsigned int conIndex) {
    if (conIndex >= edge->conVector.size()) {
        std::cout << "connection index " << conIndex << " of edge " << edge->id
                << " is out of bounds[" << conIndex << " in size "
                << edge->conVector.size() << "]" << std::endl;
        return;
    }
    SMTConnection* con = edge->conVector[conIndex];
    vias.clear();
    start = edge;
    fromLane = con->fromLane;
    target = con->toSMTEdge;
    toLane = con->toLane;
    while (con->viaSMTLane != NULL) {
        vias.push_back(con->viaSMTLane);
        // find next corresponding connection
        SMTEdge* t_toEdge = con->toSMTEdge;
        int t_toLane = con->toLane;
        if (t_toEdge->laneVector[t_toLane] != target->laneVector[toLane]) {
            // 一般情况下via中toEdge-toLane应当与target-toLane一致.
            std::cout << "unmatched target in SMTVia constructor:"
                    << t_toEdge->laneVector[t_toLane]->id << " - "
                    << target->laneVector[toLane]->id << std::endl;
        }
        unsigned int i = 0;
        // 检索via中对应toEdge的connection索引
        for (; i < con->viaSMTLane->conVector.size(); ++i) {
            // 需要找到via中与connection中一致的
            SMTConnection* conInVia = con->viaSMTLane->conVector[i];
            if (conInVia->toSMTEdge->laneVector[conInVia->toLane]
                    == t_toEdge->laneVector[t_toLane]) {
                break;
            }
        }
        if (i < con->viaSMTLane->conVector.size()) {
            vias.push_back(con->viaSMTLane);
            con = con->viaSMTLane->conVector[i];
        } else {
            std::cout << "cannot find corresponding connection" << std::endl;
            return;
        }
    }
    calcViaLength();
}

void SMTMap::addBaseRoutes() {
    for (map<string, SMTEdge*>::iterator it = edgeMap.begin();
            it != edgeMap.end(); ++it) {
        if (!it->second->isInternal) {
            list<string> route;
            route.push_back(it->first);
            getLaunchd()->getSMTComInterface()->addRoute(it->first, route);
        }
    }
}
