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

#ifndef __SMTROUTING_SMTMAP_H_
#define __SMTROUTING_SMTMAP_H_

#include <omnetpp.h>
#include <string>
#include <vector>
#include <map>
#include <list>
#include <set>
#include "SMTLaunchd.h"
#include "SMTComInterface.h"

using std::string;
using std::vector;
using std::map;
using std::list;
using std::set;

class SMTEdge;
class SMTLane;
class SMTJunction;
class SMTTlLogic;
class SMTConnection;
class SMTRoute;
/**
 * SMTEdge:道路edge.
 */
class SMTEdge {
public:
    SMTEdge() :
            priority(-1), isInternal(false) {
    }
    virtual ~SMTEdge();
    // xml attributes
    string id;
    string from;
    string to;
    int priority;
    string function;

    // SMT attributes
    bool isInternal;    // 标识是否为internal edge
    vector<SMTLane*> laneVector;    // vector of related lanes
    vector<SMTConnection*> conVector;   // vector of related connections

    // optimized attributes
    map<SMTEdge*, vector<SMTRoute*> > routeVecMap;
};
/**
 * SMTLane:车道lane.
 */
class SMTLane {
public:
    SMTLane() :
            index(0), speed(0), length(0), edge(NULL) {
    }
    virtual ~SMTLane();
    // xml attributes
    string id;  // The id of the lane
    int index;  // A running number, starting with zero at the right-most lane
    double speed;   // The maximum speed allowed on this lane [m/s]
    double length;  // The length of this lane [m]
    // shape is ignored

    // SMT attributes
    SMTEdge* edge;  // parent edge
    vector<SMTConnection*> conVector;   // vector of related connections
    vector<SMTLane*> nextVector;   // vector of next lanes
};
/**
 * SMTJunction:路口junction.
 */
class SMTJunction {
    // not implement now
};
/**
 * SMTTlLogic:交通灯逻辑tlLogic.
 */
class SMTTlLogic {
    // not implement now
};
/**
 * SMTConnection:连接connection.
 */
class SMTConnection {
public:
    SMTConnection() :
            fromLane(0), toLane(0), linkIndex(0), fromSMTEdge(0), toSMTEdge(0), fromSMTLane(
                    0), toSMTLane(0), viaSMTLane(0) {
    }
    virtual ~SMTConnection();
    // xml attributes
    string from;   // The ID of the incoming edge at which the connection begins
    string to;  // The ID of the outgoing edge at which the connection ends
    int fromLane; // The lane of the incoming edge at which the connection begins
    int toLane; // The lane of the outgoing edge at which the connection ends
    string via; // The id of the lane to use to pass this connection across the junction
    // tl is ignored    // The id of the traffic light that controls this connection; the attribute is missing if the connection is not controlled by a traffic light
    int linkIndex; // The index of the signal responsible for the connection within the traffic light; the attribute is missing if the connection is not controlled by a traffic light
    string dir; // The direction of the connection. "s" = straight, "t" = turn, "l" = left, "r" = right, "L" = partially left, R = partially right, "invalid" = no direction
    // state is ignored // The state of the connection. "-" = dead end, "=" = equal, "m" = minor link, "M" = major link, traffic light only: "O" = controller off, "o" = yellow flashing, "y" = yellow minor link, "Y" = yellow major link, "r" = red, "g" = green minor, "G" green major

    // SMT attributes
    SMTEdge* fromSMTEdge;
    SMTEdge* toSMTEdge;
    SMTLane* fromSMTLane;
    SMTLane* toSMTLane;
    SMTLane* viaSMTLane;
};

/**
 * SMTRoute:edge到edge的路径.
 */
class SMTRoute {
public:
    SMTRoute() {
    }
    virtual ~SMTRoute();

    list<SMTEdge*> via;    // 路径包含的edge列表(目的街道为终点)
    SMTEdge* start;
    SMTEdge* target;
    double getViaLength();  // 返回via路径的长度
};
/**
 * SMTMap:地图系统,负责管理地图拓扑结构.
 */
class SMTMap: public cSimpleModule {
public:
    SMTMap() :
            debug(false), hasInitialized(false), launchd(0), rouXML(0), netXML(
                    0), stepMsg(0) {
    }
    virtual ~SMTMap();

    SMTLaunchd* getLaunchd();
    bool isReady() {
        return hasInitialized;
    }

    // Map topology API
    SMTEdge* getSMTEdge(string id);

protected:
    // parameters
    bool debug;
    bool hasInitialized;
    SMTLaunchd* launchd;
    cXMLElement* rouXML;
    cXMLElement* netXML;
    cMessage* stepMsg;

    map<string, SMTEdge*> edgeMap;
    map<string, SMTLane*> laneMap;

    // functions
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);

    virtual void initNetFromXML(cXMLElement* xml);
    virtual void optimizeNet();
    virtual void initVechileTypeFromXML(cXMLElement* xml);
    void addEdgeFromEdgeXML(cXMLElement* xml);
    void addConFromConXML(cXMLElement* xml);
    void verifyNetConfig();
};

class SMTMapAccess {
public:
    SMTMap* get() {
        return FindModule<SMTMap*>::findGlobalModule();
    }
};
#endif
