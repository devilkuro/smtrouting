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

using std::string;
using std::vector;
using std::map;
using std::list;

class SMTEdge;
class SMTLane;
class SMTConnection;
/**
 * SMTEdge:道路edge.
 */
class SMTEdge {
public:
    SMTEdge() :
            priority(-1) {}
    virtual ~SMTEdge();
    // xml attributes
    string id;
    string from;
    string to;
    int priority;
    string function;

    // SMT attributes
    bool isInternal;
    vector<SMTLane*> vecLanes;
};
/**
 * SMTLane:车道lane.
 */
class SMTLane {
public:
    SMTLane() :
            index(0), speed(0), length(0) {}
    virtual ~SMTLane();
    // xml attributes
    string id;  // The id of the lane
    int index;  // A running number, starting with zero at the right-most lane
    double speed;   // The maximum speed allowed on this lane [m/s]
    double length;  // The length of this lane [m]
    // shape is ignored

    // SMT attributes
    SMTEdge* edge;  // parent edge
    vector<SMTLane*> vecNext;   // vector of next lanes
};
/**
 * SMTJunction:路口junction.
 */
/**
 * SMTTlLogic:交通灯逻辑tlLogic.
 */
/**
 * SMTConnection:连接connection.
 */
class SMTConnection {
public:

};
/**
 * SMTMap:地图系统,负责管理地图拓扑结构.
 */
class SMTMap: public cSimpleModule {
public:
    SMTMap();
    virtual ~SMTMap();
protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);

};

#endif
