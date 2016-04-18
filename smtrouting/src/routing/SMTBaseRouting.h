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

#ifndef __SMTBASEROUTING_H_
#define __SMTBASEROUTING_H_

#include <csimplemodule.h>
#include "SMTMap.h"
#include "SMTCarInfo.h"
#include <set>

using std::list;
using std::multimap;
using std::map;
using std::set;

class SMTBaseRouting: public cSimpleModule {
public:
    // 内部class
    // WEIGHTEDGE: 用于dijkstra‘s algorithm
    class WeightEdge {
    public:
        WeightEdge(SMTEdge* e) :
                w(-1), t(-1), previous(NULL), edge(e) {
        }

        double w;
        double t;
        WeightEdge* previous;
        SMTEdge* edge;
    };
    // ROUTE:用于记录选择的路径
    class Route {
    public:
        Route() :
                cost(0) {
        }
        double cost;
        list<WeightEdge*> edges;
    };
public:
    SMTBaseRouting() :
            suppressLength(40), startTime(-1), carInfo(NULL), _pMap(NULL) {
    }
    virtual ~SMTBaseRouting();

    SMTMap* getMap();

    // routing functions
    // TODO 添加基本的寻路方法
    virtual void getShortestRoute(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &rou, double time = -1, SMTCarInfo* car = NULL);

    // try to change to corrected lane by suppress cars prevent this car
    virtual void suppressEdge(SMTEdge* edge, double pos = -1);
    virtual void releaseEdge(SMTEdge* edge);
    inline const map<SMTEdge*, double> &getSuppressedEdgeMapRef() {
        return suppressedEdges;
    }
protected:
    // members for dijkstra's algorithm
    // weightEdgeMap用于存储所有WeightEdge便于回收内存
    map<SMTEdge*, WeightEdge*> weightEdgeMap;
    multimap<double, WeightEdge*> processMap;
    // the untouched edges, maybe useless
    // map<SMTEdge*, WeightEdge*> unSet;
    // the finished edges, maybe useless
    // map<SMTEdge*, WeightEdge*> outSet;
    map<SMTEdge*, double> suppressedEdges;
    double suppressLength;

    double startTime;
    SMTCarInfo* carInfo;
    // functions
    virtual int numInitStages() const;
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    virtual void runDijkstraAlgorithm(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &route);
    // independent weight modify function
    virtual double getWeightFromEdgeToEdge(WeightEdge* from, WeightEdge* to);
    double getSmallerOne(double a, double b);
    // protected members
    SMTMap* _pMap;

private:
    // dijkstra's algorithm related
    void initDijkstra(SMTEdge* origin);
    void changeDijkstraWeight(WeightEdge* from, WeightEdge* to, double w);
    int processDijkstraLoop(SMTEdge* destination);
    SMTEdge* processDijkstralNode(SMTEdge* destination);
    virtual void processDijkstralNeighbors(WeightEdge* wEdge);
    void getDijkstralResult(SMTEdge* destination, list<SMTEdge*> &route);
};

class SMTRoutingAccess {
public:
    SMTBaseRouting* get() {
        return FindModule<SMTBaseRouting*>::findGlobalModule();
    }
};

#endif /* __SMTBASEROUTING_H_ */
