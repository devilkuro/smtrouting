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
#include <set>

using std::list;
using std::multimap;
using std::map;
using std::set;

class SMTBaseRouting: public cSimpleModule {
public:
    // 内部class
    // ROUTE:用于记录选择的路径
    class Route {
    public:
        Route() :
                cost(0) {
        }

        double cost;
        list<SMTEdge*> edges;
    };
    // WEIGHTEDGE: 用于dijkstra‘s algorithm
    class WeightEdge {
    public:
        WeightEdge(SMTEdge* e) :
                w(-1), previous(NULL), edge(e) {
        }

        double w;
        SMTEdge* previous;
        SMTEdge* edge;
    };
public:
    SMTBaseRouting() :
            _pMap(NULL) {
    }
    virtual ~SMTBaseRouting();

    SMTMap* getMap();

    // routing functions
    // TODO 添加基本的寻路方法
    virtual list<SMTEdge*> getShortestRoute(SMTEdge* origin,
            SMTEdge* destination);

protected:
    // members for dijkstra's algorithm
    // weightEdgeMap用于存储所有WeightEdge便于回收内存
    map<SMTEdge*, WeightEdge*> weightEdgeMap;
    multimap<double, WeightEdge*> processMap;
    map<SMTEdge*, WeightEdge*> unSet;    // the untouched edges
    map<SMTEdge*, WeightEdge*> outSet;   // the finished edges

    // functions
    virtual int numInitStages() const;
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    // protected members
    SMTMap* _pMap;

private:
    // dijkstra's algorithm related
    void initDijkstra();
    void changeDijkstraWeight(SMTEdge* from, SMTEdge* to, double w);
    void processDijkstraLoop();
};

#endif /* __SMTBASEROUTING_H_ */
