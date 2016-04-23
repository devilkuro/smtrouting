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
using std::vector;

class SMTBaseRouting: public cSimpleModule {
public:
    // 内部class
    // WEIGHTLANE: use for record car queue of lane
    class WeightEdge;
    class WeightLane {

    public:
        class CarTime {
        public:
            CarTime(SMTCarInfo* _car, double _cost) :
                    car(_car), cost(_cost) {
            }
            SMTCarInfo* car;
            double cost;
        };
        class laneState {
        public:
            laneState() :
                    minPassTime(-1), maxPassTime(-1), totalPassTime(0), passedCarNum(
                            0) {

            }
            double minPassTime;
            double maxPassTime;
            double totalPassTime;
            int passedCarNum;
        };
        WeightLane() :
                viaLen(-1), occupation(0), occStep(0), occupaChangeFlag(false), to(
                NULL), recentCost(-1), recentCostLastupdateTime(-1), recentCostRefreshFlag(
                        false), totalRecentCost(0) {
        }

        virtual ~WeightLane();
        laneState statistic;
        double viaLen;
        double occupation;
        double occStep;
        bool occupaChangeFlag;
        static double outCarKeepDuration;
        static double limitStart;
        static double limitCap;
        static double limitFix;
        WeightEdge* to; // FIXME may lead to multiple edges
        map<SMTCarInfo*, double> carMap;
        multimap<double, SMTCarInfo*> enterTimeMap;
        multimap<double, CarTime> recentOutCars;
        virtual double getCost(double time);
        virtual void carGetOut(SMTCarInfo* car, const double &t,
                const double &cost);
        virtual void carPassVia(double time);
        void insertCar(SMTCarInfo* car, double t);
        void removeCar(SMTCarInfo* car, double t);
    protected:
        // set to true when recentOutCars or totalCost changed
        double recentCost;    // stand for pass through time
        double recentCostLastupdateTime;
        bool recentCostRefreshFlag;
        double totalRecentCost;
        virtual void updateCost(double time);
    };
    // WEIGHTEDGE: 用于dijkstra‘s algorithm
    class WeightEdge {
    public:
        WeightEdge(SMTEdge* e) :
                w(-1), t(-1), previous(NULL), edge(e) {
        }
        virtual ~WeightEdge();
        double w;
        double t;
        WeightEdge* previous;
        SMTEdge* edge;
        map<SMTEdge*, WeightLane*> w2NextMap;
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
    enum SMT_ROUTING_TYPE {
        SMT_RT_SHOREST = 0, SMT_RT_FAST, SMT_RT_AIR, SMT_RT_CORP
    };
public:
    SMTBaseRouting() :
            suppressLength(40), debug(false), debugMsg(NULL), startTime(-1), carInfo(
                    NULL), routeType(SMT_RT_FAST), _pMap(NULL) {
    }
    virtual ~SMTBaseRouting();

    SMTMap* getMap();

    // routing functions
    // TODO 添加基本的寻路方法
    virtual void changeRoad(SMTEdge* from, SMTEdge* to, int toLane, double time,
            SMTCarInfo* car, double viaTime = -1);
    virtual void getShortestRoute(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &rou, double time = -1, SMTCarInfo* car = NULL);
    virtual void getFastestRoute(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &rou, double time = -1, SMTCarInfo* car = NULL);
    virtual void getAIRRoute(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &rou, double time = -1, SMTCarInfo* car = NULL);
    virtual void getCOOPRoute(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &rou, double time = -1, SMTCarInfo* car = NULL);

    // try to change to corrected lane by suppress cars prevent this car
    virtual bool suppressEdge(SMTEdge* edge, double pos = -1);
    virtual void releaseEdge(SMTEdge* edge);
    inline const map<SMTEdge*, double> &getSuppressedEdgeMapRef() {
        return suppressedEdges;
    }
protected:
    // members for dijkstra's algorithm
    // weightEdgeMap用于存储所有WeightEdge便于回收内存
    map<SMTEdge*, WeightEdge*> weightEdgeMap;
    multimap<double, WeightEdge*> processMap;
    map<SMTEdge*, double> suppressedEdges;
    double suppressLength;

    bool debug;
    cMessage* debugMsg;

    double startTime;
    SMTCarInfo* carInfo;
    SMT_ROUTING_TYPE routeType;
    // functions
    virtual int numInitStages() const;
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    virtual void runDijkstraAlgorithm(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &route);
    // independent weight modify function
    virtual double modifyWeightFromEdgeToEdge(WeightEdge* from, WeightEdge* to);
    double getSmallerOne(double a, double b);
    // protected members
    SMTMap* _pMap;

private:
    // dijkstra's algorithm related
    void initDijkstra(SMTEdge* origin);
    void changeDijkstraWeight(WeightEdge* from, WeightEdge* to, double w);
    int processDijkstraLoop(SMTEdge* destination);
    SMTEdge* processDijkstralNode(SMTEdge* destination);
    void processDijkstralNeighbors(WeightEdge* wEdge);
    void getDijkstralResult(SMTEdge* destination, list<SMTEdge*> &route);
};

class SMTRoutingAccess {
public:
    SMTBaseRouting* get() {
        return FindModule<SMTBaseRouting*>::findGlobalModule();
    }
};

#endif /* __SMTBASEROUTING_H_ */
