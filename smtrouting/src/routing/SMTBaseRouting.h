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

#include <omnetpp.h>
#include "SMTMap.h"
#include "SMTCarInfo.h"
#include "SMTCarManager.h"
#include <set>
#include "StatisticsRecordTools.h"

using std::list;
using std::multimap;
using std::map;
using std::set;
using std::vector;
using namespace tinyxml2;
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
        class HisInfo {
        public:
            HisInfo() :
                    car(0), time(0), next(0), laneTime(0), viaTime(0), intervalToLast(
                            0) {
            }
            SMTCarInfo* car;
            double time;
            WeightLane* next;
            double laneTime;
            double viaTime;
            double intervalToLast;
        };
        class laneStatus {
        public:
            laneStatus() :
                    minViaPassTime(-1), maxViaPassTime(-1), totalViaPassTime(0), minLanePassTime(
                            -1), maxLanePassTime(-1), totalLanePassTime(0), minPassLaneInterval(
                            -1), maxPassLaneNumOneCircle(0), passedCarNum(0) {
            }
            double minViaPassTime;
            double maxViaPassTime;
            double totalViaPassTime;
            double minLanePassTime;
            double maxLanePassTime;
            double totalLanePassTime;
            double minPassLaneInterval;
            int maxPassLaneNumOneCircle;

            int passedCarNum;
        };
        WeightLane() :
                via(0), con(0), viaLen(-1), occupation(0), occStep(0), occupaChangeFlag(
                        false), airSI(0), from(0), to(0), recentCost(-1), recentCostLastupdateTime(
                        -1), recentCostRefreshFlag(false), totalRecentCost(0), airCostUpdateFlag(
                        false), airDLastUpdateTime(0), airD(0), lastCarOutTime(
                        0) {
        }

        virtual ~WeightLane();
        laneStatus status;
        SMTVia* via;
        SMTConnection* con;
        double viaLen;
        double occupation;
        double occStep;
        bool occupaChangeFlag;
        double airSI;
        static double airK;
        static double airV;
        static double outCarKeepDuration;
        static double limitStart;
        static double limitCap;
        static double limitFix;
        WeightEdge* from;
        WeightEdge* to; // FIXME may lead to multiple edges
        map<SMTCarInfo*, double> carMap;
        multimap<double, SMTCarInfo*> enterTimeMap;
        multimap<double, CarTime> recentOutCars;
        // CoRP related
        map<SMTCarInfo*, HisInfo*> hisCarMap;
        multimap<double, HisInfo*> hisTimeMap;

        virtual void carGetOut(SMTCarInfo* car, const double &t,
                const double &cost);
        void initMinAllowedCost();
        virtual void carPassVia(double time);
        virtual void carPassLane(double time);
        void insertCar(SMTCarInfo* car, double t);
        void removeCar(SMTCarInfo* car, double t);
        virtual double getCost(double time);
        // AIR related
        void updateAIRsi();
        virtual double getAIRCost(double time);
        // CoRP related
        void addHistoricalCar(SMTCarInfo* car, double t);
        void getOutHistoricalCar(SMTCarInfo* car, double laneTime,
                double viaTime, double time, WeightLane* next);
        void removeHistoricalCar(SMTCarInfo* car, double t);
    protected:
        // set to true when recentOutCars or totalCost changed
        double recentCost;    // stand for pass through time
        double minAllowedCost;
        double recentCostLastupdateTime;
        bool recentCostRefreshFlag;
        double totalRecentCost;
        bool airCostUpdateFlag;
        bool airDLastUpdateTime;
        double airD;
        double lastCarOutTime;
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
        SMT_RT_SHOREST = 0,
        SMT_RT_FAST,
        SMT_RT_AIR,
        SMT_RT_CORP_SELF,
        SMT_RT_CORP_TTS
    };
    class RoutingState {
    public:
        RoutingState() :
                arrivedCarCount(0), recordActiveCarNum(false), recordActiveCarInterval(
                        120) {

        }
        double arrivedCarCount;
        bool recordActiveCarNum;
        double recordActiveCarInterval;
    };
public:
    SMTBaseRouting() :
            suppressLength(40), debug(false), debugMsg(0), statisticMsg(0), endSimMsg(
                    0), startTime(-1), carInfo(0), majorRoutingType(
                    SMT_RT_FAST), minorRoutingType(SMT_RT_FAST), recordHisRecordRoutingType(
                    -1), enableAIR(false), enableCoRP(false), replaceAIRWithITSWithOccupancy(
                    false), recordHisRoutingData(false), endAfterLoadHisXML(
                    false), airUpdateMsg(0), routeType(SMT_RT_FAST), srt(0), _pMap(
                    0), _pCarManager(0) {
    }
    virtual ~SMTBaseRouting();

    SMTMap* getMap();
    SMTCarManager* getCarManager();

    // routing functions
    // TODO 添加基本的寻路方法
    virtual void changeRoad(SMTEdge* from, SMTEdge* to, int toLane, double time,
            SMTCarInfo* car, double viaTime = -1, double laneTime = -1);
    virtual void getShortestRoute(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &rou, double time = -1, SMTCarInfo* car = NULL);
    virtual void getFastestRoute(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &rou, double time = -1, SMTCarInfo* car = NULL);
    virtual void getAIRRoute(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &rou, double time = -1, SMTCarInfo* car = NULL);
    virtual void getCORPSelfRoute(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &rou, double time = -1, SMTCarInfo* car = NULL);
    virtual void getCORPTTSRoute(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &rou, double time = -1, SMTCarInfo* car = NULL);
    virtual void getRouteByMajorMethod(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &rou, double time = -1, SMTCarInfo* car = NULL);
    virtual void getRouteByMinorMethod(SMTEdge* origin, SMTEdge* destination,
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
    cMessage* statisticMsg;
    cMessage* endSimMsg;
    string recordXMLPrefix;
    double startTime;
    SMTCarInfo* carInfo;
    SMT_ROUTING_TYPE majorRoutingType;
    SMT_ROUTING_TYPE minorRoutingType;
    int recordHisRecordRoutingType;
    bool enableAIR;
    bool enableCoRP;
    bool replaceAIRWithITSWithOccupancy;
    // record historical routing data
    bool recordHisRoutingData;
    string hisRecordXMLPath;
    bool endAfterLoadHisXML;
    cMessage* airUpdateMsg;
    SMT_ROUTING_TYPE routeType;
    RoutingState rouState;
    Fanjing::StatisticsRecordTools* srt;
    // functions
    virtual int numInitStages() const;
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    virtual void exportHisXML();
    virtual void importHisXML();
    virtual void printStatisticInfo();
    virtual void updateStatisticInfo();
    virtual void updateAIRInfo();
    virtual void runDijkstraAlgorithm(SMTEdge* origin, SMTEdge* destination,
            list<SMTEdge*> &route);
    // independent weight modify function
    virtual double modifyWeightFromEdgeToEdge(WeightEdge* from, WeightEdge* to);
    double getSmallerOne(double a, double b);
    // protected members
    SMTMap* _pMap;
    SMTCarManager* _pCarManager;
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
