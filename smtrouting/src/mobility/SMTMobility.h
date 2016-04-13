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

#ifndef SMTMOBILITY_H_
#define SMTMOBILITY_H_

#include "TraCIMobility.h"
#include "SMTMap.h"
#include "SMTCarManager.h"
#include "SMTBaseRouting.h"

class SMTMobility: public Veins::TraCIMobility {
public:
    class SMTStat {
    public:
        SMTStat() :
                lastDroveAt(0), enterEdgeTime(0), enterQueueTime(0), outQueueTime(
                        0), outEdgeTime(0), enterNextEdgeTime(0) {

        }

        // 统计相关
        double lastDroveAt; // 用于记录上一次停车的时间
        double lastStopAt;
        double enterEdgeTime; // 进入edge时间
        double enterQueueTime;    // 进入队列区时间
        double outQueueTime;  // 启动准备离开队列区
        double outEdgeTime;   // 离开edge时间
        double enterNextEdgeTime; // 进入下一条edge时间
    };
    SMTMobility() :
            carInfo(NULL), origin(NULL), destination(
            NULL), hasRouted(false), hasInitialized(false), lastEdge(0), curPrimaryEdge(
                    0), lastPrimaryEdge(0), laneChangeMsg(0), laneChangeDuration(
                    5), preferredLaneIndex(0), isChangeAndHold(false), smtMap(
                    0), _pCarManager(
            NULL), _pRouting(NULL) {
    }
    virtual ~SMTMobility();

    virtual void initialize(int stage);
    virtual void finish();
    virtual void preInitialize(std::string external_id, const Coord& position,
            std::string road_id = "", double speed = -1, double angle = -1);
    virtual void nextPosition(const Coord& position, std::string road_id = "",
            double speed = -1, double angle = -1,
            Veins::TraCIScenarioManager::VehicleSignal signals =
                    Veins::TraCIScenarioManager::VEH_SIGNAL_UNDEF);
    virtual void handleSelfMsg(cMessage *msg);

    // 交通控制API
    void setPreferredLaneIndex(uint8_t laneIndex);
    void changeToPreferredLane(int laneIndex = -1);
protected:
    // 接口成员
    SMTStat smtStat;
    SMTCarInfo* carInfo;
    SMTBaseRouting* getRouting() {
        if (_pRouting == NULL) {
            _pRouting = SMTRoutingAccess().get();
        }
        return _pRouting;
    }
    SMTMap* getMap() {
        if (smtMap == NULL) {
            smtMap = SMTMapAccess().get();
        }
        return smtMap;
    }
    SMTCarManager* getCarManager() {
        if (_pCarManager == NULL) {
            _pCarManager = SMTCarManagerAccess().get();
        }
        return _pCarManager;
    }
    inline SMTComInterface* getComIf() {
        return getMap()->getLaunchd()->getSMTComInterface();
    }
    // 车辆相关信息
    SMTEdge* origin;
    SMTEdge* destination;
    // 中间过程变量
    bool hasRouted; // 用于判定是否已经分配路径
    bool hasInitialized;    // 用于判定是否已经于地图上初始化
    string lastRoadId;    // 用于记录上一条道路的id
    SMTEdge* lastEdge;
    string curPrimaryRoadId;    // 记录上一条主要道路id
    SMTEdge* curPrimaryEdge;
    string lastPrimaryRoadId;    // 记录上一条主要道路id
    SMTEdge* lastPrimaryEdge;
    cMessage* laneChangeMsg;  // 用于保持车道的消息
    double laneChangeDuration;
    uint8_t preferredLaneIndex;
    bool isChangeAndHold;

    // overload these function in different mobility
    // processAfterRouting
    // this function will run every 0.1 second for each car if routed in routing process!!
    // so, do not do any complicated operations here.
    virtual void processAfterRouting();
    // statisticAtFinish
    virtual void statisticAtFinish();
    // initialize the route in routing process
    virtual bool processAtRouting();
    // when road changed from one road to another, not the first time appearing on the map.
    virtual void processWhenChangeRoad();
    // when the car first appear on the map.
    virtual void processWhenInitializingRoad();
    // this function will run every 0.1 second for each car!!
    // so, do not do any complicated operations here.
    virtual void processWhenNextPosition();

    // 消息处理部分
    virtual void handleLaneChangeMsg(cMessage *msg);

    void startChangeLane(uint8_t laneIndex, double delay = 0);
private:
    SMTMap* smtMap;
    SMTCarManager* _pCarManager;
    SMTBaseRouting* _pRouting;

    void cmdSetNoOvertake();
    void cmdChangeLane(uint8_t laneIndex, uint32_t duration = 0);
    double cmdGetLanePosition();
};

#endif /* SMTMOBILITY_H_ */
