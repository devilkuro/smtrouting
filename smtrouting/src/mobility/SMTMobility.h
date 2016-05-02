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
                enterEdgeTime(-1), enterLastEdgeTime(-1), enterPrimaryEdgeTime(
                        -1), outPrimaryEdgeTime(-1) {
        }
        // 统计相关
        double enterEdgeTime; // 进入edge时间
        double enterLastEdgeTime;   // 进入上一条edge的时间
        double enterPrimaryEdgeTime;
        double outPrimaryEdgeTime;  // 离开上一条primary道路的时间
    };
    SMTMobility() :
            carInfo(NULL), origin(NULL), destination(
            NULL), hasRouted(false), hasInitialized(false), arrivedMsg(
            NULL), beSuppressed(false), hasSuppressEdge(false), isSlowDown(
                    false), checkSuppressInterval(10), checkSuppressedEdgesMsg(
            NULL), lastEdge(0), curEdge(
            NULL), lastPrimaryEdge(NULL), nextPrimaryEdge(NULL), laneChangeMsg(
                    0), laneChangeDuration(5), preferredLaneIndex(0), isChangeAndHold(
                    false), smtMap(0), _pCarManager(
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
    list<SMTEdge*> carRoute;
    // 中间过程变量
    bool hasRouted; // 用于判定是否已经分配路径
    bool hasInitialized;    // 用于判定是否已经于地图上初始化
    cMessage* arrivedMsg;

    bool beSuppressed;  // 是否被其他车辆压制
    bool hasSuppressEdge;   // 是否压制其他车辆
    bool isSlowDown;
    double checkSuppressInterval;   // 压制状态判定间隔
    cMessage* checkSuppressedEdgesMsg;

    // FIXME 一下变量需要确认作用
    string curRoadId;   // 用于记录当前道路的id,亦用作判定道路改变
    SMTEdge* lastEdge;  // 记录上一条道路对应Edge
    SMTEdge* curEdge;   // 记录当前道路对应Edge
    SMTEdge* lastPrimaryEdge;
    // double enterLastPrimaryEdge;
    SMTEdge* nextPrimaryEdge;
    cMessage* laneChangeMsg;  // 用于保持车道的消息
    double laneChangeDuration;
    uint8_t preferredLaneIndex;
    bool isChangeAndHold;
    bool isDynamicUpdateRoute;

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
    virtual void handleSuppressMsg(cMessage *msg);

    void startChangeLane(uint8_t laneIndex, double delay = 0);

    void checkSuppressState();
    bool updateVehicleRoute();
    void cmdSetNoOvertake();
    void cmdChangeLane(uint8_t laneIndex, uint32_t duration = 0);
    double cmdGetLanePosition();
    void cmdVehicleArrived();
    void cmdBrake();
    void cmdSpeedDown(double speed = 0);
    void cmdSpeedUp(double speed = -1);
private:
    SMTMap* smtMap;
    SMTCarManager* _pCarManager;
    SMTBaseRouting* _pRouting;
};

#endif /* SMTMOBILITY_H_ */
