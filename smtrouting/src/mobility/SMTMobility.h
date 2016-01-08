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

class SMTMobility: public Veins::TraCIMobility {
public:
    SMTMobility() :
            smtMap(0), hasRouted(false), hasInitialized(false), state_enterEdgeTime(
                    0), state_enterQueueTime(0), state_outQueueTime(0), state_outEdgeTime(
                    0), state_enterNextEdgeTime(0) {
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
protected:
    // 接口成员
    SMTMap* smtMap;
    SMTMap* getMap(){
        if(smtMap==NULL){
            smtMap = SMTMapAccess().get();
        }
        return smtMap;
    }
    // 中间过程变量
    bool hasRouted; // 用于判定是否已经分配路径
    bool hasInitialized;    // 用于判定是否已经于地图上初始化
    string last_road_id;    // 用于记录上一条道路的id
    string last_primary_road_id;    // 记录上一条主要道路id
    simtime_t last_droveAt; // 用于记录上一次停车的时间

    // 统计相关
    double state_enterEdgeTime; // 进入edge时间
    double state_enterQueueTime;    // 进入队列区时间
    double state_outQueueTime;  // 启动准备离开队列区
    double state_outEdgeTime;   // 离开edge时间
    double state_enterNextEdgeTime; // 进入下一条edge时间

    // overload these function in different mobility
    // processAfterRouting
    // this function will run every 0.1 second for each car if routed in routing process!!
    // so, do not do any complicated operations here.
    virtual void processAfterRouting();
    // statisticAtFinish
    virtual void statisticAtFinish();
    // initialize the route in routing process
    virtual void processAtRouting();
    // when road changed from one road to another, not the first time appearing on the map.
    virtual void processWhenChangeRoad();
    // when the car first appear on the map.
    virtual void processWhenInitializingRoad();
    // this function will run every 0.1 second for each car!!
    // so, do not do any complicated operations here.
    virtual void processWhenNextPosition();

};

#endif /* SMTMOBILITY_H_ */
