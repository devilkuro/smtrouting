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

#include "SMTMobility.h"
#include "StatisticsRecordTools.h"

Define_Module(SMTMobility);

SMTMobility::~SMTMobility() {

}

void SMTMobility::initialize(int stage) {
    Veins::TraCIMobility::initialize(stage);
    if (stage == 0) {
        smtMap = getMap();
        title = title + "external_id" + "\t" + "time" + "\t" + "road_id" + "\t"
                + "record_road" + "\t" + "position";
        isChangeAndHold = par("isChangeAndHold");
        laneChangeDuration = par("laneChangeDuration");
    }
}

void SMTMobility::finish() {
    Veins::TraCIMobility::finish();
}

void SMTMobility::preInitialize(std::string external_id, const Coord& position,
        std::string road_id, double speed, double angle) {
    Enter_Method_Silent
    ();
    Veins::TraCIMobility::preInitialize(external_id, position, road_id, speed,
            angle);
}

void SMTMobility::nextPosition(const Coord& position, std::string road_id,
        double speed, double angle,
        Veins::TraCIScenarioManager::VehicleSignal signals) {
    Enter_Method_Silent
    ();
    Veins::TraCIMobility::nextPosition(position, road_id, speed, angle,
            signals);
    if (!hasRouted) {
        if (getMap()->isReady()) {
            // Map system must be initialized first
            // initialize the route
            processAtRouting();
            hasRouted = true;
        }
    } else {
        // process after the routing
        processAfterRouting();
    }
    // road change
    if (road_id != lastRoadId) {
        // statistics process
        if (!hasInitialized) {
            // when the car first appear on the map.
            processWhenInitializingRoad();
            // switch record process trigger
            hasInitialized = true;
        } else {
            // when road changed
            processWhenChangeRoad();
        }
        // in nextPosition the car has been on the road,
        // then updata the last_road_id and enterTime
        SMTEdge* edge = getMap()->getSMTEdge(road_id);
        if (edge->isInternal) {
            if (!lastEdge->isInternal) {
                lastPrimaryRoadId = lastRoadId;
                lastPrimaryEdge = lastEdge;
            }
        } else {
            curPrimaryRoadId = road_id;
            curPrimaryEdge = edge;
            // recordRoadId = convertStrToRecordId(road_id);
        }
        lastRoadId = road_id;
        lastEdge = edge;
    }
    // normal process
    processWhenNextPosition();
}

void SMTMobility::handleSelfMsg(cMessage* msg) {
    if (msg == laneChangeMsg) {
        handleLaneChangeMsg(msg);
    } else {
        // cancel and delete the unknown message.
        cancelAndDelete(msg);
    }
}

void SMTMobility::processAfterRouting() {
    // 选路之后每个周期都会执行(请确保判定完备,不要执行复杂度过高的操作)
}

void SMTMobility::statisticAtFinish() {
    // 结束时的统计方法
}

void SMTMobility::processAtRouting() {
    // 选路阶段
    // 设置车道变换模式
    cmdSetNoOvertake();
}

void SMTMobility::processWhenChangeRoad() {
    // 当车辆首次进入某条道路时执行
    // TODO 进行Lane控制算法
}

void SMTMobility::processWhenInitializingRoad() {
    // 车辆首次出现在地图上时执行
}

void SMTMobility::processWhenNextPosition() {
    // 车辆变更位置时出现(请确保判定完备,不要执行复杂度过高的操作)

}

void SMTMobility::setPreferredLaneIndex(uint8_t laneIndex) {
    preferredLaneIndex = laneIndex;
}

void SMTMobility::changeToPreferredLane(int laneIndex) {
    if (laneIndex != -1) {
        preferredLaneIndex = (uint8_t) laneIndex;
    }
    startChangeLane(preferredLaneIndex);
}

void SMTMobility::startChangeLane(uint8_t laneIndex, double delay) {
    laneChangeMsg = new cMessage(road_id.c_str(), laneIndex);
    scheduleAt(simTime() + updateInterval + delay, laneChangeMsg);
}

void SMTMobility::cmdSetNoOvertake() {
    getComIf()->setLaneChangeMode(external_id,
            SMTComInterface::LANEMODE_DISALLOW_OVERTAKE);
    if (debug) {
        std::cout << "car " << external_id << " will not make overtake."
                << std::endl;
    }
}

void SMTMobility::cmdChangeLane(uint8_t laneIndex, uint32_t duration) {
    getComIf()->changeLane(external_id, laneIndex, duration * 1000);
}

void SMTMobility::handleLaneChangeMsg(cMessage* msg) {
    uint8_t curLaneIndex = commandGetLaneIndex();
    uint8_t targetLaneIndex = laneChangeMsg->getKind();
    string msgName = laneChangeMsg->getName();
    if (msg == laneChangeMsg) {
        if (msgName == road_id
                && (isChangeAndHold || curLaneIndex != targetLaneIndex)) {
            // 当道路没有改变时,如果需要保持车道或者车道更换为成功则继续尝试
            if (curLaneIndex != targetLaneIndex) {
                // 仅在不在目标车道时进行更改车道的尝试
                cmdChangeLane((uint8_t) laneChangeMsg->getKind(),
                        laneChangeDuration);
            }
            scheduleAt(simTime() + laneChangeDuration + updateInterval,
                    laneChangeMsg);
        } else {
            // 若道路变化,或已成功变道且不需保持车道则不在继续改变车道
            // FIXME 以下内容需要更改为持续变更车道
            // cancel and delete the message if lane changed successfully.
            cancelAndDelete(laneChangeMsg);
            laneChangeMsg = NULL;
        }
    } else {
        std::cout<<"handleLaneChangeMsg can only handle laneChangeMsg."<<std::endl;
        cancelAndDelete(msg);
    }
}

double SMTMobility::cmdGetLanePosition() {
    return getComIf()->getLanePosition(external_id);
}
