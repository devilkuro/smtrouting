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
        beSuppressed = false;
        hasSuppressEdge = false;
        checkSuppressInterval = par("checkSuppressInterval").doubleValue();
        isChangeAndHold = par("isChangeAndHold").boolValue();
        laneChangeDuration = par("laneChangeDuration").doubleValue();
        carInfo = getCarManager()->carMapByID[external_id];
        ASSERT2(carInfo, "undefined car in car manager.");
        origin = getMap()->getSMTEdgeById(carInfo->origin);
        destination = getMap()->getSMTEdgeById(carInfo->destination);
    }
}

void SMTMobility::finish() {
    Veins::TraCIMobility::finish();
    if (laneChangeMsg) {
        cancelAndDelete(laneChangeMsg);
        laneChangeMsg = NULL;
    }
    if (arrivedMsg) {
        cancelAndDelete(arrivedMsg);
        arrivedMsg = NULL;
    }
    if (checkSuppressedEdgesMsg) {
        cancelAndDelete(checkSuppressedEdgesMsg);
        checkSuppressedEdgesMsg = NULL;
    }
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
            if (!getMap()->getSMTEdgeById(road_id)->isInternal) {
                if (processAtRouting()) {
                    hasRouted = true;
                }
            }
        }
    } else {
        // process after the routing
        processAfterRouting();
    }
    // road change
    if (road_id != curRoadId) {
        // statistics process
        if (!hasInitialized) {
            // when the car first appear on the map.
            processWhenInitializingRoad();
            // switch record process trigger
            hasInitialized = true;
        }
        // in nextPosition the car has been on the road
        curRoadId = road_id;
        lastEdge = curEdge;
        if (lastEdge != NULL) {
            if (!lastEdge->isInternal) {
                lastPrimaryEdge = lastEdge;
            }
        }

        if (smtStat.enterEdgeTime > 0) {
            smtStat.enterLastEdgeTime = smtStat.enterEdgeTime;
        }
        smtStat.enterEdgeTime = simTime().dbl();

        curEdge = getMap()->getSMTEdgeById(road_id);
        // when road changed
        processWhenChangeRoad();
    }
    // normal process
    processWhenNextPosition();
}

void SMTMobility::handleSelfMsg(cMessage* msg) {
    if (msg == laneChangeMsg) {
        handleLaneChangeMsg(msg);
    } else if (msg == arrivedMsg) {
        //getComIf()->setVehicleArrived(external_id);
        cancelAndDelete(arrivedMsg);
        arrivedMsg = NULL;
        cmdVehicleArrived();
    } else if (msg == checkSuppressedEdgesMsg) {
        handleSuppressMsg(msg);
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

bool SMTMobility::processAtRouting() {
    // 选路阶段
    // 设置车道变换模式
    cmdSetNoOvertake();
    // 设置路径
    getRouting()->getFastestRoute(getMap()->getSMTEdgeById(road_id),
            destination, carRoute, simTime().dbl(), carInfo);
    return updateVehicleRoute();
}

void SMTMobility::processWhenChangeRoad() {
    // cancel the old lane change message
    cancelAndDelete(laneChangeMsg);
    laneChangeMsg = NULL;
    // 当车辆首次进入某条道路时执行
    if (hasSuppressEdge) {
        getRouting()->releaseEdge(lastEdge);
        hasSuppressEdge = false;
    }
    if (isSlowDown) {
        cmdSpeedUp();
    }
    if (curEdge == destination) {
        // 车辆抵达终点操作
        arrivedMsg = new cMessage("arrived");
        scheduleAt(simTime() + 1, arrivedMsg);
        // change to lane -1 means car arriving
        getRouting()->changeRoad(lastPrimaryEdge, curEdge, -1, simTime().dbl(),
                carInfo, simTime().dbl() - smtStat.outPrimaryEdgeTime);
    } else {
        if (!curEdge->isInternal) {
            while ((*carRoute.begin()) != curEdge) {
                std::cout << "redundant edges in route : "
                        << carRoute.front()->id << std::endl;
                carRoute.pop_front();
            }
            carRoute.pop_front();
            SMTEdge* next = carRoute.front();
            if (curEdge->viaVecMap.find(next) != curEdge->viaVecMap.end()) {
                // FIXME may not always use via 0.
                preferredLaneIndex = curEdge->viaVecMap[next][0]->fromLane;
                startChangeLane(preferredLaneIndex, 2);
            } else {
                std::cout << "next edges " << carRoute.front()->id
                        << " is unlinked at " << curEdge->id << std::endl;
                ASSERT2(!debug, "next edge unlinked ");
            }
            // change road in routing system
            getRouting()->changeRoad(lastPrimaryEdge, curEdge,
                    preferredLaneIndex, simTime().dbl(), carInfo,
                    simTime().dbl() - smtStat.outPrimaryEdgeTime);
        } else {
            // enter internal edge
            ASSERT2(lastEdge!=NULL,
                    "last edge must be primary edge if current edge is internal");
            if (!lastEdge->isInternal) {
                smtStat.outPrimaryEdgeTime = simTime().dbl();
            }
        }
    }
}

void SMTMobility::processWhenInitializingRoad() {
    // 车辆首次出现在地图上时执行
    checkSuppressedEdgesMsg = new cMessage();
    scheduleAt(simTime() + checkSuppressInterval, checkSuppressedEdgesMsg);
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
    scheduleAt(simTime() + delay, laneChangeMsg);
}

void SMTMobility::cmdSetNoOvertake() {
    getComIf()->setLaneChangeMode(external_id,
            SMTComInterface::LANEMODE_FC_NSG_NDOR_ETBS);
    if (debug) {
        std::cout << "car " << external_id << " will not make overtake."
                << std::endl;
    }
}

void SMTMobility::cmdChangeLane(uint8_t laneIndex, uint32_t duration) {
    getComIf()->changeLane(external_id, laneIndex, duration * 1000);
}

void SMTMobility::handleLaneChangeMsg(cMessage* msg) {
    if (msg == laneChangeMsg) {
        string msgName = laneChangeMsg->getName();
        if (getExternalId() == "car1691") {
            laneChangeMsg = msg;
        }
        if (msgName == road_id) {
            uint8_t curLaneIndex = commandGetLaneIndex();
            uint8_t targetLaneIndex = laneChangeMsg->getKind();
            // 当道路没有改变时,如果需要保持车道或者车道更换为成功则继续尝试
            if (isChangeAndHold || curLaneIndex != targetLaneIndex) {
                scheduleAt(simTime() + laneChangeDuration + updateInterval,
                        laneChangeMsg);
            }
            if (curLaneIndex != targetLaneIndex) {
                // 仅在不在目标车道时进行更改车道的尝试
                cmdChangeLane((uint8_t) laneChangeMsg->getKind(),
                        laneChangeDuration);
                if (!hasSuppressEdge) {
                    // suppressing curEdge
                    // if have not changed lane successfully near cross
                    // and has not suppress edge already
                    // FROM 2016-4-23, suppress edge at beginning of edge
                    // slow down and wait to change lane
                    double pos = cmdGetLanePosition();
                    //
                    if (pos > 35 && pos > curEdge->length() * 0.2) {
                        hasSuppressEdge = getRouting()->suppressEdge(curEdge,
                                pos);
                        // FIXME make slow down configurable
                        if (hasSuppressEdge) {
                            cmdSpeedDown(0);
                        } else {
                            cmdSpeedDown(1 + 2 * curLaneIndex);
                        }
                    }
                }
            } else {
                if (isSlowDown) {
                    cmdSpeedUp();
                }
                if (hasSuppressEdge) {
                    getRouting()->releaseEdge(curEdge);
                    hasSuppressEdge = false;
                }
            }
        } else {
            // 若道路变化,或已成功变道且不需保持车道则不在继续改变车道
            // cancel and delete the message if lane changed successfully.
            cancelAndDelete(laneChangeMsg);
            laneChangeMsg = NULL;
        }
    } else {
        std::cout << "handleLaneChangeMsg can only handle laneChangeMsg."
                << std::endl;
        cancelAndDelete(msg);
    }
}

double SMTMobility::cmdGetLanePosition() {
    return getComIf()->getLanePosition(external_id);
}

bool SMTMobility::updateVehicleRoute() {
    if (carRoute.empty()) {
        // change target if no chosen route
        getComIf()->changeVehicleTarget(getExternalId(), destination->id);
        return true;
    } else {
        list<string> strList;
        for (list<SMTEdge*>::iterator it = carRoute.begin();
                it != carRoute.end(); ++it) {
            strList.push_back((*it)->id);
        }
        return getComIf()->changeVehicleRoute(getExternalId(), strList);
    }
}

void SMTMobility::cmdVehicleArrived() {
    getMap()->getLaunchd()->setVehicleArrived(external_id);
}

void SMTMobility::checkSuppressState() {
    // 判定退出压制状态
    if (beSuppressed) {
        const map<SMTEdge*, double> &suppreseedEdgesRef =
                (getRouting()->getSuppressedEdgeMapRef());
        bool speedUpFlag = false;
        map<SMTEdge*, double>::const_iterator it = suppreseedEdgesRef.find(
                curEdge);
        if (it == suppreseedEdgesRef.end()) {
            // 不再压制道路列表内
            speedUpFlag = true;
        } else if (curEdge->isInternal) {
            // enter an internal edge
            speedUpFlag = true;
        } else {
            double lanePos = cmdGetLanePosition();
            if (lanePos > it->second - 10) {
                // get out of the suppressed area
                speedUpFlag = true;
            }
        }
        if (speedUpFlag) {
            cmdSpeedUp();
            beSuppressed = false;
        }
    } else if (!curEdge->isInternal) {
        if (!beSuppressed) {
            const map<SMTEdge*, double> &suppreseedEdgesRef =
                    (getRouting()->getSuppressedEdgeMapRef());
            if (suppreseedEdgesRef.size() > 0) {
                map<SMTEdge*, double>::const_iterator it =
                        suppreseedEdgesRef.find(curEdge);
                if (it != suppreseedEdgesRef.end()) {
                    double lanePos = cmdGetLanePosition();
                    // FIXME make brake distance configurable
                    if (lanePos > it->second - 30
                            && lanePos < it->second - 15) {
                        cmdBrake();
                        beSuppressed = true;
                    }
                }
            }
        }
    }
}

void SMTMobility::handleSuppressMsg(cMessage* msg) {
    checkSuppressState();
    scheduleAt(simTime() + checkSuppressInterval, checkSuppressedEdgesMsg);
}

void SMTMobility::cmdSpeedDown(double speed) {
    if (speed < carInfo->maxSpeed && speed != -1) {
        isSlowDown = true;
    }
    getComIf()->setSpeed(getExternalId(), speed);
}

void SMTMobility::cmdSpeedUp(double speed) {
    if (speed >= carInfo->maxSpeed || speed == -1) {
        isSlowDown = false;
    }
    getComIf()->setSpeed(getExternalId(), speed);
}

void SMTMobility::cmdBrake() {
    getComIf()->setSpeed(getExternalId(), 0);
}
