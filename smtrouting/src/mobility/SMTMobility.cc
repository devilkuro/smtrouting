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

#include <mobility/SMTMobility.h>

SMTMobility::~SMTMobility() {

}

void SMTMobility::initialize(int stage) {
    Veins::TraCIMobility::initialize(stage);
}

void SMTMobility::finish() {
    Veins::TraCIMobility::finish();
}

void SMTMobility::preInitialize(std::string external_id, const Coord& position,
        std::string road_id, double speed, double angle) {
    Veins::TraCIMobility::preInitialize(external_id,position,road_id,speed,angle);
}

void SMTMobility::nextPosition(const Coord& position, std::string road_id,
        double speed, double angle,
        Veins::TraCIScenarioManager::VehicleSignal signals) {
    Veins::TraCIMobility::nextPosition(position,road_id,speed,angle,signals);
}

void SMTMobility::processAfterRouting() {
    // 选路之后每个周期都会执行(请确保判定完备,不要执行复杂度过高的操作)
}

void SMTMobility::statisticAtFinish() {
    // 结束时的统计方法
}

void SMTMobility::processAtRouting() {
    // 选路阶段
}

void SMTMobility::processWhenChangeRoad() {
    // 当车辆首次进入某条道路时执行
}

void SMTMobility::processWhenInitializingRoad() {
    // 车辆首次出现在地图上时执行
}


void SMTMobility::processWhenNextPosition() {
    // 车辆变更位置时出现(请确保判定完备,不要执行复杂度过高的操作)
}



