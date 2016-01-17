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

#ifndef SMTCOMINTERFACE_H_
#define SMTCOMINTERFACE_H_

#include "TraCICommandInterface.h"

class SMTComInterface: public Veins::TraCICommandInterface {

public:
    SMTComInterface(Veins::TraCIConnection& con);
    virtual ~SMTComInterface();
    enum SMTLaneChangeMode {
        LANEMODE_NO_STRATEGIC = 0x00,
        LANEMODE_ALLOW_STRATEGIC = 0x01,
        LANEMODE_FORCE_STRATEGIC = 0x02,
        LANEMODE_NO_COOPERATIVE = 0x00 << 2,
        LANEMODE_ALLOW_COOPERATIVE = 0x01 << 2,
        LANEMODE_FORCE_COOPERATIVE = 0x02 << 2,
        LANEMODE_NO_SPEED_GAIN = 0x00 << 4,
        LANEMODE_ALLOW_SPEED_GAIN = 0x01 << 4,
        LANEMODE_FORCE_SPEED_GAIN = 0x02 << 4,
        LANEMODE_NO_DRIVE_ON_RIGHT = 0x00 << 6,
        LANEMODE_ALLOW_DRIVE_ON_RIGHT = 0x01 << 6,
        LANEMODE_FORCE_DRIVE_ON_RIGHT = 0x02 << 6,
        LANEMODE_EXT_TRACI_IGNORE_OTHERS = 0x00 << 8,
        LANEMODE_EXT_TRACI_AVOID_COLLISIONS = 0x01 << 8,
        LANEMODE_EXT_TRACI_BY_SPEED = 0x10 << 8,
        LANEMODE_EXT_TRACI_NATURAL = 0x11 << 8,
        LANEMODE_ALLOW_ALL = LANEMODE_ALLOW_STRATEGIC
                | LANEMODE_ALLOW_COOPERATIVE | LANEMODE_ALLOW_SPEED_GAIN
                | LANEMODE_ALLOW_DRIVE_ON_RIGHT,
        LANEMODE_DISALLOW_ALL = LANEMODE_NO_STRATEGIC | LANEMODE_NO_COOPERATIVE
                | LANEMODE_NO_SPEED_GAIN | LANEMODE_NO_DRIVE_ON_RIGHT,
        LANEMODE_DISALLOW_OVERTAKE = LANEMODE_FORCE_STRATEGIC
                | LANEMODE_FORCE_COOPERATIVE | LANEMODE_NO_SPEED_GAIN
                | LANEMODE_NO_DRIVE_ON_RIGHT
                | LANEMODE_EXT_TRACI_BY_SPEED
    };
    // 获取Lane连接的下一跳lane列表
    std::list<std::string> getLaneLinkedLaneIds(std::string laneId);
    // 设置车辆变道模式
    void setLaneChangeMode(std::string nodeId, SMTLaneChangeMode mode);
    // 变更车辆行驶车道
    void commandChangeLane(std::string nodeId, uint8_t laneIndex, uint32_t duration = 0);
protected:
    Veins::TraCIConnection& smtConnection;
};

#endif /* SMTCOMINTERFACE_H_ */
