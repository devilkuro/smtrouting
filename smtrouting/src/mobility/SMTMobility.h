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

class SMTMobility: public Veins::TraCIMobility {
public:
    SMTMobility() {
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
    virtual void changePosition();
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
