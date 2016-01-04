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

#ifndef __SMTROUTING_SMTLAUNCHD_H_
#define __SMTROUTING_SMTLAUNCHD_H_

#include <omnetpp.h>
#include "TraCIScenarioManagerLaunchd.h"

/**
 * 用于修改VeinsLaunchd
 */

class SMTComInterface;
class SMTLaunchd: public Veins::TraCIScenarioManagerLaunchd {

public:
    SMTLaunchd() :
            smtComIfc(0) {
    }
    virtual ~SMTLaunchd();

    cXMLElement* getLaunchdConfig();
    SMTComInterface* getSMTComInterface();

protected:
    SMTComInterface* smtComIfc;
};

class SMTLaunchdAccess {
public:
    SMTLaunchd* get() {
        return FindModule<SMTLaunchd*>::findGlobalModule();
    }
};
#endif
