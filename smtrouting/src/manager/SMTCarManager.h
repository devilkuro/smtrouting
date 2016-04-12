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

#ifndef __SMTCARMANAGER_H_
#define __SMTCARMANAGER_H_

#include <omnetpp.h>
#include "SMTMap.h"
#include "CarFlowXMLHelper.h"
#include "SMTComInterface.h"

using std::map;
using std::string;
using std::multimap;

class SMTCarManager: public cSimpleModule {
public:
    SMTCarManager() :
            debug(false), _pMap(NULL), _pComIf(NULL) {
    }
    virtual ~SMTCarManager();

    map<string, SMTCarInfo*> carMapByID;  // store car instance

protected:
    bool debug;
    // members
    string carPrefix;
    string XMLPrefix;
    string rouXMLFileName;
    string carFlowFileName;
    string carFlowXMLFileName;

    // functions
    virtual int numInitStages() const;
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    SMTMap* getMap();
    SMTComInterface* getComIf();
    virtual void addOneVehicle(SMTCarInfo* car);  // add a car of a certain type
private:
    SMTMap* _pMap;
    SMTComInterface* _pComIf;
};

class SMTCarManagerAccess {
public:
    SMTCarManager* get() {
        return FindModule<SMTCarManager*>::findGlobalModule();
    }
};

#endif /* __SMTCARMANAGER_H_ */
