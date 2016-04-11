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

using std::map;
using std::string;
using std::multimap;

class SMTCarManager: public cSimpleModule {
public:
    SMTCarManager() :
            _pMap(NULL) {
    }
    virtual ~SMTCarManager();

    map<string, SMTCarInfo*> carMapByID;  // store car instance

protected:
    // members
    string carPrefix;
    string XMLPrefix;
    string rouXMLFileName;
    string carFlowFileName;

    // functions
    virtual int numInitStages() const;
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    SMTMap* getMap();
private:
    SMTMap* _pMap;
};

class SMTCarManagerAccess {
public:
    SMTCarManager* get() {
        return FindModule<SMTCarManager*>::findGlobalModule();
    }
};

#endif /* __SMTCARMANAGER_H_ */
