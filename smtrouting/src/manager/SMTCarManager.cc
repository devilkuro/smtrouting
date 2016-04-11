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

#include "SMTCarManager.h"

Define_Module(SMTCarManager);

SMTCarManager::~SMTCarManager() {
    for (map<string, SMTCarInfo*>::iterator it = carMapByID.begin();
            it != carMapByID.end(); ++it) {
        delete (it->second);
    }
}

int SMTCarManager::numInitStages() const {
    return 2;
}

void SMTCarManager::initialize(int stage) {
    if (stage == 0) {
        // set configuration
    }
    if (stage == 1) {
        // set map and car info
    }
}

void SMTCarManager::handleMessage(cMessage* msg) {
}

void SMTCarManager::finish() {
}

SMTMap* SMTCarManager::getMap() {
    if (_pMap == NULL) {
        _pMap = SMTCarManagerAccess().get();
    }
    return _pMap;
}
