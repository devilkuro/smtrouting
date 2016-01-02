/*
 * SMTConManager.cc
 *
 *  Created on: 2016年1月1日
 *      Author: Fanjing-LAB
 */

#include <SMTConManager.h>

Define_Module(SMTConManager);
SMTConManager::~SMTConManager() {
}

double SMTConManager::calcInterfDist() {
    if (disableConnectionUpdate) {
        return playgroundSize->x > playgroundSize->y ?
                playgroundSize->x : playgroundSize->y;
    } else {
        return ConnectionManager::calcInterfDist();
    }
}

void SMTConManager::initialize(int stage) {
    if (stage == 0) {
        disableConnectionUpdate =
                hasPar("disableConnectionUpdate") ?
                        par("disableConnectionUpdate").boolValue() : false;
    }
    ConnectionManager::initialize(stage);
}

void SMTConManager::updateConnections(int nicID, const Coord* oldPos,
        const Coord* newPos) {
    if (disableConnectionUpdate) {
        // do nothing
    } else {
        ConnectionManager::updateConnections(nicID, oldPos, newPos);
    }
}
