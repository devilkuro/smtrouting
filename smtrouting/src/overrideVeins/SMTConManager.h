/*
 * SMTConManager.h
 *
 *  Created on: 2016年1月1日
 *      Author: Fanjing-LAB
 */

#ifndef SMTCONMANAGER_H_
#define SMTCONMANAGER_H_

#include "MiXiMDefs.h"
#include "ConnectionManager.h"

class SMTConManager: public ConnectionManager {
public:
    SMTConManager() :
            disableConnectionUpdate(false) {
    }
    virtual ~SMTConManager();

    virtual void initialize(int stage);
protected:
    bool disableConnectionUpdate;

    virtual double calcInterfDist();
    virtual void updateConnections(int nicID, const Coord* oldPos,
            const Coord* newPos);
};

#endif /* SMTCONMANAGER_H_ */
