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

#ifndef __SMTBASEROUTING_H_
#define __SMTBASEROUTING_H_

#include <csimplemodule.h>
#include "SMTMap.h"

class SMTBaseRouting: public cSimpleModule {
public:
    SMTBaseRouting() {
        // TODO Auto-generated constructor stub
    }
    virtual ~SMTBaseRouting();

protected:
    // functions
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    // routing functions
    // TODO 添加基本的寻路方法
    virtual list<SMTEdge*> getShortestRoute(SMTEdge* origin, SMTEdge* destination);
};

#endif /* __SMTBASEROUTING_H_ */
