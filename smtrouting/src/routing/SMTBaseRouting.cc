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

#include "SMTBaseRouting.h"

Define_Module(SMTBaseRouting);

SMTBaseRouting::~SMTBaseRouting() {
    // TODO Auto-generated destructor stub

}

void SMTBaseRouting::initialize() {
}

void SMTBaseRouting::handleMessage(cMessage* msg) {
}

void SMTBaseRouting::finish() {
}

list<SMTEdge*> SMTBaseRouting::getShortestRoute(SMTEdge* origin,
        SMTEdge* destination) {
    // 最短路径使用迪杰斯特拉算法
    multimap<double, Route> costMap;
    list<SMTEdge*> rou;

    // TODO 添加选路过程
    return rou;
}