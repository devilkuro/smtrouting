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

#ifndef __SMTCONNECTION_H_
#define __SMTCONNECTION_H_

#include <string>

using std::string;
/**
 * SMTConnection:连接connection.
 */
class SMTEdge;
class SMTLane;
class SMTTLLogic;
class SMTConnection {
public:
    SMTConnection() :
            fromLane(0), toLane(0), linkIndex(0), fromSMTEdge(0), toSMTEdge(0), fromSMTLane(
                    0), toSMTLane(0), viaSMTLane(0), tlSMTTL(0), t0(-1), tg(-1), ty(
                    -1), tr(-1) {
    }
    virtual ~SMTConnection();
    // xml attributes
    string from;   // The ID of the incoming edge at which the connection begins
    string to;  // The ID of the outgoing edge at which the connection ends
    int fromLane; // The lane of the incoming edge at which the connection begins
    int toLane; // The lane of the outgoing edge at which the connection ends
    string via; // The id of the lane to use to pass this connection across the junction
    string tl; // The id of the traffic light that controls this connection; the attribute is missing if the connection is not controlled by a traffic light
    int linkIndex; // The index of the signal responsible for the connection within the traffic light; the attribute is missing if the connection is not controlled by a traffic light
    string dir; // The direction of the connection. "s" = straight, "t" = turn, "l" = left, "r" = right, "L" = partially left, R = partially right, "invalid" = no direction
    // state is ignored
    // - The state of the connection.
    // - "-" = dead end, "=" = equal,
    // - "m" = minor link, "M" = major link,
    // - traffic light only:
    // - "O" = controller off, "o" = yellow flashing,
    // - "y" = yellow minor link, "Y" = yellow major link,
    // - "r" = red, "g" = green minor, "G" green major

    // SMT attributes
    SMTEdge* fromSMTEdge;
    SMTEdge* toSMTEdge;
    SMTLane* fromSMTLane;
    SMTLane* toSMTLane;
    SMTLane* viaSMTLane;
    SMTTLLogic* tlSMTTL;

    // time information
    double t0, tg, ty, tr;
};
#endif /* __SMTCONNECTION_H_ */
