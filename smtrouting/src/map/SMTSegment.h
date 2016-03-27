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

#ifndef __MAP_SMTSEGMENT_H_
#define __MAP_SMTSEGMENT_H_

#include "SMTPhaseSegment.h"

/**
 * SMTSegment:某connection的通行状态.
 */
class SMTSegment {
public:
    SMTSegment():T0(0),Tg(0),Ty(0),Tr(0) {
    }
    virtual ~SMTSegment();

    SMTPhaseSegment content;
    double T0; // the start time of the green light
    double Tg; // the duration of green light
    double Ty; // the duration of yellow light
    double Tr; // the duration of red light

    bool setSegment(const list<double> &durList, const list<string> &states, double offset);
    bool updateTimeInfo();
};

#endif /* __MAP_SMTSEGMENT_H_ */

