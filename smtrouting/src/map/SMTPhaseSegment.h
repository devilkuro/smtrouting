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

#ifndef SMTPHASESEGMENT_H_
#define SMTPHASESEGMENT_H_

#include<list>
#include<string>

using std::list;
using std::string;

// 用来管理SMTPhase的时间片
class SMTPhaseSegment {
public:
    class State{
    public:
        double time;
        string value;
    };
public:
    SMTPhaseSegment():offset(0),preState("") {
        segIt = segment.begin();
    }
    virtual ~SMTPhaseSegment();

public:
    list<State> segment;
    double offset;
    list<State>::iterator segIt;
    string preState;

    void addState(double start, double end, string value);
    void setStateList(double start, list<double> segLens, list<string> values);
    void setOffset(double offset);
    void moveCertainStateAHead(string value);
};

#endif /* SMTPHASESEGMENT_H_ */
