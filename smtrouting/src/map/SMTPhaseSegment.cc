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

#include <SMTPhaseSegment.h>

SMTPhaseSegment::~SMTPhaseSegment() {
    // TODO Auto-generated destructor stub
}

void SMTPhaseSegment::addState(double start, double end, string value) {
    // 设置segment的某一段为指定value
    // 合并与前方状态一致的节点
    // 寻找起始位置
    if(segIt==segment.end()||segIt->time>start){
        segIt = segment.begin();
        preState = "";
    }
    // 跳过小于start时间点的节点
    while(segIt!=segment.end()&&segIt->time<start){
        preState = segIt->value;
        segIt++;
    }
    // 修改start节点
    if(segIt!=segment.end()&&segIt->time==start){
        // 时间相等进行修改
        if (preState!=value) {
            segIt->value = value;
            segIt++;
        }else{
            segIt = segment.erase(segIt);
        }
    }else{
        // 如果segIt为end或者大于start的节点，且之前状态与value不一致
        // 则需要新建节点
        if (preState!=value) {
            State state;
            state.time = start;
            state.value = value;
            segment.insert(segIt,state);
        }
    }
    // 修改end节点
    // TODO 继续编写addState
}

void SMTPhaseSegment::setOffset(double offset) {
}

void SMTPhaseSegment::setStateList(double start, list<double> segLens,
        list<string> values) {
}

void SMTPhaseSegment::moveCertainStateAHead(string value) {
}
