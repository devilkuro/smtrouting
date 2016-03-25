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

#include "SMTPhaseSegment.h"

SMTPhaseSegment::~SMTPhaseSegment() {
    // TODO Auto-generated destructor stub
}

void SMTPhaseSegment::setState(double start, double end, string value) {
    // 设置segment的某一段为指定value
    // 合并与前方状态一致的节点
    // 寻找起始位置
    if (segIt == segment.end() || segIt->time > start) {
        segIt = segment.begin();
        preState = "x";
    }
    // 跳过小于start时间点的节点
    while (segIt != segment.end() && segIt->time < start) {
        preState = segIt->value;
        segIt++;
    }
    // 修改start节点
    string originState = preState; // 用于记录end时间后原始的状态
    if (segIt != segment.end() && segIt->time == start) {
        // 时间相等进行修改
        // 记录原始后续状态(理论上原始后续状态肯定与前方状态不同)
        // (因为相同的节点应该被删除)
        originState = segIt->value;
        if (preState != value) {
            // 若start后状态与之前状态不一致,则修改当前节点
            // 修改节点后续状态
            segIt->value = value;
            // 同步迭代器与preState状态,迭代器后移
            preState = value;
            segIt++;
        } else {
            // 若修改后的状态与start之前状态一致,则删除该节点
            // 此时preState与originState均未改变
            segIt = segment.erase(segIt);
        }
    } else {
        // 如果segIt为end()或者大于start的节点,且之前状态与value不一致
        // 则需要新建节点,反之不需要修改
        if (preState != value) {
            State state;
            state.time = start;
            state.value = value;
            segment.insert(segIt, state);
            // 记录原始后续状态
            originState = preState;
            // 同步迭代器与preState状态,
            preState = value;
        }
    }
    // 删除所有start之后,end之前的状态
    while (segIt != segment.end() && segIt->time < end) {
        // 保存后续状态
        originState = segIt->value;
        segIt = segment.erase(segIt);
    }
    // 修改end节点
    if (segIt != segment.end() && segIt->time == end) {
        // 若存在时间为end的节点
        // end后的状态与value状态一致,则删除该节点
        // 反之不做修改
        if (preState == segIt->value) {
            // 此时preState未改变
            segIt = segment.erase(segIt);
        }
    } else {
        // 若segIt为end()或大于end的节点,且end前后状态不同
        // 则新建节点,若前后状态一致,则不做改变
        if (preState != originState) {
            State state;
            state.time = end;
            state.value = originState;
            segment.insert(segIt, state);
            // 为便于连续修改segment状态,迭代器返回end时刻
            segIt--;
            // 返回后preState状态未改变
        }
    }
}

void SMTPhaseSegment::setOffset(double offset) {

}

void SMTPhaseSegment::resetState(list<double> segLens, list<string> values,
        double offset) {
    if(segLens.size()!=values.size()){
        std::cout<<"unmatched segLens and values."<<std::endl;
    }
    // 重置状态线段
    segment.clear();
    double start = 0;
    list<double>::iterator lenIt = segLens.begin();
    list<string>::iterator valIt = values.begin();
    // 依此设置各状态段落
    while(lenIt!=segLens.end()&&valIt!=values.end()){
        double end = start + *lenIt;
        setState(start,end,*valIt);
        start = end;
        lenIt++;
        valIt++;
    }
}

bool SMTPhaseSegment::moveCertainStateAHead(string value) {
}

void SMTPhaseSegment::debugPrint() {
    std::cout<<std::endl<<"debug: print the segment content."<<std::endl;
    std::cout<<"offset:"<<offset<<" ";
    for(list<State>::iterator it = segment.begin();it!=segment.end();){
        std::cout<<it->time<<":"<<it->value;
        ++it;
        if(it!=segment.end()){
            std::cout<<", ";
        }
    }
    std::cout<<std::endl;
}
