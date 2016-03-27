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

#include "SMTSegment.h"

SMTSegment::~SMTSegment() {
    // TODO Auto-generated destructor stub
}

bool SMTSegment::setSegment(const list<double>& durList,
        const list<string>& states, double offset) {
    // FIXME 需要验证offset的用法
    content.resetState(durList, states, offset);
    return updateTimeInfo();
}

bool SMTSegment::updateTimeInfo() {
    if (content.states.size() == 2) {
        // 若只有一种状态
        if (content.states.front().value == "G") {
            t0 = 0;
            tg = content.period;
            ty = 0;
            tr = 0;
            return true;
        } else {
            std::cout << "The state must be 'G' if only one state" << std::endl;
            return false;
        }
    } else {
        // 获取开始于结束的状态
        list<SMTPhaseSegment::State>::iterator uIt = content.states.begin();
        string front = uIt->value;
        uIt = content.states.end();
        --uIt;
        --uIt;
        string back = uIt->value;
        if (front == "G" && back == "G") {
            // 若前后都是绿色,且状态不只一种,则需要合并状态G
            // 通过将其他状态(此处使用'r')移至最前方即可完成
            if (!content.moveCertainStateAHead("r")) {
                if (!content.moveCertainStateAHead("y")) {
                    std::cout << "Unknown issue in SMTSegment::setSegment():"
                            << "more than one state but no 'r' or 'y'"
                            << std::endl;
                    return false;
                }
            }
        }
        content.moveCertainStateAHead("G");
        if (content.states.size() == 3 || content.states.size() == 4) {
            // 不可用重复状态区间,移动后要么为g-y-r-x,要么为g-r-x
            t0 = -1;
            tg = -1;
            ty = -1;
            tr = -1;
            for (list<SMTPhaseSegment::State>::iterator it =
                    content.states.begin(); it != content.states.end(); ++it) {
                if (t0 == -1) {
                    if (it->value == "G") {
                        t0 = it->time;
                    } else {
                        std::cout
                                << "Unknown issue in SMTSegment::setSegment():"
                                << "T0==-1" << std::endl;
                        return false;
                    }
                } else if (tg == -1) {
                    if (it->value == "y") {
                        tg = it->time - t0;
                    } else if (it->value == "r") {
                        tg = it->time - t0;
                        ty = 0;
                    } else {
                        std::cout
                                << "Unknown issue in SMTSegment::setSegment():"
                                << "Tg==-1" << std::endl;
                        return false;
                    }
                } else if (ty == -1) {
                    if (it->value == "r") {
                        ty = it->time - t0 - tg;
                    } else if(it->value=="x"){
                        ty = it->time - t0 - tg;
                        tr = 0;
                    }else {
                        std::cout
                                << "Unknown issue in SMTSegment::setSegment():"
                                << "Ty==-1" << std::endl;
                        return false;
                    }
                } else if (tr == -1) {
                    if (it->value == "x") {
                        tr = it->time - t0 - tg - ty;
                    } else {
                        std::cout
                                << "Unknown issue in SMTSegment::setSegment():"
                                << "Ty==-1" << std::endl;
                        return false;
                    }
                } else {
                    std::cout << "Unknown issue in SMTSegment::setSegment():"
                            << "last" << std::endl;
                    return false;
                }
            }
            // 检测时间长度是否符合state状态
            if (content.head != t0 || tg + ty + tr != content.period
                    || content.head + content.period != content.tail) {
                std::cout << "The time durations are unmatched" << std::endl;
                return false;
            }
        } else {
            std::cout << "Wrong states number" << std::endl;
            return false;
        }
        return true;
    }
}
