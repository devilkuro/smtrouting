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

#ifndef __FANJING_STATISTICSRECORDTOOLS_H_
#define __FANJING_STATISTICSRECORDTOOLS_H_

#include <map>
#include <list>
#include <string>
#include <stdarg.h>
#include "ctime"
#include <fstream>
#include <iostream>
#include <sstream>
#include "StatisticsRecordUnit.h"
//#include "cmessage.h"

namespace Fanjing {
typedef std::list<StatisticsRecordUnit*> StatisticsRecordUnitList;
typedef std::map<string, StatisticsRecordUnitList*> GlobalStatisticsMap;
/**
 *
 * usage:
 *  gs.changeName(name)<<value0<<value1<<...<<valueN<<gs.endl;
 *  or
 *  gs.changeName(name);
 *  gs.get()<<value0;
 *  gs.get()<<value1;
 *  ...
 *  gs.get()<<valueN;
 *  gs.get()<<gs.endl;
 *
 *
 */

class StatisticsRecordTools {
public:
    typedef void* gs_eofType;
public:
    bool recordWhenTerminate;

    static StatisticsRecordTools * request();
    static void release();

    StatisticsRecordTools& operator<<(gs_eofType& e);
    StatisticsRecordTools& operator<<(double num);
    StatisticsRecordTools& operator<<(int num);
    StatisticsRecordTools& operator<<(unsigned int num);
    StatisticsRecordTools& operator<<(uint64_t num);
    StatisticsRecordTools& operator<<(string str);

    StatisticsRecordTools& changeName(string name, string title = "");
    StatisticsRecordTools& get();

    void setDefaultDir(string dir);
    void eof();
    void clean();
    void output(string name, string dir = "", string field = "",
            std::fstream::openmode openmode = std::fstream::out);
    void outputAll(string name, string dir = "",
            std::fstream::openmode openmode = std::fstream::out);
    void outputSeparate(string name, string dir = "", string field = "",
            std::fstream::openmode openmode = std::fstream::out);
    virtual void initialize();
    virtual void finish();

    static gs_eofType endl;
    std::map<string, double> dblMap;
    //std::map<string, cMessage*> msgMap;
protected:
    GlobalStatisticsMap globalStatisticsMap;
    std::map<string, string> titleMap;
    std::list<StatisticsRecordUnit::DataUnit> unitData;
    string m_name;
    string m_default_dir_name;
private:
    static StatisticsRecordTools* ptr_singleton;
    StatisticsRecordTools();
    virtual ~StatisticsRecordTools();

    string getFileName(string name);
    string getSuffix(string name);
    string getTitleFromName(string name);
};

}  // namespace Fanjing
#endif
