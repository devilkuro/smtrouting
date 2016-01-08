/*
 * GlobalStaticsUnit.h
 *
 *  Created on: 1405051354
 *      Author: Fanjing
 */

#ifndef __FANJING_STATISTICSRECORDUNIT_H_
#define __FANJING_STATISTICSRECORDUNIT_H_

#include <string>
using namespace std;
/**
 *
 * usage:
 *  do not use this class, use StatisticsRecordTools
 */
namespace Fanjing {
class StatisticsRecordUnit {
public:
    StatisticsRecordUnit(int size);
    virtual ~StatisticsRecordUnit();
public:
    enum UnitType {
        UNIT_TYPE_ERROR = -1,
        UNIT_TYPE_INT = 0,
        UNIT_TYPE_UINT32,
        UNIT_TYPE_UINT64,
        UNIT_TYPE_DOUBLE,
        UNIT_TYPE_STRING
    };
    struct DataUnit {
        UnitType type;
        union {
            int intData;
            double douData;
            uint32_t uint32Data;
            uint64_t uint64Data;
        };
        string strData;
        string toString();
    };
public:
    int getDataType(int index);
    void setData(double data, int index);
    void setData(int data, int index);
    void setData(uint32_t data, int index);
    void setData(uint64_t data, int index);
    void setData(string data, int index);
    int getSize() const;
    string toString();

private:
    int size;       // read only, it must be set at initialize phase.
    DataUnit *data;   // statistics data
};

}  // namespace Fanjing
#endif /* GLOBALSTATISTICSUNIT_H_ */
