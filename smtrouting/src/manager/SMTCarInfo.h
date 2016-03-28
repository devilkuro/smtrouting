/*
 * SMTCarInfo.h
 *
 *  Created on: Oct 7, 2015
 *      Author: Fanjing-LAB
 */

#ifndef SMTCARINFO_H_
#define SMTCARINFO_H_

#include <string>
#include <map>
#include <list>
#include "tinyxml2.h"
#include "StringHelper.h"

namespace Fanjing {

/*
 *
 */

using namespace std;
using namespace tinyxml2;
class SMTCarInfo {
public:
    enum RouteType {
        SMTCARINFO_ROUTETYPE_OD = 0, // normal route: from origin to destination
        SMTCARINFO_ROUTETYPE_LOOP,   // cyclic route: A-..-B-..-..-X(-..-A-..)
        SMTCARINFO_ROUTETYPE_LAST_TYPE
    };
public:
    SMTCarInfo();
    virtual ~SMTCarInfo();
    static bool hasInitialized();
    static void loadVehicleTypeXML(string path);
    static void release();
    // return car with route type SMTCARINFO_ROUTETYPE_LAST_TYPE, if no car own this type id
    static SMTCarInfo getDefaultVeicleTypeInfo(string vTypeId);
    // load vehicle type xml first
    static list<string> getDefaultVeicleTypeList();

    string toString();
public:
    string id;
    RouteType type;
    string origin;
    string destination;
    string loop;
    double time;
    string vtype;

    // type related
    double accel;
    double decel;
    double sigma;
    double length;
    double minGap;
    double maxSpeed;
    string color;   // useless

private:
    static string path;
    static XMLDocument* doc;
    static map<string, SMTCarInfo> vTypeMap;
};

} /* namespace Fanjing */
#endif /* SMTCARINFO_H_ */
