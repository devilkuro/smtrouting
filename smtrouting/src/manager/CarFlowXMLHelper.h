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

#ifndef __CARFLOWXMLHELPER_H_
#define __CARFLOWXMLHELPER_H_
#include <string>
#include <list>
#include <iostream>
#include "tinyxml2.h"
#include "SMTCarInfo.h"
#include "StringHelper.h"


using namespace std;
using namespace tinyxml2;

class CarFlowXMLHelper {
public:
    CarFlowXMLHelper();
    virtual ~CarFlowXMLHelper();

    bool addODCar(string id, string origin, string destination, double time,
            string vtype);
    bool addLoopCar(string id, list<string> loop, double time, string vtype);

    list<string> getAllCars();
    SMTCarInfo getCar(string id);
    string getRouteTypeOfCar(string id);
    string getOriginOfODCar(string id);
    string getDestinationOfODCar(string id);
    list<string> getLoopOfLoopCar(string id);
    string getCarTypeOFCar(string id);
    double getDepartTimeOfCar(string id);

    // traversal xml files
    SMTCarInfo getFirstCar();
    void setCurrentCar(string id);
    SMTCarInfo getCurrentCar();
    SMTCarInfo getNextCar();
    SMTCarInfo getPreviousCar();

    list<string> switchRouteToRoadList(string route);
    string switchRoadListToRoute(list<string> roadlist);

    // path should have '.xml' as suffix
    int setXMLPath(string path);
    // path should have '.xml' as suffix
    int loadXML(string path);
    void clear(bool save = false);
    // path should have '.xml' as suffix
    void save(string path = "");

    // other
    void setPrecisionOfTime(int precision);
protected:
    XMLElement* root;
    XMLDocument* doc;
    // 用于存放xml路径,此路径为默认保存路径
    string carXMLPath; // path of the xml file, default saving path
    bool notSaved;  // true, if modified but not saved
    // 记录当前车辆节点,用于快速遍历
    XMLElement* curCarElement;

    int precisionOfTime;

    XMLElement* seekCarByAttribute(string name, string value);
    list<string> splitStringToWordsList(string str);
    void finish();

    // xml related functions
    SMTCarInfo getCar(XMLElement* e);
    string getIdOfCar(XMLElement* e);
    string getRouteTypeOfCar(XMLElement* e);
    string getOriginOfODCar(XMLElement* e);
    string getDestinationOfODCar(XMLElement* e);
    list<string> getLoopOfLoopCar(XMLElement* e);
    string getCarTypeOFCar(XMLElement* e);
    double getDepartTimeOfCar(XMLElement* e);
    int initXML();
    int setRoot();
};

#endif /* __CARFLOWXMLHELPER_H_ */
