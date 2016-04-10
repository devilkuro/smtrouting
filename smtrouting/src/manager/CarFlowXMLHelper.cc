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

#include "CarFlowXMLHelper.h"

CarFlowXMLHelper::CarFlowXMLHelper() {
    doc = NULL;
    root = NULL;
    curCarElement = NULL;
    carXMLPath = "";
    notSaved = false;
    precisionOfTime = 1;
}

CarFlowXMLHelper::~CarFlowXMLHelper() {
    finish();
}

// use to change path
int CarFlowXMLHelper::setXMLPath(string path) {
    if (doc == NULL) {
        return loadXML(path);
    }
    this->carXMLPath = path;
    notSaved = true;
    return 0;
}

// load xml file
int CarFlowXMLHelper::loadXML(string path) {
    // result: bit set
    int result = 0;
    // 设置xml path
    if (this->carXMLPath != "" && doc != NULL) {
        // 如果之前已经读取文件则进行备份
        if (notSaved) {
            save(carXMLPath + "_bak");
            result += 4;
        }
    }
    this->carXMLPath = path;
    if (doc == NULL) {
        doc = new XMLDocument();
    }
    XMLError e = doc->LoadFile(path.c_str());
    if (e == XML_ERROR_FILE_NOT_FOUND) {
        initXML();
        result += 2;
    } else {
        // load root element
        result += setRoot();
    }
    return result;
}

list<string> CarFlowXMLHelper::switchRouteToRoadList(string route) {
    return Fanjing::StringHelper::splitStringToWordsList(route);
}

string CarFlowXMLHelper::switchRoadListToRoute(list<string> roadlist) {
    string route = "";
    for (list<string>::const_iterator it = roadlist.begin();
            it != roadlist.end();) {
        route += *it;
        it++;
        if (it != roadlist.end()) {
            route += " ";
        }
    }
    return route;
}
bool CarFlowXMLHelper::addODCar(string id, string origin, string destination,
        double time, string vtype) {
    notSaved = true;
    bool beNewCar = false;
    XMLElement* e = seekCarByAttribute("id", id);
    if (e) {
        // the id is already here.
        if (!e->Attribute("type", "SMTCARINFO_ROUTETYPE_OD")) {
            root->DeleteChild(e);
            e = doc->NewElement("car");
            root->LinkEndChild(e);
        }
    } else {
        // a new id
        e = doc->NewElement("car");
        root->LinkEndChild(e);
        beNewCar = true;
    }
    e->SetAttribute("id", id.c_str());
    e->SetAttribute("type", "SMTCARINFO_ROUTETYPE_OD");
    e->SetAttribute("origin", origin.c_str());
    e->SetAttribute("destination", destination.c_str());
    e->SetAttribute("time",
            Fanjing::StringHelper::dbl2str(time, precisionOfTime).c_str());
    e->SetAttribute("vtype", vtype.c_str());
    return beNewCar;
}

bool CarFlowXMLHelper::addLoopCar(string id, list<string> loop, double time,
        string vtype) {
    notSaved = true;
    bool beNewCar = false;
    string route = switchRoadListToRoute(loop);
    XMLElement* e = seekCarByAttribute("id", id);
    if (e) {
        // the id is already here.
        if (!e->Attribute("type", "SMTCARINFO_ROUTETYPE_LOOP")) {
            root->DeleteChild(e);
            e = doc->NewElement("car");
            root->LinkEndChild(e);
        }
    } else {
        // a new id
        e = doc->NewElement("car");
        root->LinkEndChild(e);
        beNewCar = true;
    }
    e->SetAttribute("id", id.c_str());
    e->SetAttribute("type", "SMTCARINFO_ROUTETYPE_LOOP");
    e->SetAttribute("loop", route.c_str());
    e->SetAttribute("time",
            Fanjing::StringHelper::dbl2str(time, precisionOfTime).c_str());
    e->SetAttribute("vtype", vtype.c_str());
    return beNewCar;
}

string CarFlowXMLHelper::getOriginOfODCar(string id) {
    XMLElement* e = seekCarByAttribute("id", id);
    return getOriginOfODCar(e);
}

string CarFlowXMLHelper::getDestinationOfODCar(string id) {
    XMLElement* e = seekCarByAttribute("id", id);
    return getDestinationOfODCar(e);
}

list<string> CarFlowXMLHelper::getLoopOfLoopCar(string id) {
    XMLElement* e = seekCarByAttribute("id", id);
    return getLoopOfLoopCar(e);
}

XMLElement* CarFlowXMLHelper::seekCarByAttribute(string name, string value) {
    XMLElement* e = root->FirstChildElement("car");
    while (e) {
        if (NULL != e->Attribute(name.c_str(), value.c_str())) {
            return e;
        }
        e = e->NextSiblingElement("car");
    }
    return NULL;
}

list<string> CarFlowXMLHelper::splitStringToWordsList(string str) {
    const string separator = " ";
    list<string> dest;
    string substring;
    string::size_type start = 0, index;

    do {
        index = str.find_first_of(separator, start);
        if (index != string::npos) {
            substring = str.substr(start, index - start);
            dest.push_back(substring);
            start = str.find_first_not_of(separator, index);
            if (start == string::npos) {
                return dest;
            }
        }
    } while (index != string::npos);

    //the last token
    substring = str.substr(start);
    dest.push_back(substring);
    return dest;
}

string CarFlowXMLHelper::getRouteTypeOfCar(string id) {
    XMLElement* e = seekCarByAttribute("id", id);
    return getRouteTypeOfCar(e);
}

string CarFlowXMLHelper::getCarTypeOFCar(string id) {
    XMLElement* e = seekCarByAttribute("id", id);
    return getCarTypeOFCar(e);
}

double CarFlowXMLHelper::getDepartTimeOfCar(string id) {
    XMLElement* e = seekCarByAttribute("id", id);
    return getDepartTimeOfCar(e);
}

list<string> CarFlowXMLHelper::getAllCars() {
    list<string> idList;
    XMLElement* e = root->FirstChildElement("car");
    while (e) {
        if (NULL != e->Attribute("id")) {
            idList.push_back(e->Attribute("id"));
        }
        e = e->NextSiblingElement("car");
    }
    return idList;
}

void CarFlowXMLHelper::clear(bool save) {
    initXML();
    if (save) {
        this->save();
        loadXML(carXMLPath);
    }
}

void CarFlowXMLHelper::save(string path) {
    if (doc == NULL) {
        loadXML(path);
    }
    if (path == "") {
        path = this->carXMLPath;
    }
    string fileName = path;
    doc->SaveFile(fileName.c_str());
    notSaved = false;
}

SMTCarInfo CarFlowXMLHelper::getCar(string id) {
    XMLElement* e = seekCarByAttribute("id", id);
    return getCar(e);
}

void CarFlowXMLHelper::finish() {
    if (notSaved) {
        save();
    }
    if (doc) {
        doc->Clear();
        doc = NULL;
        root = NULL;
    }
}

void CarFlowXMLHelper::setPrecisionOfTime(int precision) {
    this->precisionOfTime = precision;
}

string CarFlowXMLHelper::getRouteTypeOfCar(XMLElement* e) {
    if (e != NULL) {
        return e->Attribute("type") == NULL ? "" : e->Attribute("type");
    }
    return "";
}

string CarFlowXMLHelper::getOriginOfODCar(XMLElement* e) {
    if (e != NULL) {
        if (e->Attribute("type", "SMTCARINFO_ROUTETYPE_OD")) {
            if (e->Attribute("origin")) {
                return e->Attribute("origin");
            }
        }
    }
    return "";
}

string CarFlowXMLHelper::getDestinationOfODCar(XMLElement* e) {
    if (e != NULL) {
        if (e->Attribute("type", "SMTCARINFO_ROUTETYPE_OD")) {
            if (e->Attribute("destination")) {
                return e->Attribute("destination");
            }
        }
    }
    return "";
}

list<string> CarFlowXMLHelper::getLoopOfLoopCar(XMLElement* e) {
    list<string> result;
    if (e != NULL) {
        if (e->Attribute("type", "SMTCARINFO_ROUTETYPE_LOOP")) {
            if (e->Attribute("loop")) {
                string route = e->Attribute("loop");
                if (route != "") {
                    return switchRouteToRoadList(route);
                } else {
                    cout << "empty route:" << route << endl;
                }
            }
        }
    }
    return result;
}

string CarFlowXMLHelper::getCarTypeOFCar(XMLElement* e) {
    if (e != NULL) {
        return e->Attribute("vtype") == NULL ? "" : e->Attribute("vtype");
    }
    return "";
}

SMTCarInfo CarFlowXMLHelper::getCar(XMLElement* e) {
    SMTCarInfo car;
    if (e != NULL) {
        string vtype = getCarTypeOFCar(e);
        string rtype = getRouteTypeOfCar(e);
        // set the vType related parameters first
        if (SMTCarInfo::hasInitialized()) {
            car = SMTCarInfo::getDefaultVeicleTypeInfo(vtype);
        }
        car.id = getIdOfCar(e);
        car.vtype = vtype;
        car.time = getDepartTimeOfCar(e);
        // read the default info of the vtype
        if (rtype == "SMTCARINFO_ROUTETYPE_OD") {
            car.type = SMTCarInfo::SMTCARINFO_ROUTETYPE_OD;
            car.origin = getOriginOfODCar(e);
            car.destination = getDestinationOfODCar(e);
        } else if (rtype == "SMTCARINFO_ROUTETYPE_LOOP") {
            car.type = SMTCarInfo::SMTCARINFO_ROUTETYPE_LOOP;
            car.loop = switchRoadListToRoute(getLoopOfLoopCar(e));
        } else {
            car.type = SMTCarInfo::SMTCARINFO_ROUTETYPE_LAST_TYPE;
        }
    }
    return car;
}

SMTCarInfo CarFlowXMLHelper::getFirstCar() {
    SMTCarInfo car;
    if (root != NULL) {
        curCarElement = root->FirstChildElement("car");
        return getCar(curCarElement);
    }
    return car;
}

void CarFlowXMLHelper::setCurrentCar(string id) {
    curCarElement = seekCarByAttribute("id", id);
}

SMTCarInfo CarFlowXMLHelper::getCurrentCar() {
    return getCar(curCarElement);
}

SMTCarInfo CarFlowXMLHelper::getNextCar() {
    if (curCarElement != NULL) {
        curCarElement = curCarElement->NextSiblingElement("car");
    }
    return getCurrentCar();
}

SMTCarInfo CarFlowXMLHelper::getPreviousCar() {
    if (curCarElement != NULL) {
        curCarElement = curCarElement->PreviousSiblingElement("car");
    }
    return getCurrentCar();
}

string CarFlowXMLHelper::getIdOfCar(XMLElement* e) {
    if (e != NULL) {
        return e->Attribute("id") == NULL ? "" : e->Attribute("id");
    }
    return "";
}

double CarFlowXMLHelper::getDepartTimeOfCar(XMLElement* e) {
    if (e != NULL) {
        return e->Attribute("time") == NULL ? 0 : e->DoubleAttribute("time");
    }
    return 0;
}

int CarFlowXMLHelper::initXML() {
    if (doc != NULL) {
        doc->Clear();
        XMLDeclaration* dec = doc->NewDeclaration();
        doc->LinkEndChild(dec);
        root = doc->NewElement("Cars");
        doc->LinkEndChild(root);
        notSaved = true;
        return 0;
    }
    return -1;
}

int CarFlowXMLHelper::setRoot() {
    if (doc != NULL) {
        root = doc->FirstChildElement("Cars");
        if (root == NULL) {
            root = doc->NewElement("Cars");
            doc->LinkEndChild(root);
            notSaved = true;
            return 1;
        }
        return 0;
    }
    return -1;
}
