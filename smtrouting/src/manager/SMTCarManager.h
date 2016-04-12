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

#ifndef __SMTCARMANAGER_H_
#define __SMTCARMANAGER_H_

#include <omnetpp.h>
#include "SMTMap.h"
#include "CarFlowXMLHelper.h"
#include "SMTComInterface.h"

using std::vector;
using std::map;
using std::multimap;
using std::string;
using std::multimap;

class SMTCarManager: public cSimpleModule {
public:
    class MapParameter {
    public:
        MapParameter() :
                maxInnerIndex(0), maxEnterIndex(0), maxOutIndex(0) {
        }
        int maxInnerIndex;
        int maxEnterIndex;
        int maxOutIndex;
    };
    class GeneratorParameter {
    public:
        GeneratorParameter() :
                minGenNumPerHour(0), maxGenNumPerHour(0), startTime(0), prePeriod(
                        0), increasePeriod(0), maxPeriod(0), decreasePeriod(0), sufPeriod(
                        0), generateInterval(0) {
        }
        // _-/'''\-_
        // min-increasePeriod->increase->max-maxPeriod->decrease-decreasePeriod->min
        double minGenNumPerHour;
        double maxGenNumPerHour;
        double startTime;
        double prePeriod;
        double increasePeriod;
        double maxPeriod;
        double decreasePeriod;
        double sufPeriod;
        double generateInterval;
    };
public:
    SMTCarManager() :
            debug(false), endAfterGenerateCarFlowFile(false), _pMap(NULL), _pComIf(
            NULL) {
    }
    virtual ~SMTCarManager();

    map<string, SMTCarInfo*> carMapByID;  // store car instance
    multimap<double, SMTCarInfo*> carMapByTime;

    virtual void generateCarFlowFile(const string &path = "");
    virtual void loadCarFlowFile(const string &path = "");

protected:
    bool debug;
    bool endAfterGenerateCarFlowFile;
    // members
    // configuration
    string carPrefix;
    string XMLPrefix;
    string rouXMLFileName;
    string carFlowXMLFileName;

    // generating car related
    CarFlowXMLHelper carFlowHelper;
    vector<SMTCarInfo*> carInfoVec;
    GeneratorParameter genPar;

    // use for get random edge
    MapParameter mapPar;

    // functions
    virtual int numInitStages() const;
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    SMTMap* getMap();
    SMTComInterface* getComIf();
    virtual void addOneVehicle(SMTCarInfo* car);  // add a car of a certain type

    // generating car related
    // get the car number at certain time
    // the fractional part is returned by remain
    int getGenCarNumAtTime(double time, double &remain);

private:
    SMTMap* _pMap;
    SMTComInterface* _pComIf;
    void releaseCarMap();
};

class SMTCarManagerAccess {
public:
    SMTCarManager* get() {
        return FindModule<SMTCarManager*>::findGlobalModule();
    }
};

#endif /* __SMTCARMANAGER_H_ */
