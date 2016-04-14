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

#include "SMTLaunchd.h"
#include "SMTComInterface.h"

Define_Module(SMTLaunchd);

SMTLaunchd::~SMTLaunchd() {
}

cXMLElement* SMTLaunchd::getLaunchdConfig() {
    if (launchConfig == NULL) {
        launchConfig = par("launchConfig").xmlValue();
    }
    return launchConfig;
}

SMTComInterface* SMTLaunchd::getSMTComInterface() {
    if (!smtComIfc) {
        if (connection) {
            smtComIfc = new SMTComInterface(*connection);
        }
    }
    return smtComIfc;
}

void SMTLaunchd::setVehicleArrived(std::string &nodeId) {
    Enter_Method_Silent
    ();
    getSMTComInterface()->setVehicleArrived(nodeId);
    if (debug) {
        EV << "SMTLaunchd reports " << nodeId << " arrived." << endl;
    }

    if (subscribedVehicles.find(nodeId) != subscribedVehicles.end()) {
        subscribedVehicles.erase(nodeId);
        unsubscribeFromVehicleVariables(nodeId);
    }

    // check if this object has been deleted already (e.g. because it was outside the ROI)
    cModule* mod = getManagedModule(nodeId);
    if (mod)
        deleteModule(nodeId);

    if (unEquippedHosts.find(nodeId) != unEquippedHosts.end()) {
        unEquippedHosts.erase(nodeId);
    }

    activeVehicleCount--;
    if ((activeVehicleCount == 0) && autoShutdown)
        autoShutdownTriggered = true;
    drivingVehicleCount--;
}
