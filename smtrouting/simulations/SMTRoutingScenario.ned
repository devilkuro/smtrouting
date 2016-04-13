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

package smt.simulations;

import org.car2x.veins.base.modules.BaseWorldUtility;
import org.car2x.veins.modules.obstacle.ObstacleControl;
import org.car2x.veins.modules.world.annotations.AnnotationManager;
import smt.map.SMTMap;
import smt.overrideVeins.SMTLaunchd;
import smt.overrideVeins.SMTConManager;
import smt.manager.SMTCarManager;
import smt.routing.SMTBaseRouting;

network SMTRoutingScenario
{
    parameters:
        double playgroundSizeX @unit(m); // x size of the area the nodes are in (in meters)
        double playgroundSizeY @unit(m); // y size of the area the nodes are in (in meters)
        double playgroundSizeZ @unit(m); // z size of the area the nodes are in (in meters)
        @display("bgb=500,500");
    submodules:
        obstacles: ObstacleControl {
            @display("p=172,24");
        }
        annotations: AnnotationManager {
            @display("p=223,24");
        }
        connectionManager: SMTConManager {
            parameters:
                @display("p=103,24;i=abstract/multicast");
        }
        world: BaseWorldUtility {
            parameters:
                playgroundSizeX = playgroundSizeX;
                playgroundSizeY = playgroundSizeY;
                playgroundSizeZ = playgroundSizeZ;
                @display("p=45,24;i=misc/globe");
        }
        manager: SMTLaunchd {
            parameters:
                @display("p=281,24");
        }
        map: SMTMap {
            @display("p=45,83");
        }
        carmanager: SMTCarManager {
            @display("p=45,136");
        }
        routingMethod: SMTBaseRouting {
            @display("p=45,194");
        }
    connections allowunconnected:
}
