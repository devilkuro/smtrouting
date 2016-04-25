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

package smt.manager;

//
// TODO auto-generated type
//
simple SMTCarManager
{
    parameters:
        @class(SMTCarManager);
        @display("i=block/network2");
        @labels("SMTCarManager");

        bool debug = default(false);
        bool endAfterGenerateCarFlowFile = default(true);
        string carPrefix = default("car");
        string XMLPrefix = default("smt");
        string rouXMLFileName = default("");
        string carFlowXMLFileName = default("");

        double minGenNumPerHour = default(1000);
        double maxGenNumPerHour = default(20000);
        double startTime @unit(s) = default(1s);
        double prePeriod @unit(s) = default(20min);
        double increasePeriod @unit(s) = default(20min);
        double maxPeriod @unit(s) = default(40min);
        double decreasePeriod @unit(s) = default(20min);
        double sufPeriod @unit(s) = default(100min);
        double generateInterval @unit(s) = default(1s);
        double crossRatio = default(0.5);
        double innerRatio = default(0.5);
        bool forceGenerate = default(false);
}
