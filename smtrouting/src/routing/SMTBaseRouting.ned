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

package smt.routing;

//
// TODO auto-generated module
//
simple SMTBaseRouting like IBaseRouting
{
    parameters:
        @class(SMTBaseRouting);
        @display("i=block/network2");
        @labels("SMTBaseRouting");

        bool debug = default(false);
        double suppressLength @unit(m) = default(20m);
        double limitStart = default(0.5);
        double limitFix = default(0.01);
        double limitCap = default(0.7);
        // while replace on, work as limitstart
        double airK = default(0.2287);	
        // while replace on, work as limitCap
        double airV = default(0.6352);
        bool replaceAIRWithITSWithOccupancy = default(true);
        bool recordActiveCarNum = default(false);
        double recordActiveCarInterval @unit(s) = default(120s);
        string recordXMLPrefix = default("default_recordXML");
		bool recordHisRoutingData = default(false);
		string hisRecordXMLPath = default("default_hisRecordXML.xml");
		// SMT_RT_SHOREST = 0, SMT_RT_FAST, SMT_RT_AIR, SMT_RT_CORP
        int majorRoutingType = default(1);
        int minorRoutingType = default(2);
        int recordHisRecordRoutingType = default(-1);
}
