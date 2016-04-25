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
        double limitFix = default(0.69);
        double limitCap = default(0.7);
        bool recordActiveCarNum = default(false);

		// SMT_RT_SHOREST = 0, SMT_RT_FAST, SMT_RT_AIR, SMT_RT_CORP
        int majorRoutingType = default(1);
        int minorRoutingType = default(2);
}
