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

package org.car2x.veins.subprojects.ble_simulation;

import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple VruApplication extends DemoBaseApplLayer
{
    parameters:
    	@class(ble_simulation::VruApplication);
    	@display("i=block/app2");
    	
    	// AdvInterval should be a multiple of 0.625ms
    	double advInterval =  default(100ms) @unit(s); // Between 20ms and 10.24s
    	double minAdvDelay = default(0ms) @unit(s);
    	double maxAdvDelay = default(20ms) @unit(s);
}