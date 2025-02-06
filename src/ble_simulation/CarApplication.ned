//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

package org.car2x.veins.subprojects.ble_simulation;
import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple CarApplication extends DemoBaseApplLayer
{
    @class(ble_simulation::CarApplication);
    @display("i=block/app2");
    
    // Values should be a multiple of 0.625ms and range from 2.5ms to 10.24s
    // Duty-cycle = scanWindow/scanInterval
    double scanInterval =  default(50ms) @unit(s);
    double scanWindow =  default(25ms) @unit(s);
    
    // Threshold value, uses manufacturer receiving sensitivity value but also accounts for PER.
    double minRssi = default(-87.0dBm) @unit(dBm); // Without CRC, NOT using minimum receiver sensitivity as a threshold, but instead a realistic value that has a PER of 1%
	
	// Measurement accuracy of rssi
	double rssiDeviation = default(6dBm) @unit(dBm); // OBSOLETE!
	
	// RSSI at 1 meter (factory-calibrated default, technically sent from advertiser but abstracted and averaged here)
	double calibratedRssi = default(-37.2dBm) @unit(dBm); 
	
	int distanceMode = default(2);
	double rssiTunableError = default(2);
	double rssiTunableKalmanReduction = default(0.27); // Kalman-filtering default 27%
}
