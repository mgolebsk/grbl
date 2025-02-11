/*
  probe.h - code pertaining to probing methods
  Part of Grbl

  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef probe_h
#define probe_h

// Values that define the probing state machine.
#define PROBE_OFF     0       // Probing disabled or not in use. (Must be zero.)
#define PROBE_Y_BEGIN bit(1)  // Searching for paper top
#define PROBE_Y_END   bit(2)  // Searching for paper bottom
#define PROBE_X_BEGIN bit(3)  // Searching for paper right 
#define PROBE_X_END   bit(4)  // Searching for paper left

// Probe pin initialization routine.
void probe_init();

// Returns probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
uint8_t probe_get_state();

// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
void probe_state_monitor();

#endif
