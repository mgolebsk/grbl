/*
  cpu_map.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

/* The cpu_map.h files serve as a central pin mapping selection file for different
   processor types or alternative pin layouts. This version of Grbl officially supports
   only the Arduino Mega328p. */


#ifndef cpu_map_h
#define cpu_map_h

// For cloned PCB
#define KEYS_CLONE true

#ifdef CPU_MAP_ATMEGA328P // (Arduino Uno) Officially supported by Grbl.

  // Define serial port pins and interrupt vectors.
  #define SERIAL_RX     USART_RX_vect
  #define SERIAL_UDRE   USART_UDRE_vect

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define STEP_DDR        DDRD
  #define STEP_PORT       PORTD

  #ifdef KEYS_CLONE
    #define X_STEP_BIT      7  // Uno Digital Pin 7
    #define Y_STEP_BIT      6  // Uno Digital Pin 6
    #define Z_STEP_BIT      5  // Uno Digital Pin 5
    #define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)) // All step bits
  #else
    #define X_STEP_BIT      2  // Uno Digital Pin 2
    #define Y_STEP_BIT      3  // Uno Digital Pin 3
    #define Z_STEP_BIT      4  // Uno Digital Pin 4
    #define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
  #endif


  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define DIRECTION_DDR     DDRD
  #define DIRECTION_PORT    PORTD

  #ifdef KEYS_CLONE
    #define X_DIRECTION_BIT   4  // Uno Digital Pin 4
    #define Y_DIRECTION_BIT   3  // Uno Digital Pin 3
    #define Z_DIRECTION_BIT   2  // Uno Digital Pin 2
    #define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)) // All direction bits
  #else
    #define X_DIRECTION_BIT   5  // Uno Digital Pin 5
    #define Y_DIRECTION_BIT   6  // Uno Digital Pin 6
    #define Z_DIRECTION_BIT   7  // Uno Digital Pin 7
    #define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
  #endif


  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR    DDRB
  #define STEPPERS_DISABLE_PORT   PORTB
  #define STEPPERS_DISABLE_BIT    0  // Uno Digital Pin 8
  #define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)

  // Define homing/hard limit switch input pins and limit interrupt vectors.
  // NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (CONTROL).
  #define LIMIT_DDR        DDRB
  #define LIMIT_PIN        PINB
  #define LIMIT_PORT       PORTB
  #define X_LIMIT_BIT      1  // Uno Digital Pin 9
  #define Y_LIMIT_BIT      2  // Uno Digital Pin 10
  #define Z_LIMIT_BIT      3  // Uno Digital Pin 11
  #if !defined(ENABLE_DUAL_AXIS)
    #define LIMIT_MASK     ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits
  #endif
  #define LIMIT_INT        PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect   PCINT0_vect
  #define LIMIT_PCMSK      PCMSK0 // Pin change interrupt register

  // Define user-control controls (cycle start, reset, feed hold) input pins.
  // NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
  #define CONTROL_DDR       DDRC
  #define CONTROL_PIN       PINC
  #define CONTROL_PORT      PORTC
  #define CONTROL_RESET_BIT         0  // Uno Analog Pin 0
  #define CONTROL_FEED_HOLD_BIT     1  // Uno Analog Pin 1
  #define CONTROL_CYCLE_START_BIT   2  // Uno Analog Pin 2
  #define CONTROL_SAFETY_DOOR_BIT   1  // Uno Analog Pin 1 NOTE: Safety door is shared with feed hold. Enabled by config define.
  #define CONTROL_INT       PCIE1  // Pin change interrupt enable pin
  #define CONTROL_INT_vect  PCINT1_vect
  #define CONTROL_PCMSK     PCMSK1 // Pin change interrupt register
  #define CONTROL_MASK      ((1<<CONTROL_RESET_BIT)|(1<<CONTROL_FEED_HOLD_BIT)|(1<<CONTROL_CYCLE_START_BIT)|(1<<CONTROL_SAFETY_DOOR_BIT))
  #define CONTROL_INVERT_MASK   CONTROL_MASK // May be re-defined to only invert certain control pins.

  // Define probe switch input pin.
  #define PROBE_DDR       DDRC
  #define PROBE_PIN       PINC
  #define PROBE_PORT      PORTC
  #define PROBE_BIT       5  // Uno Analog Pin 5
  #define PROBE_MASK      (1<<PROBE_BIT)

  #ifdef ENABLE_DUAL_AXIS

    // Dual axis feature requires an independent step pulse pin to operate. The independent direction pin is not 
    // absolutely necessary but facilitates easy direction inverting with a Grbl $$ setting. These pins replace 
    // the spindle direction and optional coolant mist pins.

    #ifdef DUAL_AXIS_CONFIG_PROTONEER_V3_51
      // NOTE: Step pulse and direction pins may be on any port and output pin.
      #define STEP_DDR_DUAL       DDRC
      #define STEP_PORT_DUAL      PORTC
      #define DUAL_STEP_BIT       4  // Uno Analog Pin 4
      #define STEP_MASK_DUAL      ((1<<DUAL_STEP_BIT))
      #define DIRECTION_DDR_DUAL  DDRC
      #define DIRECTION_PORT_DUAL PORTC
      #define DUAL_DIRECTION_BIT  3  // Uno Analog Pin 3
      #define DIRECTION_MASK_DUAL ((1<<DUAL_DIRECTION_BIT))

      // NOTE: Dual axis limit is shared with the z-axis limit pin by default. Pin used must be on the same port
      // as other limit pins.
      #define DUAL_LIMIT_BIT    Z_LIMIT_BIT
      #define LIMIT_MASK        ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)|(1<<DUAL_LIMIT_BIT))

    #endif

    // NOTE: Variable spindle not supported with this shield.
    #ifdef DUAL_AXIS_CONFIG_CNC_SHIELD_CLONE
      // NOTE: Step pulse and direction pins may be on any port and output pin.
      #define STEP_DDR_DUAL       DDRB
      #define STEP_PORT_DUAL      PORTB
      #define DUAL_STEP_BIT       4  // Uno Digital Pin 12
      #define STEP_MASK_DUAL      ((1<<DUAL_STEP_BIT))
      #define DIRECTION_DDR_DUAL  DDRB
      #define DIRECTION_PORT_DUAL PORTB
      #define DUAL_DIRECTION_BIT  5  // Uno Digital Pin 13
      #define DIRECTION_MASK_DUAL ((1<<DUAL_DIRECTION_BIT))

      // NOTE: Dual axis limit is shared with the z-axis limit pin by default.
      #define DUAL_LIMIT_BIT    Z_LIMIT_BIT
      #define LIMIT_MASK        ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)|(1<<DUAL_LIMIT_BIT))

    #endif

  #endif

#endif

/*
#ifdef CPU_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and edit one of the available cpu
  // map files and modify it to your needs. Make sure the defined name is also changed in
  // the config.h file.
#endif
*/

#endif
