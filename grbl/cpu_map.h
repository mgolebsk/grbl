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
  #define LIMIT_MASK       (1<<X_LIMIT_BIT) // Only X limit bit
  #define LIMIT_INT        PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect   PCINT0_vect
  #define LIMIT_PCMSK      PCMSK0 // Pin change interrupt register

  #define SENSOR_DDR        DDRB
  #define SENSOR_PIN        PINB
  #define SENSOR_PORT       PORTB
  #define PAPER_SENSOR_IN           4  // Uno Digital Pin 12
  #define PAPER_SENSOR_Y            2  // Uno Digital Pin 10 (old Y_LIMIT_BIT)
  #define PAPER_SENSOR_X            3  // Uno Digital Pin 11 (old Z_LIMIT_BIT)
  #define PEN_SENSOR                5  // Uno Digital Pin 13 (BUILDIN_LED)
  #define SENSOR_MASK     ((1<<PAPER_SENSOR_IN)|(1<<PAPER_SENSOR_Y)|(1<<PAPER_SENSOR_X)) // All sensor bits
  #define PAPER_IS_OUT    (SENSOR_PIN & bit(PAPER_SENSOR_IN))
  #define PAPER_X_IS_OUT  (SENSOR_PIN & bit(PAPER_SENSOR_X))
  #define PAPER_Y_IS_OUT  (SENSOR_PIN & bit(PAPER_SENSOR_Y))
  #define PEN_IS_DOWN     (SENSOR_PIN & bit(PEN_SENSOR))


  // Define user-control controls (cycle start, reset, feed hold) input pins.
  // NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
  #define CONTROL_DDR       DDRC
  #define CONTROL_PIN       PINC
  #define CONTROL_PORT      PORTC
  #define CONTROL_FEED_HOLD_BIT     0  // Uno Analog Pin 0 (orig. Reset / Abort)
  #define CONTROL_LOAD_PAPER_BIT    1  // Uno Analog Pin 1 (orig. Feed Hold)
  #define CONTROL_RESET_BIT         2  // Uno Analog Pin 2 (orig. Cycle Start / Resume)
  #define CONTROL_HOMING_BIT        3  // Uno Analog Pin 3 (orig. Coolant Enable)


  #define C_I2C_SDA       4  // Uno Analog Pin 4  I2C SDA
  #define C_I2C_SCL       5  // Uno Analog Pin 5  I2C SCL

  // FIXME ONLY ANALOG INPUT PINS ADC6 ADC7
  #define C_FIX_ANALOG_TO_USE1       6  // Uno Analog Pin 6  
  #define C_FIX_ANALOG_TO_USE2       7  // Uno Analog Pin 7



  #define CONTROL_INT       PCIE1  // Pin change interrupt enable pin
  #define CONTROL_INT_vect  PCINT1_vect
  #define CONTROL_PCMSK     PCMSK1 // Pin change interrupt register
  #define CONTROL_IRQ_MASK      (bit(CONTROL_RESET_BIT)|bit(CONTROL_FEED_HOLD_BIT)|bit(CONTROL_HOMING_BIT)|bit(CONTROL_LOAD_PAPER_BIT))
  #define CONTROL_PULLUP_MASK   (bit(CONTROL_RESET_BIT)|bit(CONTROL_FEED_HOLD_BIT)|bit(CONTROL_HOMING_BIT)|bit(CONTROL_LOAD_PAPER_BIT)|bit(PEN_SENSOR))
  #define CONTROL_INVERT_MASK   CONTROL_MASK // May be re-defined to only invert certain control pins.

 
  // Define probe switch input pin.
  #define PROBE_DDR       DDRB
  #define PROBE_PIN       PINB
  #define PROBE_PORT      PORTB
  #define PROBE_MASK      (bit(PAPER_SENSOR_X) | bit(PAPER_SENSOR_Y) )

#endif

/*
#ifdef CPU_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and edit one of the available cpu
  // map files and modify it to your needs. Make sure the defined name is also changed in
  // the config.h file.
#endif
*/

#endif
