/*
  protocol.c - controls Grbl execution protocol and procedures
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include "grbl.h"

// Define line flags. Includes comment type tracking and line overflow detection.
#define LINE_FLAG_OVERFLOW bit(0)
#define LINE_FLAG_COMMENT_PARENTHESES bit(1)
#define LINE_FLAG_COMMENT_SEMICOLON bit(2)


static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.

static void protocol_exec_rt_suspend();


/*
  GRBL PRIMARY LOOP:
*/
void protocol_main_loop()
{
  // Perform some machine checks to make sure everything is good to go.
  #ifdef CHECK_LIMITS_AT_INIT
    if (bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE)) {
      if (limit_get_state()) {
        sys.state = STATE_ALARM; // Ensure alarm state is active.
        report_feedback_message(MESSAGE_CHECK_LIMITS);
      }
    }
  #endif
  // Check for and report alarm state after a reset, error, or an initial power up.
  // NOTE: Sleep mode disables the stepper drivers and position can't be guaranteed.
  // Re-initialize the sleep state as an ALARM mode to ensure user homes or acknowledges.
  if (sys.state & (STATE_ALARM | STATE_SLEEP)) {
    report_feedback_message(MESSAGE_ALARM_LOCK);
    sys.state = STATE_ALARM; // Ensure alarm state is set.
  } else {
    // Check if the safety door is open.
    sys.state = STATE_IDLE;
    // All systems go!
    system_execute_startup(line); // Execute startup script.
  }
      
  led_update();

  // ---------------------------------------------------------------------------------
  // Primary loop! Upon a system abort, this exits back to main() to reset the system.
  // This is also where Grbl idles while waiting for something to do.
  // ---------------------------------------------------------------------------------

  uint8_t line_flags = 0;
  uint8_t char_counter = 0;
  uint8_t c;
  for (;;) {

    // Process one line of incoming serial data, as the data becomes available. Performs an
    // initial filtering by removing spaces and comments and capitalizing all letters.
    while((c = serial_read()) != SERIAL_NO_DATA) {
      if ((c == '\n') || (c == '\r')) { // End of line reached

        protocol_execute_realtime(); // Runtime command check point.
        if (sys.abort) { return; } // Bail to calling function upon system abort

        line[char_counter] = 0; // Set string termination character.
        #ifdef REPORT_ECHO_LINE_RECEIVED
          report_echo_line_received(line);
        #endif

        // Direct and execute one line of formatted input, and report status of execution.
        if (line_flags & LINE_FLAG_OVERFLOW) {
          // Report line overflow error.
          report_status_message(STATUS_OVERFLOW);
        } else if (line[0] == 0) {
          // Empty or comment line. For syncing purposes.
          report_status_message(STATUS_OK);
        } else if (line[0] == '$') {
          // Grbl '$' system command
          report_status_message(system_execute_line(line));
        } else if (sys.state & (STATE_ALARM | STATE_JOG)) {
          // Everything else is gcode. Block if in alarm or jog mode.
          report_status_message(STATUS_SYSTEM_GC_LOCK);
        } else {
          // Parse and execute g-code block.
          report_status_message(gc_execute_line(line));
        }

        // Reset tracking data for next line.
        line_flags = 0;
        char_counter = 0;

      } else {

        if (line_flags) {
          // Throw away all (except EOL) comment characters and overflow characters.
          if (c == ')') {
            // End of '()' comment. Resume line allowed.
            if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) { line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES); }
          }
        } else {
          if (c <= ' ') {
            // Throw away whitepace and control characters
          } else if (c == '/') {
            // Block delete NOT SUPPORTED. Ignore character.
            // NOTE: If supported, would simply need to check the system if block delete is enabled.
          } else if (c == '(') {
            // Enable comments flag and ignore all characters until ')' or EOL.
            // NOTE: This doesn't follow the NIST definition exactly, but is good enough for now.
            // In the future, we could simply remove the items within the comments, but retain the
            // comment control characters, so that the g-code parser can error-check it.
            line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
          } else if (c == ';') {
            // NOTE: ';' comment to EOL is a LinuxCNC definition. Not NIST.
            line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
          // TODO: Install '%' feature
          // } else if (c == '%') {
            // Program start-end percent sign NOT SUPPORTED.
            // NOTE: This maybe installed to tell Grbl when a program is running vs manual input,
            // where, during a program, the system auto-cycle start will continue to execute
            // everything until the next '%' sign. This will help fix resuming issues with certain
            // functions that empty the planner buffer to execute its task on-time.
          } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
            // Detect line buffer overflow and set flag.
            line_flags |= LINE_FLAG_OVERFLOW;
          } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        }

      }
    }

    // If there are no more characters in the serial read buffer to be processed and executed,
    // this indicates that g-code streaming has either filled the planner buffer or has
    // completed. In either case, auto-cycle start, if enabled, any queued moves.
    protocol_auto_cycle_start();

    protocol_execute_realtime();  // Runtime command check point.
    if (sys.abort) { return; } // Bail to main() program loop to reset system.
  }

  return; /* Never reached */
}


// Block until all buffered steps are executed or in a cycle state. Works with feed hold
// during a synchronize call, if it should happen. Also, waits for clean cycle end.
void protocol_buffer_synchronize()
{
  // If system is queued, ensure cycle resumes if the auto start flag is present.
  protocol_auto_cycle_start();
  do {
    protocol_execute_realtime();   // Check and execute run-time commands
    if (sys.abort) { return; } // Check for system abort
  } while (plan_get_current_block() || (sys.state == STATE_CYCLE));
}


// Auto-cycle start triggers when there is a motion ready to execute and if the main program is not
// actively parsing commands.
// NOTE: This function is called from the main loop, buffer sync, and mc_line() only and executes
// when one of these conditions exist respectively: There are no more blocks sent (i.e. streaming
// is finished, single commands), a command that needs to wait for the motions in the buffer to
// execute calls a buffer sync, or the planner buffer is full and ready to go.
void protocol_auto_cycle_start()
{
  if (plan_get_current_block() != NULL) { // Check if there are any blocks in the buffer.
    system_set_exec_state_flag(EXEC_CYCLE_START); // If so, execute them!
  }
}


// This function is the general interface to Grbl's real-time command execution system. It is called
// from various check points in the main program, primarily where there may be a while loop waiting
// for a buffer to clear space or any point where the execution time from the last check point may
// be more than a fraction of a second. This is a way to execute realtime commands asynchronously
// (aka multitasking) with grbl's g-code parsing and planning functions. This function also serves
// as an interface for the interrupts to set the system realtime flags, where only the main program
// handles them, removing the need to define more computationally-expensive volatile variables. This
// also provides a controlled way to execute certain tasks without having two or more instances of
// the same task, such as the planner recalculating the buffer upon a feedhold or overrides.
// NOTE: The sys_rt_exec_state variable flags are set by any process, step or serial interrupts, pinouts,
// limit switches, or the main program.
void protocol_execute_realtime()
{
  protocol_exec_rt_system();
  if (sys.suspend) { protocol_exec_rt_suspend(); }
}


// Executes run-time commands, when required. This function primarily operates as Grbl's state
// machine and controls the various real-time features Grbl has to offer.
// NOTE: Do not alter this unless you know exactly what you are doing!
void protocol_exec_rt_system()
{
 
  uint8_t rt_exec; // Temp variable to avoid calling volatile multiple times.
  rt_exec = sys_rt_exec_alarm; // Copy volatile sys_rt_exec_alarm.
  if (rt_exec) { // Enter only if any bit flag is true
    // System alarm. Everything has shutdown by something that has gone severely wrong. Report
    // the source of the error to the user. If critical, Grbl disables by entering an infinite
    // loop until system reset/abort.
    sys.state = STATE_ALARM; // Set system alarm state
    report_alarm_message(rt_exec);
    // Halt everything upon a critical event flag. Currently hard and soft limits flag this.
    if ((rt_exec == EXEC_ALARM_HARD_LIMIT) || (rt_exec == EXEC_ALARM_SOFT_LIMIT)) {
      report_feedback_message(MESSAGE_CRITICAL_EVENT);
      system_clear_exec_state_flag(EXEC_RESET); // Disable any existing reset
      do {
        led_update();
        // Block everything, except reset and status reports, until user issues reset or power
        // cycles. Hard limits typically occur while unattended or not paying attention. Gives
        // the user and a GUI time to do what is needed before resetting, like killing the
        // incoming stream. The same could be said about soft limits. While the position is not
        // lost, continued streaming could cause a serious crash if by chance it gets executed.
      } while (bit_isfalse(sys_rt_exec_state,EXEC_RESET));
    }
    system_clear_exec_alarm(); // Clear alarm
  }

  if(sys_rt_pen_motion || sys.state==STATE_ALARM) {
    pen_rt_move();
  }

  if(bit_istrue(presed_control_pins,bit(CONTROL_HOMING_BIT))) {
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE) && 
        (sys.state == STATE_IDLE || sys.state == STATE_ALARM)) {
      sys.state = STATE_HOMING; // Set system state variable
      mc_homing_cycle(HOMING_CYCLE_ALL);
      if (!sys.abort) {  // Execute startup scripts after successful homing.
        sys.state = STATE_IDLE; // Set to IDLE when complete.
        st_go_idle(); // Set steppers to the settings idle state before returning.
      }
    }
    bit_false(presed_control_pins,bit(CONTROL_HOMING_BIT));
  }

  rt_exec = sys_rt_exec_state; // Copy volatile sys_rt_exec_state.
  if (rt_exec) {

    // Execute system abort.
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // Only place this is set true.
      return; // Nothing else to do but exit.
    }

    // Execute and serial print status
    if (rt_exec & EXEC_STATUS_REPORT) {
      led_update();
      report_realtime_status();
      system_clear_exec_state_flag(EXEC_STATUS_REPORT);
    }

    // NOTE: Once hold is initiated, the system immediately enters a suspend state to block all
    // main program processes until either reset or resumed. This ensures a hold completes safely.
    if (rt_exec & (EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SLEEP)) {

      // State check for allowable states for hold methods.
      if (!(sys.state & (STATE_ALARM | STATE_CHECK_MODE))) {
      
        // If in CYCLE or JOG states, immediately initiate a motion HOLD.
        if (sys.state & (STATE_CYCLE | STATE_JOG)) {
          if (!(sys.suspend & (SUSPEND_MOTION_CANCEL | SUSPEND_JOG_CANCEL))) { // Block, if already holding.
            st_update_plan_block_parameters(); // Notify stepper module to recompute for hold deceleration.
            sys.step_control = STEP_CONTROL_EXECUTE_HOLD; // Initiate suspend state with active flag.
            if (sys.state == STATE_JOG) { // Jog cancelled upon any hold event, except for sleeping.
              if (!(rt_exec & EXEC_SLEEP)) { sys.suspend |= SUSPEND_JOG_CANCEL; } 
            }
          }
        }
        // If IDLE, Grbl is not in motion. Simply indicate suspend state and hold is complete.
        if (sys.state == STATE_IDLE) { sys.suspend = SUSPEND_HOLD_COMPLETE; }

        // Execute and flag a motion cancel with deceleration and return to idle. Used primarily by probing cycle
        // to halt and cancel the remainder of the motion.
        if (rt_exec & EXEC_MOTION_CANCEL) {
          // MOTION_CANCEL only occurs during a CYCLE, but a HOLD and SAFETY_DOOR may been initiated beforehand
          // to hold the CYCLE. Motion cancel is valid for a single planner block motion only, while jog cancel
          // will handle and clear multiple planner block motions.
          if (!(sys.state & STATE_JOG)) { sys.suspend |= SUSPEND_MOTION_CANCEL; } // NOTE: State is STATE_CYCLE.
        }

        // Execute a feed hold with deceleration, if required. Then, suspend system.
        if (rt_exec & EXEC_FEED_HOLD) {
          // Block JOG, and SLEEP states from changing to HOLD state.
          if (!(sys.state & (STATE_JOG | STATE_SLEEP))) { sys.state = STATE_HOLD; }
        }

      }

      if (rt_exec & EXEC_SLEEP) {
        if (sys.state == STATE_ALARM) { sys.suspend |= (SUSPEND_RETRACT_COMPLETE|SUSPEND_HOLD_COMPLETE); }
        sys.state = STATE_SLEEP; 
      }

      system_clear_exec_state_flag((EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SLEEP));
    }

    // Execute paper load and set WCO
    if (rt_exec & EXEC_PAPER_LOAD) {
      system_clear_exec_state_flag(EXEC_PAPER_LOAD);

      pl_cycle();
    }

    // Execute a cycle start by starting the stepper interrupt to begin executing the blocks in queue.
    if (rt_exec & EXEC_CYCLE_START) {
      // Block if called at same time as the hold commands: feed hold, motion cancel, and safety door.
      // Ensures auto-cycle-start doesn't resume a hold without an explicit user-input.
      if (!(rt_exec & (EXEC_FEED_HOLD | EXEC_MOTION_CANCEL))) {
        // Cycle start only when IDLE or when a hold is complete and ready to resume.
        if ((sys.state == STATE_IDLE) || ((sys.state & STATE_HOLD) && (sys.suspend & SUSPEND_HOLD_COMPLETE))) {
          if(sys.suspend && bit_isfalse(sys.suspend,SUSPEND_RESTORE_COMPLETE)) {
            sys.suspend |= SUSPEND_INITIATE_RESTORE;
          }
          else {
            // Start cycle only if queued motions exist in planner buffer and the motion is not canceled.
            sys.step_control = STEP_CONTROL_NORMAL_OP; // Restore step control to normal operation
            if (plan_get_current_block() && bit_isfalse(sys.suspend,SUSPEND_MOTION_CANCEL)) {
              sys.suspend = SUSPEND_DISABLE; // Break suspend state.
              sys.state = STATE_CYCLE;
              st_prep_buffer(); // Initialize step segment buffer before beginning cycle.
              st_wake_up();
            } else { // Otherwise, do nothing. Set and resume IDLE state.
              sys.suspend = SUSPEND_DISABLE; // Break suspend state.
              sys.state = STATE_IDLE;
            }
          }
        }
      }
      system_clear_exec_state_flag(EXEC_CYCLE_START);
    }

    if (rt_exec & EXEC_CYCLE_STOP) {
      // Reinitializes the cycle plan and stepper system after a feed hold for a resume. Called by
      // realtime command execution in the main program, ensuring that the planner re-plans safely.
      // NOTE: Bresenham algorithm variables are still maintained through both the planner and stepper
      // cycle reinitializations. The stepper path should continue exactly as if nothing has happened.
      // NOTE: EXEC_CYCLE_STOP is set by the stepper subsystem when a cycle or feed hold completes.
      if ((sys.state & (STATE_HOLD|STATE_SLEEP)) && !(sys.soft_limit) && !(sys.suspend & SUSPEND_JOG_CANCEL)) {
        // Hold complete. Set to indicate ready to resume.  Remain in HOLD or DOOR states until user
        // has issued a resume command or reset.
        plan_cycle_reinitialize();
        if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { sys.suspend |= SUSPEND_HOLD_COMPLETE; }
        bit_false(sys.step_control,(STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION));
      } else {
        // Motion complete. Includes CYCLE/JOG/HOMING states and jog cancel/motion cancel/soft limit events.
        // NOTE: Motion and jog cancel both immediately return to idle after the hold completes.
        if (sys.suspend & SUSPEND_JOG_CANCEL) {   // For jog cancel, flush buffers and sync positions.
          sys.step_control = STEP_CONTROL_NORMAL_OP;
          plan_reset();
          st_reset();
          gc_sync_position();
          plan_sync_position();
        }
        sys.suspend = SUSPEND_DISABLE;
        sys.state = STATE_IDLE;
      }
      system_clear_exec_state_flag(EXEC_CYCLE_STOP);
    }
  }

  // Execute overrides.
  rt_exec = sys_rt_exec_motion_override; // Copy volatile sys_rt_exec_motion_override
  if (rt_exec) {
    system_clear_exec_motion_overrides(); // Clear all motion override flags.

    uint8_t new_f_override =  sys.f_override;
    if (rt_exec & EXEC_FEED_OVR_RESET) { new_f_override = DEFAULT_FEED_OVERRIDE; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_PLUS) { new_f_override += FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_MINUS) { new_f_override -= FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_PLUS) { new_f_override += FEED_OVERRIDE_FINE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_MINUS) { new_f_override -= FEED_OVERRIDE_FINE_INCREMENT; }
    new_f_override = min(new_f_override,MAX_FEED_RATE_OVERRIDE);
    new_f_override = max(new_f_override,MIN_FEED_RATE_OVERRIDE);

    uint8_t new_r_override = sys.r_override;
    if (rt_exec & EXEC_RAPID_OVR_RESET) { new_r_override = DEFAULT_RAPID_OVERRIDE; }
    if (rt_exec & EXEC_RAPID_OVR_MEDIUM) { new_r_override = RAPID_OVERRIDE_MEDIUM; }
    if (rt_exec & EXEC_RAPID_OVR_LOW) { new_r_override = RAPID_OVERRIDE_LOW; }

    if ((new_f_override != sys.f_override) || (new_r_override != sys.r_override)) {
      sys.f_override = new_f_override;
      sys.r_override = new_r_override;
      sys.report_ovr_counter = 0; // Set to report change immediately
      plan_update_velocity_profile_parameters();
      plan_cycle_reinitialize();
    }
  }

  #ifdef DEBUG
    if (sys_rt_exec_debug) {
      report_realtime_debug();
      sys_rt_exec_debug = 0;
    }
  #endif

  // Reload step segment buffer
  if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_HOMING | STATE_SLEEP| STATE_JOG)) {
    st_prep_buffer();
  }

}


// Handles Grbl system suspend procedures, such as feed hold, safety door, and parking motion.
// The system will enter this loop, create local variables for suspend tasks, and return to
// whatever function that invoked the suspend, such that Grbl resumes normal operation.
// This function is written in a way to promote custom parking motions. Simply use this as a
// template
static void protocol_exec_rt_suspend()
{
  #ifdef PARKING_ENABLE
    // Declare and initialize parking local variables
    float restore_target[N_AXIS];
    float parking_target[N_AXIS];
    float pen_position;
    plan_line_data_t plan_data;
    plan_line_data_t *pl_data = &plan_data;
    memset(pl_data,0,sizeof(plan_line_data_t));
    pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
    pl_data->feed_rate = settings.homing_seek_rate;
    #ifdef USE_LINE_NUMBERS
      pl_data->line_number = PARKING_MOTION_LINE_NUMBER;
    #endif
  #endif

  plan_block_t *block = plan_get_current_block(); 
  uint8_t restore_condition = 0;
  if (block != NULL) { 
    restore_condition = block->condition; 
  }


  while (sys.suspend) {

    if (sys.abort) { return; }

    // Block until initial hold is complete and the machine has stopped motion.
    if (sys.suspend & SUSPEND_HOLD_COMPLETE) {
      // Parking manager. Handles de/re-energizing, switch state checks, and parking motions for 
      // sleep states.

      // Handles retraction motions and de-energizing.
      if (bit_isfalse(sys.suspend,SUSPEND_RETRACT_COMPLETE)) {

        #ifdef PARKING_ENABLE
          // Get current position and store restore location and spindle retract waypoint.
          system_convert_array_steps_to_mpos(parking_target,sys_position);
          parking_target[X_AXIS] -= settings.tool_x_offset[sys_tool];
          memcpy(restore_target,parking_target,sizeof(parking_target));

          // Execute slow pull-out parking retract motion. Parking requires homing enabled, the
          // current location not exceeding the parking target location, and laser mode disabled.
          // NOTE: State is will remain DOOR, until the de-energizing and retract is complete.
          if ((bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) &&
                          (parking_target[PARKING_AXIS] < -settings.homing_pulloff)) {
            // move pen up
            pen_position = parking_target[Z_AXIS];
            // important to move target exacly equals 10 sys_step on Z_AXIS
            parking_target[Z_AXIS] = PEN_UP_POSITION;
            mc_parking_motion(parking_target, pl_data);
          }
        #endif
        sys.suspend |= SUSPEND_RETRACT_COMPLETE;
      } 
      // here homming button is teated as parking request
      if(bit_istrue(presed_control_pins,bit(CONTROL_HOMING_BIT))) {
          parking_target[PARKING_AXIS] = -PARKING_X_POS;
          mc_parking_motion(parking_target, pl_data);
      }

      // Handles parking restore and safety door resume.
      if (sys.suspend & SUSPEND_INITIATE_RESTORE) {
        #ifdef PARKING_ENABLE
          // Execute fast restore motion to the pull-out position. Parking requires homing enabled.
          if ((settings.flags & (BITFLAG_HOMING_ENABLE|BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
            // Check to ensure the motion doesn't move below pull-out position.
            if (restore_target[PARKING_AXIS] <= -settings.homing_pulloff) {
              // ensure pen is up
              // important to move target exacly equals 10
              parking_target[Z_AXIS] = PEN_UP_POSITION;
              mc_parking_motion(parking_target, pl_data);
              // move over saved XY position
              // important to move target exacly equals 10
              restore_target[Z_AXIS] = PEN_UP_POSITION;
              mc_parking_motion(restore_target, pl_data);
              // restore pen position
              restore_target[Z_AXIS] = pen_position;
              mc_parking_motion(restore_target, pl_data);
            }
          }
        #endif
        sys.suspend &= ~(SUSPEND_INITIATE_RESTORE);
        sys.suspend |= SUSPEND_RESTORE_COMPLETE;
        system_set_exec_state_flag(EXEC_CYCLE_START); // Set to resume program.
      }
    } 

    protocol_exec_rt_system();
  }

}
